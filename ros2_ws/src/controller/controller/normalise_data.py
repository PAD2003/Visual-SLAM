import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sklearn import linear_model
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
import struct

class FloorDetectionNode(Node):
    def __init__(self):
        super().__init__('detect_floor_node')
        
        self.transform_matrix = None
        self.trajectory_points = []
        self.good = False
        self.target_z = 0.67 # height of your robot

        # Subscriber and Publisher
        self.path_sub = self.create_subscription(Path, '/vslam/path', self.path_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/vslam/pointcloud', self.pointcloud_callback, 10)
        self.marker_sub = self.create_subscription(MarkerArray, '/vslam/human_model_mesh', self.marker_callback, 10)
        
        self.trajectory_publisher = self.create_publisher(Path, '/transformed/path', 10)
        self.pc_publisher = self.create_publisher(PointCloud2, '/transformed/pointcloud', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/transformed/human_model_mesh', 10)

        # Timer
        self.timer_1 = self.create_timer(1.0, self.detect_and_publish_plane)
        
    def path_callback(self, msg):
        trans_msg = Path()
        trans_msg.header.frame_id = msg.header.frame_id
        trans_msg.header.stamp = msg.header.stamp
        
        if self.transform_matrix is not None:
            self.good = True
        
        # Apply transform for all points in trajectory
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            self.trajectory_points.append([x, y, z])
            
            if self.transform_matrix is not None:
                homogeneous_point = np.array([x, -y, -z, 1])
                transformed_point = np.dot(self.transform_matrix, homogeneous_point)

                trans_pose = PoseStamped()
                trans_pose.pose.position.x = transformed_point[0]
                trans_pose.pose.position.y = transformed_point[1]
                trans_pose.pose.position.z = transformed_point[2]
                trans_pose.pose.orientation.w = 1.0
                
                if abs(transformed_point[2] - self.target_z) > 0.2:
                    self.good = False
                
                trans_msg.poses.append(trans_pose)
        
        self.trajectory_publisher.publish(trans_msg)
        
    def pointcloud_callback(self, msg):
        if self.transform_matrix is not None:
            # Convert PointCloud2 to numpy array for easier processing
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], -point[1], -point[2], 1.0])
            
            if len(points) == 0:
                return

            # Convert to numpy array
            points_homogeneous = np.array(points).T

            # Apply transform to pointcloud data
            transformed_points_homogeneous = np.dot(self.transform_matrix, points_homogeneous).T

            # Remove the homogeneous coordinate and keep (x, y, z)
            transformed_points = transformed_points_homogeneous[:, :3]

            # Convert transformed points back to PointCloud2 message to publish
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id  # Keep original frame_id from the input topic
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            
            cloud_data = []
            for i in range(len(transformed_points)):
                x, y, z = transformed_points[i]
                intensity = 1.0
                packed_data = struct.pack('ffff', x, y, z, intensity)
                cloud_data.append(packed_data)

            # Join all binary data into a single byte string
            cloud_data = b''.join(cloud_data)
            
            transformed_cloud_msg = PointCloud2()
            transformed_cloud_msg.header = header
            transformed_cloud_msg.height = 1
            transformed_cloud_msg.width = transformed_points.shape[0]
            transformed_cloud_msg.is_dense = False
            transformed_cloud_msg.is_bigendian = False
            transformed_cloud_msg.fields = fields
            transformed_cloud_msg.point_step = 16
            transformed_cloud_msg.row_step = transformed_cloud_msg.point_step * transformed_points.shape[0]
            transformed_cloud_msg.data = cloud_data

            # Publish transformed pointcloud
            self.pc_publisher.publish(transformed_cloud_msg)

    def marker_callback(self, msg):
        if self.transform_matrix is not None:
            transformed_markers = MarkerArray()
            for marker in msg.markers:
                transformed_marker = Marker()
                transformed_marker.header = marker.header
                transformed_marker.ns = marker.ns
                transformed_marker.id = marker.id
                transformed_marker.type = marker.type
                transformed_marker.action = marker.action
                transformed_marker.scale = marker.scale
                transformed_marker.color = marker.color
                transformed_marker.lifetime = marker.lifetime
                transformed_marker.frame_locked = marker.frame_locked
                
                # Apply transformation to marker position
                homogeneous_point = np.array([[marker.pose.position.x, -marker.pose.position.y, -marker.pose.position.z, 1.0]]).T
                transformed_points_homogeneous = np.dot(self.transform_matrix, homogeneous_point).T
                
                # Remove the homogeneous coordinate and keep (x, y, z)
                transformed_points = transformed_points_homogeneous[:, :3]

                transformed_marker.pose.position.x = transformed_points[0][0]
                transformed_marker.pose.position.y = transformed_points[0][1]
                transformed_marker.pose.position.z = transformed_points[0][2]
                transformed_marker.pose.orientation = marker.pose.orientation
                
                transformed_markers.markers.append(transformed_marker)
            
            # Publish transformed markers if needed (assuming there is a publisher for markers)
            self.marker_publisher.publish(transformed_markers)

    def detect_and_publish_plane(self):
        if self.good:
            return
        
        if len(self.trajectory_points) < 10:
            self.get_logger().warn("Not enough trajectory points to fit a plane")
            return

        # Convert points to a numpy array
        points = np.array(self.trajectory_points)

        # RANSAC plane fitting
        ransac = linear_model.RANSACRegressor()
        ransac.fit(points[:, :2], points[:, 2])  # Fit x, y to z (z = ax + by + d)

        # Coefficients of the plane
        a, b = ransac.estimator_.coef_
        d = ransac.estimator_.intercept_
        self.get_logger().info(f"Detected plane equation: z = {a:.3f}*x + {b:.3f}*y + {d:.3f}")

        # Calculate transform to align plane to z = target_z
        transform_matrix = self.calculate_transform_to_plane(a, b, -1, d)

        # Apply transform to trajectory and publish
        self.transform_matrix = transform_matrix

    def calculate_transform_to_plane(self, a, b, c, d):
        # Step 1: Calculate the normal vector of the current plane
        normal_current = np.array([a, b, c])
        normal_target = np.array([0, 0, 1])  # Normal vector of target plane z = target_z
        
        # Step 2: Calculate rotation angle and axis between current normal and target normal
        normal_current_norm = normal_current / np.linalg.norm(normal_current)
        
        axis_of_rotation = np.cross(normal_current_norm, normal_target)
        angle_of_rotation = np.arccos(np.dot(normal_current_norm, normal_target))
        
        # Step 3: Calculate quaternion from angle and axis of rotation
        if np.linalg.norm(axis_of_rotation) > 0:
            rotation = R.from_rotvec(angle_of_rotation * axis_of_rotation / np.linalg.norm(axis_of_rotation))
        else:
            rotation = R.identity()  # No rotation needed
        
        rotation_matrix = rotation.as_matrix()
        
        # Step 4: Calculate translation to align plane to target z
        translation_z = self.target_z - d / c
        
        # Create 4x4 translation matrix
        translation_matrix = np.eye(4)
        translation_matrix[2, 3] = translation_z
        
        # Create 4x4 rotation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        
        # Step 5: Combine rotation and translation matrices
        final_transform = np.dot(translation_matrix, transform_matrix)
        
        return final_transform

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = FloorDetectionNode()

    # Spin the node to process callbacks
    rclpy.spin(node)

    # Shutdown when the node is killed
    rclpy.shutdown()

if __name__ == '__main__':
    main()