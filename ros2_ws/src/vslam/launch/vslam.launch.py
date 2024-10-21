from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbslam3',
            executable='rgbd',
            output='screen',
            parameters=[
                {'vocalbulary_path': '/mnt/SSD128/tungpt/Workspace/Photo-SLAM/ORB-SLAM3/Vocabulary/ORBvoc.txt'},
                {'setting_path': '/mnt/SSD128/tungpt/Workspace/Photo-SLAM/cfg/ORB_SLAM3/RGB-D/RealCamera/realsense_d415_rgbd.yaml'},
                {'gaussian_setting_path': '/mnt/SSD128/tungpt/Workspace/Photo-SLAM/cfg/gaussian_mapper/RGB-D/RealCamera/realsense_rgbd.yaml'},
                {'output_dir': '/mnt/SSD128/tungpt/Workspace/Runtime/outputs/realsense_d415_rgbd'}
            ]
        ),
        Node(
            package='lifa_controller',
            executable='detect_floor',
            name='detect_floor'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_show',
            output='screen'
        ),
    ])