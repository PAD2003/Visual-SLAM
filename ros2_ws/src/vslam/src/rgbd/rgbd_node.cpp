#include "rgbd_node.h"

RGBDNode::RGBDNode(string vocalbularyPath, string settingPath, string gaussianSettingPath, string outputDir): Node("PhotoSLAM_ROS2") {}

RGBDNode::RGBDNode(): Node("PhotoSLAM_ROS2")
{
    // TODO: set path to masked-photo-slam
    this->declare_parameter("vocalbulary_path", "/home/ducpa/Project/slam3d/Photo-SLAM/ORB-SLAM3/Vocabulary/ORBvoc.txt");
    this->declare_parameter("setting_path", "/home/ducpa/Project/slam3d/Photo-SLAM/cfg/ORB_SLAM3/RGB-D/RealCamera/realsense_d415_rgbd.yaml");
    this->declare_parameter("gaussian_setting_path", "/home/ducpa/Project/slam3d/Photo-SLAM/cfg/gaussian_mapper/RGB-D/RealCamera/realsense_rgbd.yaml");
    this->declare_parameter("output_dir", "~/output");

    this->initializedVSLAM();

    pcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("vslam/pointcloud", 10);
    dense_pcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("vslam/densepointcloud", 10);
    lmp_ref_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("vslam/localpointcloud", 10);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("vslam/path", 10);
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("vslam/robot_pose", 10);
    mark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vslam/human_model_mesh", 10);

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/rgb");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/depth");
    seg_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/segment");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RGBDNode::GrabRGBDS, this);

    // set path to ros2_ws/src/vslam/src/segment/yolov8n-seg.engine
    segmentModel_.init("/home/ducpa/ros2_ws/src/vslam/src/segment/yolov8n-seg.engine", 1);
    segmentModel_.setInputImageFormat(tpt::IFormat::RGB);
}

RGBDNode::~RGBDNode()
{
    //stop all threads
    pSLAM->Shutdown();
    training_thd.join();
    if (use_viewer)
        viewer_thd.join();
}

void RGBDNode::initializedVSLAM()
{
    std::string vocalbularyPath = this->get_parameter("vocalbulary_path").as_string();
    std::string settingPath = this->get_parameter("setting_path").as_string();
    std::string gaussianSettingPath = this->get_parameter("gaussian_setting_path").as_string();
    std::string outputDir = this->get_parameter("output_dir").as_string();

    if (torch::cuda::is_available())
    {
        std::cout << "CUDA available! Training on GPU." << std::endl;
        device_type = torch::kCUDA;
    }
    else
    {
        std::cout << "Training on CPU." << std::endl;
        device_type = torch::kCPU;
    }

    if (outputDir.back() != '/')
        outputDir += "/";
    output_dir = outputDir;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    pSLAM = std::make_shared<ORB_SLAM3::System>(
                vocalbularyPath, settingPath, ORB_SLAM3::System::RGBD, 0, "realsense_rgbd");
    imageScale = pSLAM->GetImageScale();

    // Create GaussianMapper
    std::filesystem::path gaussian_cfg_path(gaussianSettingPath);

    pGausMapper = std::make_shared<GaussianMapper>(
            pSLAM, gaussian_cfg_path, output_dir, 0, device_type);
    training_thd = std::thread(&GaussianMapper::run, pGausMapper.get());

    if (use_viewer)
    {
        pViewer = std::make_shared<ImGuiViewer>(pSLAM, pGausMapper);
        viewer_thd = std::thread(&ImGuiViewer::run, pViewer.get());
    }
}

void RGBDNode::GrabRGBDS(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void *)(cv_ptrRGB->image.ptr(0)), cv::Mat::AUTO_STEP);
    cv::Mat depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void *)(cv_ptrD->image.ptr(0)), cv::Mat::AUTO_STEP);

    // Infer yolo segmentim
    std::vector<std::vector<tpt::objectsegment::OutputSeg>> segmented;
    segmentModel_.infer(std::vector<cv::Mat>{im}, segmented);
    std::vector<tpt::objectsegment::OutputSeg> resSegment = segmented.at(0);
    cv::Mat segment = segmentModel_.drawMaskPred(im, resSegment);

    // Pass the image to the SLAM system
    Sophus::SE3f camera_pose_SE3 = pSLAM->TrackRGBD(segment, im, depth, Utility::StampToSec(msgRGB->header.stamp),
                     std::vector<ORB_SLAM3::IMU::Point>(), std::to_string(Utility::StampToSec(msgRGB->header.stamp)));

    this->publishCurrentFramePointCloud(msgRGB->header.stamp);
    this->publishPointClould(msgRGB->header.stamp);
    this->publishTrajectory(msgRGB->header.stamp);
    this->publishPose(camera_pose_SE3, msgRGB->header.stamp);
    segment_publish(im, depth, resSegment);
}

void RGBDNode::publishCurrentFramePointCloud(builtin_interfaces::msg::Time stamp)
{
    std::vector<ORB_SLAM3::MapPoint*> local_mps = pSLAM->getTracker()->mCurrentFrame.mvpMapPoints;
    // Create a PointCloud2 message
    sensor_msgs::msg::PointCloud2 point_cloud_msg;

    // Define the size of the point cloud
    const size_t num_points = local_mps.size();
    point_cloud_msg.header.stamp = stamp;
    point_cloud_msg.header.frame_id = "map";  // The frame ID for the point cloud
    point_cloud_msg.height = 1;
    point_cloud_msg.width = static_cast<int>(num_points);
    point_cloud_msg.is_dense = false;
    point_cloud_msg.is_bigendian = false;

    // Define the fields of the PointCloud2 message (x, y, z)
    sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);

    // Populate the point cloud data with map points from ORB-SLAM3
    sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");

    for (auto map_point : local_mps)
    {
        if (map_point)
        {
            Eigen::Matrix<float,3,1> world_pos = map_point->GetWorldPos();  // Get 3D world position of the map point

            // Set the (x, y, z) coordinates in the point cloud
            *iter_x = world_pos(0);
            *iter_y = world_pos(1);
            *iter_z = world_pos(2);

            // Increment the iterators to the next point
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }

    // Publish the point cloud message
    lmp_ref_pub->publish(point_cloud_msg);
}

void RGBDNode::publishPointClould(builtin_interfaces::msg::Time stamp)
{
    // Create a PointCloud2 message
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Fill header information
    cloud_msg->header.stamp = stamp;
    cloud_msg->header.frame_id = "map";  // Frame of reference for the point cloud

    std::vector<std::vector<float>> v_pcloud;
    this->getPointCloud(v_pcloud);

    // ---------------------------
    // push map point cloud
    // ---------------------------
    size_t num_points = v_pcloud.size();

    // Set the fields (x, y, z, intensity) for the point cloud
    cloud_msg->height = 1;  // Unordered point cloud
    cloud_msg->width = static_cast<int>(num_points);
    cloud_msg->is_dense = false;  // True if there are no invalid points

    // Define the fields for PointCloud2 message
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    // Fill point cloud data
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");

    for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
    {
        // Fill point data with dummy values
        *iter_x = v_pcloud.at(i)[0];
        *iter_y = v_pcloud.at(i)[1];
        *iter_z = v_pcloud.at(i)[2];

        // Assign color values (red for illustration purposes)
        *iter_r = 255;
        *iter_g = 0;
        *iter_b = 0;
    }

    // Publish the point cloud
    pcloud_pub->publish(*cloud_msg);

    // ---------------------------
    // push reference point cloud
    // ---------------------------
#if false
    auto ref_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    // Fill header information
    ref_cloud_msg->header.stamp = stamp;
    ref_cloud_msg->header.frame_id = "map";  // Frame of reference for the point cloud

    size_t ref_num_points = v_ref_pcloud.size();

    // Set the fields (x, y, z, intensity) for the point cloud
    ref_cloud_msg->height = 1;  // Unordered point cloud
    ref_cloud_msg->width = static_cast<int>(ref_num_points);
    ref_cloud_msg->is_dense = false;  // True if there are no invalid points

    // Define the fields for PointCloud2 message
    sensor_msgs::PointCloud2Modifier ref_modifier(*ref_cloud_msg);
    ref_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    // Fill point cloud data
    sensor_msgs::PointCloud2Iterator<float> ref_iter_x(*ref_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> ref_iter_y(*ref_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> ref_iter_z(*ref_cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> ref_iter_r(*ref_cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> ref_iter_g(*ref_cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> ref_iter_b(*ref_cloud_msg, "b");

    for (size_t i = 0; i < ref_num_points; ++i, ++ref_iter_x, ++ref_iter_y, ++ref_iter_z, ++ref_iter_r, ++ref_iter_g, ++ref_iter_b)
    {
        // Fill point data with dummy values
        *ref_iter_x = v_ref_pcloud.at(i)[0];
        *ref_iter_y = v_ref_pcloud.at(i)[1];
        *ref_iter_z = v_ref_pcloud.at(i)[2];

        // Assign color values (red for illustration purposes)
        *ref_iter_r = 0;
        *ref_iter_g = 255;
        *ref_iter_b = 0;
    }

    // Publish the reference point cloud
    pcloud_ref_pub->publish(*ref_cloud_msg);
#endif
}

void RGBDNode::publishTrajectory(builtin_interfaces::msg::Time stamp)
{
    // Get the current trajectory from ORB-SLAM3
        std::vector<Sophus::SE3f> trajectory = this->getCurrentTrajectory();

        // Create a Path message
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "map";  // Adjust the frame_id as needed

        // Convert trajectory to ROS2 Path message
        for (const auto& pose_sophus : trajectory)
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = path_msg.header;

            // Convert Sophus::SE3f to geometry_msgs Pose
            Eigen::Matrix3f rotation_matrix = pose_sophus.rotationMatrix();
            Eigen::Vector3f translation = pose_sophus.translation();

            pose_msg.pose.position.x = translation.x();
            pose_msg.pose.position.y = translation.y();
            pose_msg.pose.position.z = translation.z();

            // Convert rotation matrix to quaternion
            Eigen::Quaternionf q(rotation_matrix);
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose_msg);
        }

        // Publish the Path message
        path_pub->publish(path_msg);
}

void RGBDNode::publishPose(Sophus::SE3f camera_pose_SE3, builtin_interfaces::msg::Time stamp)
{
    // Convert to geometry_msgs::msg::PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";  // Frame of reference

    // Extract translation and rotation from Sophus::SE3f
    Eigen::Matrix<float, 3, 1> translation = camera_pose_SE3.translation();
    Eigen::Quaternionf quaternion(camera_pose_SE3.unit_quaternion());

    pose_msg.pose.position.x = translation[0];
    pose_msg.pose.position.y = translation[1];
    pose_msg.pose.position.z = translation[2];
    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();
    pose_msg.pose.orientation.w = quaternion.w();

    // Publish the pose
    pose_pub->publish(pose_msg);
}

void RGBDNode::publishMarker(cv::Mat segImg, cv::Mat depthImg, builtin_interfaces::msg::Time stamp)
{
     ORB_SLAM3::Frame& current_frame = pSLAM->getTracker()->mCurrentFrame;
     float fx = current_frame.fx;
     float fy = current_frame.fy;
     float cx = current_frame.cx;
     float cy = current_frame.cy;

     Sophus::SE3f Tcw_SE3 = current_frame.GetPose();
     // Convert Sophus::SE3 to a 4x4 transformation matrix (Eigen::Matrix4f)
     Eigen::Matrix4f Tcw = Tcw_SE3.matrix();
     // Inverse to get the transformation from camera coordinates to world coordinates
     Eigen::Matrix4f Twc = Tcw.inverse();


    // Create PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < height_img; ++i) {
        for (int j = 0; j < width_img; ++j) {
            if (segImg.at<uchar>(i, j) > 0) {  // If the pixel is part of the human
                uint16_t depth = depthImg.at<uint16_t>(i, j);
                if (depth == 0) continue;  // Skip invalid depth

                float z = depth / 1000.0f; // Convert mm to meters
                float x = (j - cx) * z / fx;
                float y = (i - cy) * z / fy;

                // Create a 4x1 homogeneous point in the camera frame
                Eigen::Vector4f point_cam(x, y, z, 1.0f);

                // Transform the point from the camera frame to the local map frame using Twc
                Eigen::Vector4f point_world = Twc * point_cam;

                cloud->points.emplace_back(point_world.x(), point_world.y(), point_world.z());
            }
        }
    }

    // Create marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";  // Adjust frame as needed
    marker.header.stamp = stamp;
    marker.ns = "human_model_mesh";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    // Set mesh color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    if (cloud->empty()) {
        return;
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    // Mesh generation using PCL (Greedy Projection Triangulation)
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    normal_estimation.setSearchMethod(tree);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setKSearch(20);
    normal_estimation.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);        // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Convert mesh to ROS Marker
    // Convert triangles to geometry_msgs/Point
    pcl::PointCloud<pcl::PointXYZ> cloud_mesh;
    pcl::fromPCLPointCloud2(triangles.cloud, cloud_mesh);

    for (const auto& polygon : triangles.polygons) {
        for (const auto& vertex : polygon.vertices) {
            geometry_msgs::msg::Point point;
            point.x = cloud_mesh.points[vertex].x;
            point.y = cloud_mesh.points[vertex].y;
            point.z = cloud_mesh.points[vertex].z;
            marker.points.push_back(point);
        }
    }

    // Set scale
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
}

void RGBDNode::publishCurrentDensePointCloud(cv::Mat depthImg, builtin_interfaces::msg::Time stamp)
{
    std::vector<Eigen::Vector3f> dense_point_cloud = this->generateCurrentDensePointCloud(depthImg);
    // Create a PointCloud2 message
    sensor_msgs::msg::PointCloud2 point_cloud_msg;

    // Define the size of the point cloud
    const size_t num_points = dense_point_cloud.size();
    point_cloud_msg.header.stamp = stamp;
    point_cloud_msg.header.frame_id = "map";  // The frame ID for the point cloud
    point_cloud_msg.height = 1;
    point_cloud_msg.width = static_cast<int>(num_points);
    point_cloud_msg.is_dense = false;
    point_cloud_msg.is_bigendian = false;

    // Define the fields of the PointCloud2 message (x, y, z)
    sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(num_points);

    // Populate the point cloud data with map points from ORB-SLAM3
    sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud_msg, "b");

    for (auto point : dense_point_cloud)
    {

        // Set the (x, y, z) coordinates in the point cloud
        *iter_x = point.x();
        *iter_y = point.y();
        *iter_z = point.z();

        // Assign color values (red for illustration purposes)
        *iter_r = 255;
        *iter_g = 255;
        *iter_b = 255;

        // Increment the iterators to the next point
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
    }

    // Publish the point cloud message
    dense_pcloud_pub->publish(point_cloud_msg);
}

void RGBDNode::rvizViewCallback()
{
    // Ensure the service is available
    if (!view_rviz_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for /rviz2/get_parameters service...");
        return;
    }

    // Prepare the request
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("view_controller");

    // Call the service asynchronously
    auto result_future = view_rviz_client->async_send_request(request,
                                                              std::bind(&RGBDNode::rvizResponseHandler, this, std::placeholders::_1));
}

void RGBDNode::rvizResponseHandler(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future)
{
    auto response = future.get();
    std::string current_camera_info = "";

    for (const auto &parameter : response->values)
    {
        if (parameter.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
        {
            current_camera_info = parameter.string_value;
            break;
        }
    }

    // Check if there is a change in the camera information
    if (current_camera_info != last_camera_info_)
    {
        RCLCPP_INFO(this->get_logger(), "Camera information changed: %s", current_camera_info.c_str());

        // Update the last known camera information
        last_camera_info_ = current_camera_info;
    }

}

void RGBDNode::getPointCloud(std::vector<std::vector<float>> &pcloud)
{
    pcloud.clear();

    ORB_SLAM3::Map* pActiveMap = this->pSLAM->getAtlas()->GetCurrentMap();
    if (!pActiveMap) {
        cout << endl << "There is no active map (pActiveMap is null)" << endl;
        return;
    }

    // Vectors containing pointers to MapPoint objects contained in the maps
    // Vector of pointers for Map Points -- vpMPs
    // Vector of pointers for Reference Map Points -- vpRefMPs
    // TODO figure out the difference between Reference Map Points and normal Map Points
    const vector<ORB_SLAM3::MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<ORB_SLAM3::MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    // Use a set for fast lookup of reference frames
    set<ORB_SLAM3::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    // Iterate over map points, skip "bad" ones and reference map points
    for (size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        float float_array[3];
        std::memcpy(float_array, pos.data(), 3 * sizeof(float));
        std::vector<float> v_array = {float_array[0], float_array[1], float_array[2]};
        pcloud.push_back(v_array);
    }

#if false
    std::vector<std::vector<float>> ref_pcloud;
    for (size_t i=0, iend=vpRefMPs.size(); i<iend;i++) {
        if (vpRefMPs[i]->isBad()){
            continue;
        }
        Eigen::Matrix<float,3,1> pos = vpRefMPs[i]->GetWorldPos();
        float float_array[3];
        std::memcpy(float_array, pos.data(), 3 * sizeof(float));
        std::vector<float> v_array = {float_array[0], float_array[1], float_array[2]};
        ref_pcloud.push_back(v_array);
    }
#endif
}

std::vector<Sophus::SE3f> RGBDNode::getCurrentTrajectory()
{
    std::vector<Sophus::SE3f> trajectory;

    // Retrieve all keyframes
    std::vector<ORB_SLAM3::KeyFrame*> keyframes = this->pSLAM->getAtlas()->GetAllKeyFrames();

    // Sort the keyframes by their ID to ensure the trajectory is in order
    std::sort(keyframes.begin(), keyframes.end(), ORB_SLAM3::KeyFrame::lId);

    // Loop through the keyframes and store their poses in the trajectory vector
    for (auto kf : keyframes) {
        if (kf->isBad()) continue;  // Skip bad keyframes
        Sophus::SE3f pose = kf->GetPoseInverse(); // Get the keyframe pose
        trajectory.push_back(pose);
    }

    return trajectory;
}

std::vector<Eigen::Vector3f> RGBDNode::generateCurrentDensePointCloud(cv::Mat depthImg)
{
    std::vector<Eigen::Vector3f> dense_point_cloud;
    ORB_SLAM3::Frame& current_frame = pSLAM->getTracker()->mCurrentFrame;
    float fx = current_frame.fx;
    float fy = current_frame.fy;
    float cx = current_frame.cx;
    float cy = current_frame.cy;

    Sophus::SE3f Tcw_SE3 = current_frame.GetPose();
    // Convert Sophus::SE3 to a 4x4 transformation matrix (Eigen::Matrix4f)
    Eigen::Matrix4f Tcw = Tcw_SE3.matrix();
    // Inverse to get the transformation from camera coordinates to world coordinates
    Eigen::Matrix4f Twc = Tcw.inverse();

    // Loop through the depth map and back-project each pixel into 3D
    for (int i = 0; i < depthImg.rows; ++i)
    {
        for (int j = 0; j < depthImg.cols; ++j)
        {
            // Get the depth value for the current pixel
            float depth = depthImg.at<float>(i, j);

            // Ignore invalid depth values
            if (depth > 0)
            {
                // Back-project the pixel (x, y) to 3D coordinates in the camera frame
                float z = depth / 1000.0f;
                float x = (j - cx) * z / fx;
                float y = (i - cy) * z / fy;

                // Create a 4x1 homogeneous point in the camera frame
                Eigen::Vector4f point_cam(x, y, z, 1.0f);

                // Transform the point from the camera frame to the world frame using Twc
                Eigen::Vector4f point_world = Twc * point_cam;

                // Convert to Eigen::Vector3f for easier handling
                Eigen::Vector3f world_pos(point_world.x(), point_world.y(), point_world.z());

                // Add the world position to the dense point cloud
                dense_point_cloud.push_back(world_pos);
            }
        }
    }

    return dense_point_cloud;
}

visualization_msgs::msg::Marker RGBDNode::create_marker(int marker_type,
                             std::array<double, 3> position,
                             std::array<double, 3> scale,
                             std::array<double, 4> color,
                             std::string ns,
                             int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = marker_type;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    marker.pose.orientation.x = 0.7071;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.7071;

    marker.scale.x = scale[0];
    marker.scale.y = scale[1];
    marker.scale.z = scale[2];

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    marker.lifetime = rclcpp::Duration::from_seconds(1);  // Marker stays forever
    return marker;
}

void RGBDNode::publish_human_marker_aligned_X(double human_height_, std::array<double, 3> human_position_, int marker_id, Eigen::Matrix4f Tcw)
{
    visualization_msgs::msg::MarkerArray marker_array;
    // Proportions based on human height
    double head_height = 0.2 * human_height_;
    double body_height = 0.3 * human_height_;
    double leg_height = 0.5 * human_height_;
    double arm_length = 0.4 * human_height_;
    double body_width = 0.3 * human_height_;
    double arm_width = 0.1 * human_height_;
    double leg_width = 0.1 * human_height_;

    // Create a 4x1 homogeneous point in the camera frame
    Eigen::Vector4f c_head_position(human_position_[0] + leg_height + body_height + 0.5 * head_height, human_position_[1], human_position_[2], 1.0f);
    Eigen::Vector4f c_body_position(human_position_[0] + leg_height + 0.5 * body_height, human_position_[1], human_position_[2] , 1.0f);
    Eigen::Vector4f c_right_leg_position(human_position_[0] + 0.5 * leg_height, human_position_[1], human_position_[2] - 0.1 * human_height_, 1.0f);
    Eigen::Vector4f c_left_leg_position(human_position_[0] + 0.5 * leg_height, human_position_[1], human_position_[2] + 0.1 * human_height_, 1.0f);
    Eigen::Vector4f c_right_arm_position(human_position_[0] + leg_height + body_height - 0.5 * arm_length, human_position_[1], human_position_[2] - 0.2 * human_height_, 1.0f);
    Eigen::Vector4f c_left_arm_position(human_position_[0] + leg_height + body_height - 0.5 * arm_length, human_position_[1], human_position_[2] + 0.2 * human_height_, 1.0f);

    // Transform the point from the camera frame to the local map frame using Twc
    Eigen::Vector4f world_head_position = Tcw * c_head_position;
    Eigen::Vector4f world_body_position = Tcw * c_body_position;
    Eigen::Vector4f world_right_leg_position = Tcw * c_right_leg_position;
    Eigen::Vector4f world_left_leg_position = Tcw * c_left_leg_position;
    Eigen::Vector4f world_right_arm_position = Tcw * c_right_arm_position;
    Eigen::Vector4f world_left_arm_position = Tcw * c_left_arm_position;


    // Head (sphere)
    auto head_marker = create_marker(
        visualization_msgs::msg::Marker::SPHERE,
        {world_head_position.x(), world_head_position.y(), world_head_position.z()},  // Head position based on total height
        {head_height, head_height, head_height},  // Head size proportional to height
        {1.0, 0.8, 0.6, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(head_marker);

    // Body (cylinder)
    auto body_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_body_position.x(), world_body_position.y(), world_body_position.z()},  // Body position
        {body_height, body_width, body_width},  // Body diameter and height
        {0.0, 0.0, 1.0, 1.0},  // Blue color for clothes
        "human", marker_id++);
    marker_array.markers.push_back(body_marker);

    // Right leg (cylinder)
    auto right_leg_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_right_leg_position.x(), world_right_leg_position.y(), world_right_leg_position.z()},  // Right leg position
        {leg_height, leg_width, leg_width},  // Leg size proportional to height
        {0.0, 0.0, 0.0, 1.0},  // Black for pants
        "human", marker_id++);
    marker_array.markers.push_back(right_leg_marker);

    // Left leg (cylinder)
    auto left_leg_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_left_leg_position.x(), world_left_leg_position.y(), world_left_leg_position.z()},  // Left leg position
        {leg_height, leg_width, leg_width},  // Leg size proportional to height
        {0.0, 0.0, 0.0, 1.0},  // Black for pants
        "human", marker_id++);
    marker_array.markers.push_back(left_leg_marker);

    // Right arm (cylinder)
    auto right_arm_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_right_arm_position.x(), world_right_arm_position.y(), world_right_arm_position.z()},  // Right arm position
        {arm_length, arm_width, arm_width},  // Arm length proportional to height
        {1.0, 0.8, 0.6, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(right_arm_marker);

    // Left arm (cylinder)
    auto left_arm_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_left_arm_position.x(), world_left_arm_position.y(), world_left_arm_position.z()},  // Left arm position
        {arm_length, arm_width, arm_width},  // Arm length proportional to height
        {1.0, 0.8, 0.6, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(left_arm_marker);

    mark_pub_->publish(marker_array);
}

void RGBDNode::publish_human_marker_aligned_Y(double human_height_, std::array<double, 3> human_position_, int marker_id, Eigen::Matrix4f Tcw)
{
    visualization_msgs::msg::MarkerArray marker_array;

    if (human_height_ >= 0.9 && human_height_ < 1.5) human_height_ = 1.3;
    if (human_height_ >= 1.5) human_height_ = 1.6;
    if (human_height_ < 0.9) human_height_ = 0.7;

    double head_height = 0.2 * human_height_;
    double body_height = 0.3 * human_height_;
    double leg_height = 0.5 * human_height_;
    double arm_length = 0.4 * human_height_;
    double body_width = 0.3 * human_height_;
    double arm_width = 0.1 * human_height_;
    double leg_width = 0.1 * human_height_;

    // Create a 4x1 homogeneous point in the camera frame
    Eigen::Vector4f c_head_position(human_position_[0], (human_position_[1] - leg_height - body_height - 0.5 * head_height), human_position_[2], 1.0f);
    Eigen::Vector4f c_body_position(human_position_[0], (human_position_[1] - leg_height - 0.5 * body_height), human_position_[2], 1.0f);
    Eigen::Vector4f c_right_leg_position(human_position_[0], (human_position_[1] - 0.5 * leg_height), human_position_[2] - 0.1 * human_height_, 1.0f);
    Eigen::Vector4f c_left_leg_position(human_position_[0], (human_position_[1] - 0.5 * leg_height), human_position_[2] + 0.1 * human_height_, 1.0f);
    Eigen::Vector4f c_right_arm_position(human_position_[0], (human_position_[1] - leg_height - body_height + 0.5 * arm_length), human_position_[2] - 0.2 * human_height_, 1.0f);
    Eigen::Vector4f c_left_arm_position(human_position_[0], (human_position_[1] - leg_height - body_height + 0.5 * arm_length), human_position_[2] + 0.2 * human_height_, 1.0f);

    // Transform the point from the camera frame to the local map frame using Twc
    Eigen::Vector4f world_head_position = Tcw * c_head_position;
    Eigen::Vector4f world_body_position = Tcw * c_body_position;
    Eigen::Vector4f world_right_leg_position = Tcw * c_right_leg_position;
    Eigen::Vector4f world_left_leg_position = Tcw * c_left_leg_position;
    Eigen::Vector4f world_right_arm_position = Tcw * c_right_arm_position;
    Eigen::Vector4f world_left_arm_position = Tcw * c_left_arm_position;


    // Head (sphere)
    auto head_marker = create_marker(
        visualization_msgs::msg::Marker::SPHERE,
        {world_head_position.x(), world_head_position.y(), world_head_position.z()},  // Head position based on total height
        {head_height, head_height, head_height},  // Head size proportional to height
        {1.0, 0.8, 0.6, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(head_marker);

    // Body (cylinder)
    auto body_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_body_position.x(), world_body_position.y(), world_body_position.z()},  // Body position
        {body_width, body_width, body_height},  // Body diameter and height
        {1.0, 1.0, 1.0, 1.0},  // Blue color for clothes
        "human", marker_id++);
    marker_array.markers.push_back(body_marker);

    // Right leg (cylinder)
    auto right_leg_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_right_leg_position.x(), world_right_leg_position.y(), world_right_leg_position.z()},  // Right leg position
        {leg_width, leg_width, leg_height},  // Leg size proportional to height
        {1.0, 1.0, 1.0, 1.0},  // Black for pants
        "human", marker_id++);
    marker_array.markers.push_back(right_leg_marker);

    // Left leg (cylinder)
    auto left_leg_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_left_leg_position.x(), world_left_leg_position.y(), world_left_leg_position.z()},  // Left leg position
        {leg_width, leg_width, leg_height},  // Leg size proportional to height
        {1.0, 1.0, 1.0, 1.0},  // Black for pants
        "human", marker_id++);
    marker_array.markers.push_back(left_leg_marker);

    // Right arm (cylinder)
    auto right_arm_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_right_arm_position.x(), world_right_arm_position.y(), world_right_arm_position.z()},  // Right arm position
        {arm_width, arm_width, arm_length},  // Arm length proportional to height
        {1.0, 0.8, 0.6, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(right_arm_marker);

    // Left arm (cylinder)
    auto left_arm_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_left_arm_position.x(), world_left_arm_position.y(), world_left_arm_position.z()},  // Left arm position
        {arm_width, arm_width, arm_length},  // Arm length proportional to height
        {1.0, 0.8, 0.6, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(left_arm_marker);

    mark_pub_->publish(marker_array);
}


void RGBDNode::publish_human_marker(double human_height_, std::array<double, 3> human_position_, int marker_id, Eigen::Matrix4f Tcw)
{
    visualization_msgs::msg::MarkerArray marker_array;
    // Proportions based on human height
    double head_height = 0.2 * human_height_;
    double body_height = 0.3 * human_height_;
    double leg_height = 0.5 * human_height_;
    double arm_length = 0.4 * human_height_;
    double body_width = 0.3 * human_height_;
    double arm_width = 0.1 * human_height_;
    double leg_width = 0.1 * human_height_;

    // Create a 4x1 homogeneous point in the camera frame
    Eigen::Vector4f c_head_position(human_position_[0], human_position_[1], human_position_[2] + body_height + 0.5 * head_height, 1.0f);
    Eigen::Vector4f c_body_position(human_position_[0], human_position_[1], human_position_[2], 1.0f);
    Eigen::Vector4f c_right_leg_position(human_position_[0] - 0.1 * human_height_, human_position_[1], human_position_[2]- body_height * 0.5 - 0.5 * leg_height, 1.0f);
    Eigen::Vector4f c_left_leg_position(human_position_[0] + 0.1 * human_height_, human_position_[1], human_position_[2] - body_height * 0.5 - 0.5 * leg_height, 1.0f);
    Eigen::Vector4f c_right_arm_position(human_position_[0] - 0.2 * human_height_, human_position_[1], human_position_[2] + body_height * 0.5 - 0.5 * arm_length, 1.0f);
    Eigen::Vector4f c_left_arm_position(human_position_[0] + 0.2 * human_height_, human_position_[1], human_position_[2] + body_height * 0.5 - 0.5 * arm_length, 1.0f);

    Eigen::Vector4f world_head_position = c_head_position;
    Eigen::Vector4f world_body_position = c_body_position;
    Eigen::Vector4f world_right_leg_position = c_right_leg_position;
    Eigen::Vector4f world_left_leg_position = c_left_leg_position;
    Eigen::Vector4f world_right_arm_position = c_right_arm_position;
    Eigen::Vector4f world_left_arm_position = c_left_arm_position;


    // Head (sphere)
    auto head_marker = create_marker(
        visualization_msgs::msg::Marker::SPHERE,
        {world_head_position.x(), world_head_position.y(), world_head_position.z()},  // Head position based on total height
        {head_height, head_height, head_height},  // Head size proportional to height
        {1.0, 0.8, 0.6, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(head_marker);

    // Body (cylinder)
    auto body_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_body_position.x(), world_body_position.y(), world_body_position.z()},  // Body position
        {body_width, body_width, body_height},  // Body diameter and height
        {0.0, 0.0, 1.0, 1.0},  // Blue color for clothes
        "human", marker_id++);
    marker_array.markers.push_back(body_marker);

    // Right leg (cylinder)
    auto right_leg_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_right_leg_position.x(), world_right_leg_position.y(), world_right_leg_position.z()},  // Right leg position
        {leg_width, leg_width, leg_height},  // Leg size proportional to height
        {0.0, 0.0, 0.0, 1.0},  // Black for pants
        "human", marker_id++);
    marker_array.markers.push_back(right_leg_marker);

    // Left leg (cylinder)
    auto left_leg_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_left_leg_position.x(), world_left_leg_position.y(), world_left_leg_position.z()},  // Left leg position
        {leg_width, leg_width, leg_height},  // Leg size proportional to height
        {0.0, 0.0, 0.0, 1.0},  // Black for pants
        "human", marker_id++);
    marker_array.markers.push_back(left_leg_marker);

    // Right arm (cylinder)
    auto right_arm_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_right_arm_position.x(), world_right_arm_position.y(), world_right_arm_position.z()},  // Right arm position
        {arm_width, arm_width, arm_length},  // Arm length proportional to height
        {1.0, 0.8, 0.6, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(right_arm_marker);

    // Left arm (cylinder)
    auto left_arm_marker = create_marker(
        visualization_msgs::msg::Marker::CYLINDER,
        {world_left_arm_position.x(), world_left_arm_position.y(), world_left_arm_position.z()},  // Left arm position
        {arm_width, arm_width, arm_length},  // Arm length proportional to height
        {1.0, 0.8, 0.6, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(left_arm_marker);

    mark_pub_->publish(marker_array);
}

void RGBDNode::publish_human_marker_circle(double human_height_, std::array<double, 3> human_position_, int marker_id, Eigen::Matrix4f Tcw)
{
    visualization_msgs::msg::MarkerArray marker_array;
    // Proportions based on human height
    double sphere_height = 0.2 * human_height_;

    // Create a 4x1 homogeneous point in the camera frame
    Eigen::Vector4f c_human_position(human_position_[0], human_position_[1], human_position_[2], 1.0f);

    // Transform the point from the camera frame to the local map frame using Twc
    Eigen::Vector4f world_human_position = Tcw * c_human_position;

    // Head (sphere)
    auto head_marker = create_marker(
        visualization_msgs::msg::Marker::SPHERE,
        {world_human_position.x(), world_human_position.y(), world_human_position.z()},  // Head position based on total height
        {sphere_height, sphere_height, sphere_height},  // Head size proportional to height
        {1.0, 1.0, 0.0, 1.0},  // Skin tone
        "human", marker_id++);
    marker_array.markers.push_back(head_marker);

    mark_pub_->publish(marker_array);
}

void RGBDNode::publish_human(std::vector<Human> humans_, Eigen::Matrix4f Tcw)
{
    int marker_id = 0;
    for (const auto &human : humans_)
    {
        publish_human_marker_circle(human.height, human.position, marker_id, Tcw);
        marker_id += 1;
    }
}

void RGBDNode::segment_publish(cv::Mat rgbImg, cv::Mat depthImg, std::vector<tpt::objectsegment::OutputSeg> resSegment)
{
    std::vector<Human> humans;
    ORB_SLAM3::Frame& current_frame = pSLAM->getTracker()->mCurrentFrame;
    float fx = current_frame.fx;
    float fy = current_frame.fy;
    float cx = current_frame.cx;
    float cy = current_frame.cy;

    Sophus::SE3f Twc_SE3 = current_frame.GetPose();

    // Convert Sophus::SE3 to a 4x4 transformation matrix (Eigen::Matrix4f)
    Eigen::Matrix4f Twc = Twc_SE3.matrix();

    // Inverse to get the transformation from camera coordinates to world coordinates
    Eigen::Matrix4f Tcw = Twc.inverse();


    for (size_t i = 0; i < resSegment.size(); i++) {
        cv::Rect object_bbox_ = resSegment.at(i).box;

        int center_x = object_bbox_.x + object_bbox_.width / 2;
        int center_y = object_bbox_.y + object_bbox_.height / 2;
        int bottom_y = object_bbox_.y + object_bbox_.height;
        uint16_t depth = depthImg.at<uint16_t>(center_y, center_x);

        // Height of the bounding box in pixels
        float pixel_height = object_bbox_.height;
        float real_height = (pixel_height * depth) / fy;

        float z = depth / 1000.0f; // Convert mm to meters
        float x = (center_x - cx) * z / fx;
        float y = (bottom_y - cy) * z / fy;

        // Create a 4x1 homogeneous point in the camera frame
        Eigen::Vector4f point_cam(x, y, z, 1.0f);

        // Transform the point from the camera frame to the local map frame using Twc
        Eigen::Vector4f point_world = Tcw * point_cam;
        humans.emplace_back(Human{real_height / 1000.0f, {x, y, z}});
    }

    publish_human(humans, Tcw);
}


