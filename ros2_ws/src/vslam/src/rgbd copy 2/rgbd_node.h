#ifndef RGBD_NODE_H
#define RGBD_NODE_H

#include <torch/torch.h>

#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <thread>
#include <filesystem>
#include <memory>

#include <opencv2/core/core.hpp>

#include "ORB-SLAM3/include/System.h"
#include "include/gaussian_mapper.h"
#include "viewer/imgui_viewer.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
//#include "opencv4/opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>

#include "utility.h"
#include "yolosegment.h"

#define GET_MOMENT	    		std::chrono::high_resolution_clock::now()
#define GET_DURATION(stop,start)	std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count()

using namespace std::chrono_literals;

struct Human
    {
        double height;
        std::array<double, 3> position;
    };


class RGBDNode: public rclcpp::Node
{
public:
    RGBDNode(string vocalbularyPath, string settingPath, string gaussianSettingPath, string outputDir);
    RGBDNode();
    ~RGBDNode();

private:
    using ImuMsg = sensor_msgs::msg::Imu;
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync3_policy;

    void initializedVSLAM();

    void GrabImu(const ImuMsg::SharedPtr msg);
    // void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);
    void GrabRGBDS(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    void publishCurrentFramePointCloud(builtin_interfaces::msg::Time stamp);
    void publishPointClould(builtin_interfaces::msg::Time stamp);
    void publishTrajectory(builtin_interfaces::msg::Time stamp);
    void publishPose(Sophus::SE3f camera_pose_SE3, builtin_interfaces::msg::Time stamp);
    void publishMarker(cv::Mat segImg, cv::Mat depthImg, builtin_interfaces::msg::Time stamp);
    void publishCurrentDensePointCloud(cv::Mat depthImg, builtin_interfaces::msg::Time stamp);

    void rvizViewCallback();
    void rvizResponseHandler(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future);

    void getPointCloud(std::vector<std::vector<float>> &pcloud);
    std::vector<Sophus::SE3f> getCurrentTrajectory();
    std::vector<Eigen::Vector3f> generateCurrentDensePointCloud(cv::Mat depthImg);

    visualization_msgs::msg::Marker create_marker(int marker_type,
                       std::array<double, 3> position,
                       std::array<double, 3> scale,
                       std::array<double, 4> color,
                       std::string ns,
                       int id);
    void publish_human_marker(double human_height_, std::array<double, 3> human_position_, int marker_id, Eigen::Matrix4f Tcw);
    void publish_human_marker_aligned_X(double human_height_, std::array<double, 3> human_position_, int marker_id, Eigen::Matrix4f Tcw);
    void publish_human_marker_aligned_Y(double human_height_, std::array<double, 3> human_position_, int marker_id, Eigen::Matrix4f Tcw);
    void publish_human_marker_circle(double human_height_, std::array<double, 3> human_position_, int marker_id, Eigen::Matrix4f Tcw);
    void publish_human(std::vector<Human> humans_, Eigen::Matrix4f Tcw);
    void segment_publish(cv::Mat rgbImg, cv::Mat depthImg, std::vector<tpt::objectsegment::OutputSeg> resSegment);

    std::shared_ptr<ORB_SLAM3::System> pSLAM;
    std::shared_ptr<GaussianMapper> pGausMapper;
    std::shared_ptr<ImGuiViewer> pViewer;

    // thread for gaussianMapper
    std::thread training_thd;

    // thread for Viewer
    std::thread viewer_thd;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;
    cv_bridge::CvImageConstPtr cv_ptrS;

    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr view_rviz_client;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dense_pcloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lmp_ref_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mark_pub_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > seg_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
    // std::shared_ptr<message_filters::Synchronizer<approximate_sync3_policy> > syncApproximate;

    rclcpp::TimerBase::SharedPtr view_rviz_timer_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    float imageScale = 1.f;
    torch::DeviceType device_type;
    bool use_viewer = true;

    tpt::objectsegment::YoloSegment segmentModel_;

    std::filesystem::path output_dir;

    std::string last_camera_info_;

    // config
    int width_img = 1280;
    int height_img = 720;
    int fps = 30;
};

#endif // RGBD_NODE_H
