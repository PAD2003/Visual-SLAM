#ifndef REALSENSE_STREAM_NODE_H
#define REALSENSE_STREAM_NODE_H

#include <chrono>
#include <iostream>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "yolosegment.h"

using namespace std::chrono_literals;

class RealsenseNode: public rclcpp::Node
{
public:
    RealsenseNode(bool segment=false, bool record=false);
    ~RealsenseNode();
    void setRecord(bool value);
    void initBag();

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr seg_pub;

    rclcpp::TimerBase::SharedPtr timer_rgb;
    rclcpp::TimerBase::SharedPtr timer_depth;
    rclcpp::TimerBase::SharedPtr timer_camera;

    std::unique_ptr<rosbag2_cpp::Writer> writer_;

    bool record_ = false;
    bool segment_ = false;
    tpt::objectsegment::YoloSegment segmentModel_;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Align depth and RGB frames
    rs2_stream align_to = RS2_STREAM_ANY;
    std::shared_ptr<rs2::align> align;

    // config
    int width_img = 1280;
    int height_img = 720;
    int fps = 30;

    void TimerCallback();
};

#endif // REALSENSE_STREAM_NODE_H
