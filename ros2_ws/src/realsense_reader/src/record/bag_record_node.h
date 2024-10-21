#ifndef REALSENSE_RECORD_NODE_H
#define REALSENSE_RECORD_NODE_H

#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <cv_bridge/cv_bridge.h>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "yolosegment.h"

using namespace std::chrono_literals;

class ImageBagRecordNode: public rclcpp::Node
{
public:
    ImageBagRecordNode();
    ~ImageBagRecordNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::unique_ptr<rosbag2_cpp::Writer> rgb_writer_;
    std::unique_ptr<rosbag2_cpp::Writer> depth_writer_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;

    bool segment_ = true;
    tpt::objectsegment::YoloSegment segmentModel_;

    // config
    int width_img = 1280;
    int height_img = 720;
    int fps = 30;

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);
    void serialize_message(const sensor_msgs::msg::Image::SharedPtr &msg,
                           std::shared_ptr<rosbag2_storage::SerializedBagMessage> &serialized_msg,
                           std::string topic_name, builtin_interfaces::msg::Time time_stamp);
};

#endif // REALSENSE_RECORD_NODE_H
