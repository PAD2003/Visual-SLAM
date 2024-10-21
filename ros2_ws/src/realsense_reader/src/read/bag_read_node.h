#ifndef BAG_FILE_READER_NODE_H
#define BAG_FILE_READER_NODE_H

#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rosbag2_cpp/reader.hpp>

//#include "yolosegment.h"


using namespace std::chrono_literals;

class BagFileReaderNode: public rclcpp::Node
{
public:
    BagFileReaderNode(bool segment=false);
    ~BagFileReaderNode();

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;

    rclcpp::TimerBase::SharedPtr timer_rgb;
    rclcpp::TimerBase::SharedPtr timer_depth;
    rclcpp::TimerBase::SharedPtr timer_camera;
    bool running = true;
    bool segment_ = false;
//    tpt::objectsegment::YoloSegment segmentModel_;

    std::shared_ptr<rosbag2_cpp::Reader> reader_;

    void read_messages();
};

#endif // RGBD_NODE_H
