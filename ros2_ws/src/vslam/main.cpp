#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rgbd_node.h"

//#include "ORB-SLAM3/include/System.h"
//#include "include/gaussian_mapper.h"
//#include "viewer/imgui_viewer.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RGBDNode>();
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);

    // Create a single-threaded executor
//    rclcpp::executors::SingleThreadedExecutor executor;
//    executor.add_node(node);

    // Spin the executor to handle callbacks
//    executor.spin();

    rclcpp::shutdown();

    return 0;
}
