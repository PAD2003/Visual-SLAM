#include "rclcpp/rclcpp.hpp"
#include "stream_node.h"


int main(int argc, char** argv)
{
    bool segment = false;
    bool record = false;
    if (argc >= 2) {
        int t = std::atoi(argv[1]);
        segment = t > 0;
    }
    if (argc >= 3) {
        int t = std::atoi(argv[2]);
        record = t > 0;
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<RealsenseNode>(segment, record);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
