#include "rclcpp/rclcpp.hpp"
#include "bag_record_node.h"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImageBagRecordNode>();
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
