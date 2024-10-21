#include "bag_read_node.h"


BagFileReaderNode::BagFileReaderNode(bool segment): Node("bag_file_reader")
{
    this->segment_ = segment;
    // create publishser
    rgb_pub = this->create_publisher<sensor_msgs::msg::Image>("camera/rgb", 10);
    depth_pub = this->create_publisher<sensor_msgs::msg::Image>("camera/depth", 10);

    reader_ = std::make_shared<rosbag2_cpp::Reader>();
    reader_->open("record_bag");

    // Read and deserialize messages
    read_messages();
}

BagFileReaderNode::~BagFileReaderNode()
{
    running = false;
}


void BagFileReaderNode::read_messages()
{
    // Check if the bag file contains messages
    if (!reader_->has_next()) {
        RCLCPP_WARN(this->get_logger(), "No messages found in the bag file.");
        return;
    }

    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    sensor_msgs::msg::Image image_msg;

    while (reader_->has_next() && running) {
        // Read the next message in the bag
        auto serialized_msg = reader_->read_next();
        rclcpp::Time timestamp(serialized_msg->time_stamp);

        RCLCPP_INFO(this->get_logger(), "Reading message from topic: %s at time: %f", 
                        serialized_msg->topic_name.c_str(), timestamp.seconds());

        // Deserialize the message
        rclcpp::SerializedMessage serialized_data(*serialized_msg->serialized_data);
        serializer.deserialize_message(&serialized_data, &image_msg);

        if (serialized_msg->topic_name == "camera/rgb") {
            rgb_pub->publish(image_msg);
        }
        if (serialized_msg->topic_name == "camera/depth") {
            depth_pub->publish(image_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Image published");
        usleep(20000);
    }
}
