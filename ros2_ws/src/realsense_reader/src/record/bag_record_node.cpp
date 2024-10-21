#include "bag_record_node.h"

ImageBagRecordNode::ImageBagRecordNode(): Node("image_bag_recorder")
{   
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("record_bag");

    writer_->create_topic(
      {"camera/rgb",
       "sensor_msgs/msg/Image",
       rmw_get_serialization_format(),
       ""});


    writer_->create_topic(
      {"camera/depth",
       "sensor_msgs/msg/Image",
       rmw_get_serialization_format(),
       ""});

    if (segment_) {
        writer_->create_topic(
          {"camera/segment",
           "sensor_msgs/msg/Image",
           rmw_get_serialization_format(),
           ""});
    }

    if (segment_) {
        segmentModel_.init("/home/ducpa/ros2_ws/src/realsense_reader/src/segment/yolov8n-seg.engine", 1);
        segmentModel_.setInputImageFormat(tpt::IFormat::RGB);
    }

    // create sub
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/rgb");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/depth");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&ImageBagRecordNode::GrabRGBD, this);

}

ImageBagRecordNode::~ImageBagRecordNode()
{
    // rgb_writer_->reset();
    // depth_writer_->reset();
    RCLCPP_INFO(this->get_logger(), "Bag recording finished.");
}

void ImageBagRecordNode::GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD)
{
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mask;

    if (segment_) {
        cv::Mat im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void *)(cv_ptrRGB->image.ptr(0)), cv::Mat::AUTO_STEP);
        std::vector<std::vector<tpt::objectsegment::OutputSeg>> segmented;
        segmentModel_.infer(std::vector<cv::Mat>{im}, segmented);
        cv::Mat mask = segmentModel_.drawMaskPred(im, segmented.at(0));
    }

    // Serialize the image message and write it to the bag
    auto serialized_msg_rgb = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    auto serialized_msg_depth = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    // serialize
    rclcpp::Serialization<sensor_msgs::msg::Image> rgb_serializer;
    rclcpp::SerializedMessage rgb_serialized_data;

    // Serialize the image message
    rgb_serializer.serialize_message(msgRGB.get(), &rgb_serialized_data);
    rclcpp::Time timestamp(msgRGB->header.stamp);

    // Fill the serialized bag message with data
    serialized_msg_rgb->serialized_data = std::make_shared<rcutils_uint8_array_t>(rgb_serialized_data.get_rcl_serialized_message());
    serialized_msg_rgb->topic_name = "camera/rgb";
    serialized_msg_rgb->time_stamp = timestamp.nanoseconds();

    rclcpp::Serialization<sensor_msgs::msg::Image> depth_serializer;
    rclcpp::SerializedMessage depth_serialized_data;

    // Serialize the image message
    depth_serializer.serialize_message(msgD.get(), &depth_serialized_data);

    // Fill the serialized bag message with data
    serialized_msg_depth->serialized_data = std::make_shared<rcutils_uint8_array_t>(depth_serialized_data.get_rcl_serialized_message());
    serialized_msg_depth->topic_name = "camera/depth";
    serialized_msg_depth->time_stamp = timestamp.nanoseconds();

    // rgb_writer_->write(serialized_msg_rgb);
    // depth_writer_->write(serialized_msg_depth);

    writer_->write(serialized_msg_rgb);
    writer_->write(serialized_msg_depth);

    if (segment_) {
        sensor_msgs::msg::Image::SharedPtr msg_mask = cv_bridge::CvImage(std_msgs::msg::Header(), "8UC1", mask).toImageMsg();

        //serialize
        auto serialized_msg_mask = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        rclcpp::Serialization<sensor_msgs::msg::Image> mask_serializer;
        rclcpp::SerializedMessage mask_serialized_data;

        // Serialize the image message
        mask_serializer.serialize_message(msg_mask.get(), &mask_serialized_data);

        // Fill the serialized bag message with data
        serialized_msg_mask->serialized_data = std::make_shared<rcutils_uint8_array_t>(mask_serialized_data.get_rcl_serialized_message());
        serialized_msg_mask->topic_name = "camera/segment";
        serialized_msg_mask->time_stamp = timestamp.nanoseconds();
        writer_->write(serialized_msg_mask);
//        RCLCPP_INFO(this->get_logger(), "Write mask.");
    }
}

void ImageBagRecordNode::serialize_message(const sensor_msgs::msg::Image::SharedPtr &msg,
                                      std::shared_ptr<rosbag2_storage::SerializedBagMessage> &serialized_msg,
                                      std::string topic_name, builtin_interfaces::msg::Time time_stamp)
{
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    rclcpp::SerializedMessage serialized_data;

    // Serialize the image message
    serializer.serialize_message(msg.get(), &serialized_data);

    // Fill the serialized bag message with data
    serialized_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>(serialized_data.get_rcl_serialized_message());
    serialized_msg->topic_name = topic_name;
    serialized_msg->time_stamp = time_stamp.nanosec;
}
