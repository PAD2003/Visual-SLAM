#include "realsense_node.h"

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}

static rs2_option get_sensor_option(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

RealsenseNode::RealsenseNode(): Node("realsense415")
{
    // create realsense
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        throw std::runtime_error("No device connected, please connect a RealSense device.");
//        return 0;
    }
    else
        selected_device = devices[0];

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
        if (sensor.supports(RS2_CAMERA_INFO_NAME))
        {
            ++index;
            get_sensor_option(sensor);
            if (index == 1) // irSensor
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for depth information
            }

            if (index == 2) // RGB camera
            {
                // sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
                sensor.set_option(RS2_OPTION_EXPOSURE, 150);
                sensor.set_option(RS2_OPTION_GAIN, 64);
            }

            if (index == 3)
                sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
        }

    // RGB stream
    cfg.enable_stream(RS2_STREAM_COLOR, width_img, height_img, RS2_FORMAT_RGB8, fps);

    // Depth stream
    cfg.enable_stream(RS2_STREAM_DEPTH, width_img, height_img, RS2_FORMAT_Z16, fps);

    rs2::pipeline_profile pipe_profile = pipe.start(cfg);
    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;

    align_to = find_stream_to_align(pipe_profile.get_streams());
    align = std::make_shared<rs2::align>(align_to);

    // create publishser
//    rgb_pub = this->create_publisher<sensor_msgs::msg::Image>("camera/rgb", 10);
//    depth_pub = this->create_publisher<sensor_msgs::msg::Image>("camera/depth", 10);

//    rgb_writer_ = std::make_unique<rosbag2_cpp::Writer>();
//    depth_writer_ = std::make_unique<rosbag2_cpp::Writer>();
//    rgb_writer_->open("rgb_bag");
//    depth_writer_->open("depth_bag");

//    rclcpp::Time timestamp(msgRGB->header.stamp);

    timer_camera = this->create_wall_timer(std::chrono_literals::operator""ms(10), std::bind(&RealsenseNode::TimerCallback, this));

}

RealsenseNode::~RealsenseNode()
{

}

void RealsenseNode::TimerCallback()
{
    // Block program until frames arrive
    rs2::frameset frames = pipe.wait_for_frames();
//    double timestamp_image = frames.get_timestamp() * 1e-3; // second

    // Perform alignment here
    auto processed = align->process(frames);

    // Trying to get both other and aligned depth frames
    rs2::video_frame color_frame = processed.first(align_to);
    rs2::depth_frame depth_frame = processed.get_depth_frame();

    cv::Mat rgb_img = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void *)(color_frame.get_data()), cv::Mat::AUTO_STEP);
    cv::Mat depth_img = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void *)(depth_frame.get_data()), cv::Mat::AUTO_STEP);

    auto now = this->get_clock()->now();

    sensor_msgs::msg::Image::SharedPtr msg_rgb = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_img).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr msg_depth = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_img).toImageMsg();

    msg_rgb->header.stamp = now;
    msg_depth->header.stamp = now;

    rgb_pub->publish(*msg_rgb.get());
    depth_pub->publish(*msg_depth.get());
    RCLCPP_INFO(this->get_logger(), "Image published");

    // Serialize the image message and write it to the bag
//    auto serialized_msg_rgb = std::make_shared<rosbag2_storage::SerializedBagMessage>();
//    auto serialized_msg_depth = std::make_shared<rosbag2_storage::SerializedBagMessage>();
//    serialize_message(msg_rgb, serialized_msg_rgb, "camera/rgb", now);
//    serialize_message(msg_depth, serialized_msg_depth, "camera/depth", now);
//    rgb_writer_->write(serialized_msg_rgb);
//    depth_writer_->write(serialized_msg_depth);
}

void RealsenseNode::serialize_message(const sensor_msgs::msg::Image::SharedPtr &msg,
                                      std::shared_ptr<rosbag2_storage::SerializedBagMessage> &serialized_msg,
                                      std::string topic_name, rclcpp::Time time_stamp)
{
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    rclcpp::SerializedMessage serialized_data;

    // Serialize the image message
    serializer.serialize_message(msg.get(), &serialized_data);

    // Fill the serialized bag message with data
    serialized_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>(serialized_data.get_rcl_serialized_message());
    serialized_msg->topic_name = topic_name;
    serialized_msg->time_stamp = time_stamp.nanoseconds();
}
