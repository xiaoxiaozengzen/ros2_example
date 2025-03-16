#include "ros2bagrecord/BagRecord.hpp"

BagRecord::BagRecord(std::string path)
    :Node("my_test"),
    path_(path) {
        writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

        write_storage_options.uri = path_;
        write_storage_options.storage_id = "sqlite3";

        converter_options.input_serialization_format = "cdr";  // bag 包序列化类型
        converter_options.output_serialization_format = "cdr";

        WriterOpen();
    }

BagRecord::~BagRecord() {}

void BagRecord::WriterOpen() {
    writer_->open(write_storage_options, converter_options);
    writer_->create_topic(
      {"/sensor/radar0_object",
       "devastator_perception_msgs/msg/RadarObjectArray",
       "cdr",
       ""});
    
    subscription_ = create_subscription<devastator_perception_msgs::msg::RadarObjectArray>(
      "/sensor/radar0_object", 10, std::bind(&BagRecord::topic_callback, this, std::placeholders::_1));
}

void BagRecord::topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const {
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        new rcutils_uint8_array_t,
        [this](rcutils_uint8_array_t *msg){
            auto fini_return = rcutils_uint8_array_fini(msg);
            delete msg;
            if (fini_return != RCUTILS_RET_OK) {
                RCLCPP_ERROR(this->get_logger(),
                    "Failed to destroy serialized message %s", rcutils_get_error_string().str);
            }
        }
    );

    *bag_message->serialized_data = msg->release_rcl_serialized_message();

    std::cout << "record message length: " << static_cast<std::uint32_t>(bag_message->serialized_data->buffer_length) << std::endl;

    bag_message->topic_name = "/sensor/radar0_object";
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
        rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
}