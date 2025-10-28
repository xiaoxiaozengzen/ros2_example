#include <chrono>
#include <iostream>
#include <vector>
#include <string>

#include "devastator_perception_msgs/msg/my_test.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_storage/storage_filter.hpp"

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "rclcpp/rclcpp.hpp"

class Db3FileWriteNode : public rclcpp::Node {
 public:
    Db3FileWriteNode(std::string path):
     Node("db3_file_writer_node"),
     path_(path) {
        writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

        // 存储包的文件夹地址，注意不是具体的文件名
        write_storage_options.uri = path_;
        write_storage_options.storage_id = "sqlite3";

        converter_options.input_serialization_format = "cdr";  // bag 包序列化类型
        converter_options.output_serialization_format = "cdr";

        WriterOpen();
    }
    ~Db3FileWriteNode() {}

    void ReaderOpen();

    void WriterOpen() {
        writer_->open(write_storage_options, converter_options);
        writer_->create_topic(
        {"/my/test",
        "devastator_perception_msgs/msg/MyTest",
        "cdr",
        ""});

        subscription_ = create_subscription<devastator_perception_msgs::msg::MyTest>(
        "/my/test", 10, std::bind(&Db3FileWriteNode::topic_callback, this, std::placeholders::_1));
    }

    void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const {
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

        bag_message->topic_name = "/my/test";
        if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
            RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
                rcutils_get_error_string().str);
        }

        writer_->write(bag_message);
    }

 private:
    std::string path_;

    rosbag2_cpp::ConverterOptions converter_options{};

    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
    rosbag2_cpp::StorageOptions write_storage_options{};  // bag 包地址

};

int main(int argc, char** argv) {
    std::string path = argv[1];

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Db3FileWriteNode>(path));
    rclcpp::shutdown();
    return 0;
}