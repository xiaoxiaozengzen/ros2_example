#include <chrono>
#include <iostream>
#include <vector>
#include <string>

#include "devastator_perception_msgs/msg/my_test.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_storage/storage_filter.hpp"

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "rclcpp/rclcpp.hpp"

class Db3FileParseNode : public rclcpp::Node {
 public:
    Db3FileParseNode(std::string path):
     Node("db3_file_parse_node"),
     path_(path) {
        storage_options.uri = path_;
        storage_options.storage_id = "sqlite3";

        converter_options.input_serialization_format = "cdr";  // bag 包序列化类型
        converter_options.output_serialization_format = "cdr";

        filter_name = "/my/test";
        storage_filter.topics.push_back(filter_name);

        ReaderOpen();
    }

    ~Db3FileParseNode() {}

    void ReaderOpen() {
        reader.open(storage_options, converter_options);

        std::cout << "file = " << reader.get_current_file() << std::endl;
        std::cout << "uri = " << reader.get_current_uri() << std::endl;

        auto topics = reader.get_all_topics_and_types();  // 获取bag包的topic以及序列化类型
        for (auto t : topics) {
            std::cout << "meta name: " << t.name << std::endl;
            std::cout << "meta type: " << t.type << std::endl;
            std::cout << "meta serialization_format: " << t.serialization_format << std::endl;
        }

        reader.set_filter(storage_filter);

        devastator_perception_msgs::msg::MyTest test_msg;
        auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
        ros_message->time_stamp = 0;
        ros_message->message = nullptr;
        ros_message->allocator = rcutils_get_default_allocator();
        ros_message->message = &test_msg;

        auto library = rosbag2_cpp::get_typesupport_library(
        "devastator_perception_msgs/msg/MyTest", "rosidl_typesupport_cpp");
        
        auto type_support = rosbag2_cpp::get_typesupport_handle(
        "devastator_perception_msgs/msg/MyTest", "rosidl_typesupport_cpp", library);

        rosbag2_cpp::SerializationFormatConverterFactory factory;
        std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;
        cdr_deserializer_ = factory.load_deserializer("cdr");

        int count = 0;

        while (reader.has_next()) {
            // serialized data
            auto serialized_message = reader.read_next();
            std::cout << "serialized_message topic_name : " << serialized_message->topic_name << std::endl;
            std::cout << "serialized_message data length : " << serialized_message->serialized_data->buffer_length << std::endl;
            for(size_t i=0; i<serialized_message->serialized_data->buffer_length; i++) {
                std::cout << std::hex << (std::uint32_t)(*(serialized_message->serialized_data->buffer + i)) << " ";
            }
            std::cout << std::dec << std::endl;

            // deserialization and conversion to ros message
            cdr_deserializer_->deserialize(serialized_message, type_support, ros_message);

            // use ros message
            std::cout << "test_msg.a = " << static_cast<std::uint32_t>(test_msg.a) << std::endl;
            std::cout << "test_msg.b = " << static_cast<std::uint32_t>(test_msg.b) << std::endl;
            std::cout << "test_msg.c = " << std::boolalpha << test_msg.c << std::endl;

            count ++;
        }

        std::cout << "count = " << count << std::endl;
    }
 private:
    std::string path_;

    rosbag2_cpp::readers::SequentialReader reader;
    rosbag2_cpp::StorageOptions storage_options{};  // bag 包地址

    rosbag2_cpp::ConverterOptions converter_options{};

    std::string filter_name;
    rosbag2_storage::StorageFilter storage_filter;
};


int main(int argc, char** argv) {
    std::string path = argv[1];

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Db3FileParseNode>(path));
    rclcpp::shutdown();
    return 0;
}
