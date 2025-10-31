#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include "rosbag2_cpp/typesupport_helpers.hpp"

#include <chrono>
#include <memory>
#include <fstream>
#include <thread>

#include "devastator_perception_msgs/msg/my_test.hpp"
#include "devastator_perception_msgs/msg/my_test.h"

using namespace std::chrono_literals;

static const std::string topic_name = "/my/test";

class McapFileReaderNode : public rclcpp::Node
{
public:
  explicit McapFileReaderNode(std::string path) 
  : Node("mcap_file_reader_node"),
  path_(path)
  {
    reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();

    thread_ = std::thread([this]() {
      std::this_thread::sleep_for(500ms);
      Work();
    });
  }

  ~McapFileReaderNode() {
    if(thread_.joinable()) {
      thread_.join();
    }
  }

  void Work() {
    try {
      // 改部分代码会自动搜索metadata.yaml文件，找不到会报错
      rosbag2_storage::MetadataIo metadata_io;
      auto ros2bag_meta_data = metadata_io.read_metadata(path_);
      std::cout << "Read metadata.yaml successfully!" << std::endl;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception : %s", e.what());
    }

    const rosbag2_cpp::StorageOptions storage_options({path_, "mcap"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
        rmw_get_serialization_format()});

    std::string input_serialization_format = rmw_get_serialization_format();
    std::string output_serialization_format = rmw_get_serialization_format();
    std::cerr << "input_serialization_format: " << input_serialization_format << std::endl;
    std::cerr << "output_serialization_format: " << output_serialization_format << std::endl;

    reader_->open(storage_options, converter_options);

#if 0
    // 白名单，只有在白名单中的topic才会被读取。如果不设置白名单，则所有的topic都会被读取
    rosbag2_storage::StorageFilter storage_filter;
    storage_filter.topics.push_back("/front_4d_radar_data");
    reader_->set_filter(storage_filter);
#endif

    // 需要保证存在metadata.yaml文件，否则会报错
    rosbag2_storage::BagMetadata bag_meta_data = reader_->get_metadata();
    std::cout << "version: " << bag_meta_data.version << std::endl;
    std::cout << "bag_size: " << bag_meta_data.bag_size << std::endl; //文件大小
    std::cout << "storage_identifier: " << bag_meta_data.storage_identifier << std::endl;
    for (const auto & entry : bag_meta_data.relative_file_paths) {
      std::cout << "relative_file_paths: " << entry << std::endl;
    }
    std::cout << "duration: " << std::chrono::duration_cast<std::chrono::seconds>(bag_meta_data.duration).count() << std::endl;
    std::cout << "starting_time(s): " << std::chrono::duration_cast<std::chrono::seconds>(bag_meta_data.starting_time.time_since_epoch()).count() << std::endl;
    std::cout << "starting_time(ns): " << std::chrono::duration_cast<std::chrono::nanoseconds>(bag_meta_data.starting_time.time_since_epoch()).count() << std::endl;
    std::cout << "message_count: " << bag_meta_data.message_count << std::endl;
    for (const rosbag2_storage::TopicInformation& entry : bag_meta_data.topics_with_message_count) {
      rosbag2_storage::TopicMetadata topic_metadata = entry.topic_metadata;
      std::size_t message_count = entry.message_count;
      std::cerr << "topic_metadata.name: " << topic_metadata.name
                << ", topic_metadata.type: " << topic_metadata.type
                << ", topic_metadata.serialization_format: " << topic_metadata.serialization_format
                // << ", topic_metadata.offered_qos_profiles: " << topic_metadata.offered_qos_profiles
                << ", message_count: " << message_count << std::endl;
    }

    auto current_file = reader_->get_current_file();
    std::cout << "current file: " << current_file << std::endl;

    auto current_uri = reader_->get_current_uri();
    std::cout << "current uri: " << current_uri << std::endl;

    auto topics = reader_->get_all_topics_and_types();
    for (const auto & entry : topics) {
      std::cout << "get topic name: " << entry.name << " type: " << entry.type << std::endl;
    }

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


    /**
     * https://github1s.com/ros2/rcutils/blob/foxy/include/rcutils/types/uint8_array.h#L29-L35
     * typedef struct RCUTILS_PUBLIC_TYPE rcutils_uint8_array_t
        {
          uint8_t * buffer;
          size_t buffer_length;
          size_t buffer_capacity;
          rcutils_allocator_t allocator;
        } rcutils_uint8_array_t;
     */
    while(reader_->has_next()) {
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> message = reader_->read_next();
      if (message->topic_name == topic_name) {
          std::cerr 
          << "topic name: " << message->topic_name
          << ", time_stamp sec: " << message->time_stamp / 1000000000
          << ", time_stamp nanosec: " << message->time_stamp % 1000000000
          << ", buffer_length: " << message->serialized_data->buffer_length
          << ", buffer_capacity: " << message->serialized_data->buffer_capacity << std::endl;

          cdr_deserializer_->deserialize(message, type_support, ros_message);

          std::cerr << "test_msg.a: " << static_cast<int>(test_msg.a)
                    << ", test_msg.b: " << static_cast<int>(test_msg.b)
                    << std::endl;
      } else {
        // std::cout << "------------- parse :" << message->topic_name << std::endl;
      }
    }    
  }

private:
    std::string path_{""};
    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
    std::thread thread_;
};

int main(int argc, char * argv[])
{
  if(argc < 2) {
    std::cerr << "Usage: ros2 run ros2bag_example reader <path>" << std::endl;
    return 1;
  }

  std::string path = argv[1];

  std::ifstream file(path);
  if (!file.good()) {
    std::cerr << "File not found: " << path << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<McapFileReaderNode>(path));
  rclcpp::shutdown();
  return 0;
}
