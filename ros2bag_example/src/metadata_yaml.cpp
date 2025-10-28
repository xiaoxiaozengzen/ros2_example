#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <example_interfaces/msg/int32.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/metadata_io.hpp>

#include <chrono>
#include <memory>
#include <fstream>

#include "devastator_perception_msgs/msg/my_test.hpp"

using namespace std::chrono_literals;

class MetaData : public rclcpp::Node
{
public:
  explicit MetaData(std::string path) 
  : Node("metadata_yaml"),
  path_(path)
  {
    Work();
  }

  void Work() {
    rosbag2_storage::MetadataIo metadata_io;
    auto bag_meta_data = metadata_io.read_metadata(path_);

    std::cout << "version: " << bag_meta_data.version << std::endl;
    std::cout << "bag_size: " << bag_meta_data.bag_size << std::endl;
    std::cout << "storage_identifier: " << bag_meta_data.storage_identifier << std::endl;
    for (const auto & entry : bag_meta_data.relative_file_paths) {
      std::cout << "relative_file_paths: " << entry << std::endl;
    }
    std::cout << "duration: " << std::chrono::duration_cast<std::chrono::seconds>(bag_meta_data.duration).count() << std::endl;
    std::cout << "starting_time: " << std::chrono::duration_cast<std::chrono::seconds>(bag_meta_data.starting_time.time_since_epoch()).count() << std::endl;
    std::cout << "message_count: " << bag_meta_data.message_count << std::endl;
    for (const auto& entry : bag_meta_data.topics_with_message_count) {
      std::cout << "topic name: " << entry.topic_metadata.name << " message_count: " << entry.message_count << std::endl;
    }    
  }

private:
    std::string path_{""};
    
};

int main(int argc, char * argv[])
{
  if(argc < 2) {
      std::cerr << "Usage: metadata_yaml <bag_path>" << std::endl;
      return -1;
  }
  std::string bag_path = argv[1];

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MetaData>(bag_path));
  rclcpp::shutdown();
  return 0;
}
