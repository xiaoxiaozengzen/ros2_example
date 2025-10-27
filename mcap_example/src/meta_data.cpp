// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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

#include "e171_msgs/msg/e171.hpp"

using namespace std::chrono_literals;

class MetaData : public rclcpp::Node
{
public:
  explicit MetaData(std::string path) 
  : Node("data_generator"),
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

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MetaData>("/home/user/cgz_workspace/Bag/rosbag2_2024_01_28-15_56_33"));
  rclcpp::shutdown();
  return 0;
}
