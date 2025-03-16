#pragma once

#include <chrono>
#include <iostream>
#include <vector>
#include <string>

#include "devastator_perception_msgs/msg/radar_object_array.hpp"
#include "devastator_perception_msgs/msg/my_test.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_storage/storage_filter.hpp"

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "rclcpp/rclcpp.hpp"

class BagParse : public rclcpp::Node {
 public:
    BagParse(std::string path);
    ~BagParse();

    void ReaderOpen();
 private:
    std::string path_;

    rosbag2_cpp::readers::SequentialReader reader;
    rosbag2_cpp::StorageOptions storage_options{};  // bag 包地址

    rosbag2_cpp::ConverterOptions converter_options{};

    std::string filter_name;
    rosbag2_storage::StorageFilter storage_filter;
};