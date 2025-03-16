#pragma once

#include <chrono>
#include <iostream>
#include <vector>
#include <string>

#include "devastator_perception_msgs/msg/radar_object_array.hpp"
#include "devastator_perception_msgs/msg/my_test.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_storage/storage_filter.hpp"

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "rclcpp/rclcpp.hpp"

class BagRecord : public rclcpp::Node {
 public:
    BagRecord(std::string path);
    ~BagRecord();

    void ReaderOpen();

    void WriterOpen();
    void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const;

 private:
    std::string path_;

    rosbag2_cpp::ConverterOptions converter_options{};

    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
    rosbag2_cpp::StorageOptions write_storage_options{};  // bag 包地址

};