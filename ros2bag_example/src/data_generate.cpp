#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <example_interfaces/msg/int32.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <chrono>
#include <memory>

#include "devastator_perception_msgs/msg/my_test.hpp"

using namespace std::chrono_literals;

class DataGenerator : public rclcpp::Node
{
public:
  DataGenerator()
  : Node("data_generator")
  {
    const rosbag2_cpp::StorageOptions storage_options({"timed_synthetic_bag", "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
        rmw_get_serialization_format()});
    
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"/my/test",
        "devastator_perception_msgs/msg/MyTest",
        rmw_get_serialization_format(),
        ""});

    timer_ = create_wall_timer(1s, std::bind(&DataGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    data_.a = 42;
    data_.b = 10;
    data_.c = true;

    auto serializer = rclcpp::Serialization<devastator_perception_msgs::msg::MyTest>();
    auto serialized_message = rclcpp::SerializedMessage();
    serializer.serialize_message(&data_, &serialized_message);

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [this](rcutils_uint8_array_t * msg) {
        auto fini_return = rcutils_uint8_array_fini(msg);
        delete msg;
        if (fini_return != RCUTILS_RET_OK) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to destroy serialized message %s",
            rcutils_get_error_string().str);
        }
      });
    *bag_message->serialized_data = serialized_message.release_rcl_serialized_message();

    bag_message->topic_name = "/my/test";
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "Error getting current time: %s",
        rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
  devastator_perception_msgs::msg::MyTest data_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGenerator>());
  rclcpp::shutdown();
  return 0;
}
