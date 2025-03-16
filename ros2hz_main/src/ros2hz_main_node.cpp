#include <chrono>
#include <deque>
#include <iostream>
#include <memory>
#include <thread>
#include <unordered_map>

#include <toml.hpp>

#include "ros2hz/generic_subscription.hpp"
#include "ros2hz/ros2hz.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char *argv[]) {
  const auto& dir = ament_index_cpp::get_package_share_directory("ros2hz_main");
  std::cout << dir + "/config/config.toml" <<std::endl;

  auto data = toml::parse(dir + "/config/config.toml");
  std::vector<std::string> topics =
      toml::find<std::vector<std::string>>(data, "topics");
  std::cout << "topics: " << topics.size() << std::endl;
  for(const auto& topic : topics) {
    std::cout << "topic: " << topic << std::endl;
  }

  rclcpp::init(argc, argv);
  //rclcpp::executors::SingleThreadedExecutor executor;
  //executor.add_node(std::make_shared<devastator::topichz::TopicHz>(topics));
  //executor.spin();
  rclcpp::spin(std::make_shared<devastator::topichz::TopicHz>(topics));
  rclcpp::shutdown();
  return 0;
}