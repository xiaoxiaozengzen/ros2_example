#include "devastator_perception_msgs/srv/ser_cli.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <random>
#include <thread>

using namespace std::chrono_literals;

/**
 * 对于普通的node节点，其默认会提供几个服务
 * 1. /<node_name>/get_parameter_types
 * 2. /<node_name>/get_parameters
 * 3. /<node_name>/list_parameters
 * 4. /<node_name>/set_parameters
 * 5. /<node_name>/describe_parameters
 * 6. /<node_name>/set_parameters_atomically
 * 
 */
class ClientNode : public rclcpp::Node {
public:
  ClientNode() : Node("add_two_ints_client") {
    client_ = this->create_client<devastator_perception_msgs::srv::SerCli>(
        "add_two_ints");

    send_request_thread_ = std::thread(&ClientNode::send_request, this);

    get_parameters_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(
        "/add_two_ints_server/get_parameters");
    get_parameters_thread_ = std::thread(&ClientNode::get_parameters_request, this);
  }

  ~ClientNode() {
    if(send_request_thread_.joinable()) {
      send_request_thread_.join();
    }
  }

  void send_request() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service is available.");

    while(rclcpp::ok()) {
      static int count = 0;
      count++;

      auto request = std::make_shared<devastator_perception_msgs::srv::SerCli::Request>();
      request->a = std::rand() % 100;
      request->b = std::rand() % 100;

      std::shared_future<devastator_perception_msgs::srv::SerCli::Response::SharedPtr> result = client_->async_send_request(request);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending request num %ld: Request a %ld, b %ld", count, request->a, request->b);

      // // Wait for the result.
      // if (rclcpp::spin_until_future_complete(node, result) ==
      //     rclcpp::FutureReturnCode::SUCCESS) {
      //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
      // } else {
      //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      //               "Failed to call service add_two_ints");
      // }

      std::future_status status = result.wait_for(5s);
      if(status == std::future_status::ready) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
      } else if (status == std::future_status::timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "service call timed out");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
      }

      std::this_thread::sleep_for(2s);
    }
  }

  void get_parameters_request() {
    while(rclcpp::ok()) {
      auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      request->names.push_back("para1");
      request->names.push_back("para2");
      request->names.push_back("para3");

      auto result = get_parameters_client_->async_send_request(request);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending get_parameters request");

      std::future_status status = result.wait_for(5s);
      if(status == std::future_status::ready) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received get_parameters response");
        for(auto &param : result.get()->values) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameter type: %d", param.type);
          if(param.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameter value: %d", param.integer_value);
          } else if(param.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameter value: %s", param.string_value.c_str());
          } else if(param.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameter value: %f", param.double_value);
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown parameter type");
          }
        }
      } else if (status == std::future_status::timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "get_parameters call timed out");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call get_parameters");
      }

      std::this_thread::sleep_for(5s);
    }    
  }
private:
  rclcpp::Client<devastator_perception_msgs::srv::SerCli>::SharedPtr client_;
  std::thread send_request_thread_;

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
  std::thread get_parameters_thread_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ClientNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}