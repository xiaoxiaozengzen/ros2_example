#include "devastator_perception_msgs/srv/ser_cli.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <thread>

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
class ServerNode : public rclcpp::Node {
public:
  ServerNode()
      : Node("add_two_ints_server") {
    this->declare_parameter("para1", 10);
    this->declare_parameter("para2", "service_para2");
    this->declare_parameter("para3", 10.0);

    RCLCPP_INFO(get_logger(), "Ready to add two ints.");
    service_ = this->create_service<devastator_perception_msgs::srv::SerCli>("add_two_ints", std::bind(&ServerNode::add, this, std::placeholders::_1, std::placeholders::_2));
  }

  void add(const std::shared_ptr<devastator_perception_msgs::srv::SerCli::Request>
              request,
          std::shared_ptr<devastator_perception_msgs::srv::SerCli::Response>
              response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Incoming request\na: %ld"
                " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]",
                (long int)response->sum);
  }

private:
  rclcpp::Service<devastator_perception_msgs::srv::SerCli>::SharedPtr service_{nullptr};
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ServerNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
}