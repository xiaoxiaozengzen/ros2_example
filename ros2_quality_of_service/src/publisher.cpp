#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "devastator_perception_msgs/msg/my_test.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(std::string topic_name, rclcpp::NodeOptions options, rclcpp::QoS qos)
    : Node(topic_name, options), options_(options), qos_(qos)
    {
      get_node_options(options_);
      get_qos(qos_);
      qos_.keep_last(2);

      publisher_ = this->create_publisher<devastator_perception_msgs::msg::MyTest>("topic", qos_);
      timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void get_node_options(rclcpp::NodeOptions & options) {
      std::cout << "================== get_node_options ==================" << std::endl;
      const rcl_node_options_t * node_options = options.get_rcl_node_options();
      std::cout << std::hex << "rcl_node_options_t domain_id: " << node_options->domain_id << std::endl;
      std::cout << std::boolalpha << "rcl_node_options_t use_global_arguments: " << node_options->use_global_arguments << std::endl;
      std::cout << std::boolalpha << "rcl_node_options_t enable_rosout: " << node_options->enable_rosout << std::endl;

      const std::vector<std::string> arguments = options.arguments();
      std::cout << "The amount of arguments: " << arguments.size() << std::endl;
      for(auto& arg : arguments)
      {
        std::cout << "Argument: " << arg << std::endl;
      }

      std::cout << std::boolalpha << "use_intra_process_comms: " << options.use_intra_process_comms() << std::endl;
    }

    void get_qos(rclcpp::QoS& qos) {
      std::cout << "================== get_qos ==================" << std::endl;
      rmw_qos_profile_t profile = qos.get_rmw_qos_profile();

      rmw_qos_history_policy_t history = profile.history;
      if(history == rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
      {
        std::cout << "History: RMW_QOS_POLICY_HISTORY_KEEP_LAST" << std::endl;
      }
      else if(history == rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL)
      {
        std::cout << "History: RMW_QOS_POLICY_HISTORY_KEEP_ALL" << std::endl;
      }
      else if(history == rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT)
      {
        std::cout << "History: RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT" << std::endl;
      }
      else
      {
        std::cout << "History: Unknown" << std::endl;
      }
      std::cout << std::dec << "Depth: " << profile.depth << std::endl;

      rmw_qos_reliability_policy_t reliability = profile.reliability;
      if(reliability == rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
      {
        std::cout << "Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT" << std::endl;
      }
      else if(reliability == rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      {
        std::cout << "Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE" << std::endl;
      }
      else if(reliability == rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT)
      {
        std::cout << "Reliability: RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT" << std::endl;
      }
      else
      {
        std::cout << "Reliability: Unknown" << std::endl;
      }

      rmw_time_t deadline = profile.deadline;
      std::cout << "Deadline: " << deadline.sec << "s " << deadline.nsec << "ns" << std::endl;
    }

    void timer_callback()
    {
      auto message = devastator_perception_msgs::msg::MyTest();
      message.a = 1;
      message.b = 2;
      message.c = true;
      RCLCPP_INFO(this->get_logger(), "Publishing: ");
      publisher_->publish(message);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<devastator_perception_msgs::msg::MyTest>::SharedPtr publisher_;
    rclcpp::NodeOptions options_;
    rclcpp::QoS qos_;
};

void set_node_options(rclcpp::NodeOptions & options) {

}

void set_qos(rclcpp::QoS & qos) {

}

int main(int argc, char * argv[])
{
  std::string node_name = "minimal_publisher";
  rclcpp::NodeOptions options = rclcpp::NodeOptions();

  rclcpp::QoSInitialization qos_init = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default);
  rclcpp::QoSInitialization qos_init2 = rclcpp::QoSInitialization(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10);
  rclcpp::QoS qos(10);
  rclcpp::QoS qos2(qos_init);
  rclcpp::QoS qos3(qos_init2);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(node_name, options, qos2));
  rclcpp::shutdown();
  return 0;
}