#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/context.hpp"

#include "devastator_perception_msgs/msg/my_test.hpp"

using namespace std::chrono_literals;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(std::string topic_name, rclcpp::NodeOptions options, rclcpp::QoS qos)
    : Node(topic_name, options), options_(options), qos_(qos)
    {
      get_qos(qos_);
      subscriber_ = this->create_subscription<devastator_perception_msgs::msg::MyTest>("/my/qos/topic", qos_, std::bind(&MinimalSubscriber::callback, this, std::placeholders::_1));
    }

  private:
    void callback([[maybe_unused]]const devastator_perception_msgs::msg::MyTest::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Receiveing %d!", msg->a);
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
      std::size_t depth = profile.depth;
      std::cout << std::dec << "Depth: " << depth << std::endl;

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

      rmw_qos_durability_policy_t durability = profile.durability;
      if(durability == rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT)
      {
        std::cout << "Durability: RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT" << std::endl;
      }
      else if(durability == rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      {
        std::cout << "Durability: RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL" << std::endl;
      }
      else if(durability == rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
      {
        std::cout << "Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE" << std::endl;
      }
      else
      {
        std::cout << "Durability: Unknown" << std::endl;
      }

      rmw_time_t deadline = profile.deadline;
      std::cout << "Deadline: " << deadline.sec << "s " << deadline.nsec << "ns" << std::endl;

      rmw_time_t lifespan = profile.lifespan;
      std::cout << "Lifespan: " << lifespan.sec << "s " << lifespan.nsec << "ns" << std::endl;

      rmw_qos_liveliness_policy_t liveliness = profile.liveliness;
      if(liveliness == rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT)
      {
        std::cout << "Liveliness: RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT" << std::endl;
      }
      else if(liveliness == rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
      {
        std::cout << "Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC" << std::endl;
      }
      else if(liveliness == rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
      {
        std::cout << "Liveliness: RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC" << std::endl;
      }
      else
      {
        std::cout << "Liveliness: Unknown" << std::endl;
      }

      rmw_time_t liveliness_lease_duration = profile.liveliness_lease_duration;
      std::cout << "Liveliness Lease Duration: " << liveliness_lease_duration.sec << "s " << liveliness_lease_duration.nsec << "ns" << std::endl;

      /**
       * 如果为true，则表示任何ROS Specific Namespace Prefix都会被避免。
       * 例如：对于DDS来说，ros2消息将不在默认加前缀"rt"，这一般用于通过ROS2 topic直接跟原始的dds进行通信
       */
      bool avoid_ros_namespace_conventions = profile.avoid_ros_namespace_conventions;
      std::cout << std::boolalpha << "Avoid ROS Namespace Conventions: " << avoid_ros_namespace_conventions << std::endl;
    }

  private:
    rclcpp::Subscription<devastator_perception_msgs::msg::MyTest>::SharedPtr subscriber_;
    rclcpp::NodeOptions options_;
    rclcpp::QoS qos_;
};


int main(int argc, char * argv[])
{
  std::string node_name = "minimal_subscriber";
  rclcpp::NodeOptions options = rclcpp::NodeOptions();

  rclcpp::QoSInitialization qos_init = rclcpp::QoSInitialization(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL, 10);
  rclcpp::QoS qos(qos_init, rmw_qos_profile_default);
  qos.best_effort();
  /**
   * 这个设置跟publisher端不一致，导致消息无法接收
   */
  qos.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  rclcpp::InitOptions init_options;
#if 0
  // 会导致 Ctrl+C 无法正常关闭节点
  init_options.shutdown_on_sigint = false;
#endif
  
  rclcpp::init(argc, argv, init_options);
  rclcpp::spin(std::make_shared<MinimalSubscriber>(node_name, options, qos));
  rclcpp::shutdown();
  return 0;
}