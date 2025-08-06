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

/**
 * ros2跟dds的命名规范：
 * 1.参考 https://design.ros2.org/articles/topic_and_service_names.html#ros-specific-namespace-prefix
 * 2. 为了方便跟ros消息区别，所有通过ros创建的dds消息都会默认添加 /rx前缀，其中x是一个单字符，表示是ros中那个子系统
 *    例如：rt表示ros topic；rq表示ros request；rr表示ros response；rs表示ros service；rp表示ros parameter；ra表示ros action
 */

std::string GetCurrentTime() {
  auto now = std::chrono::system_clock::now();
  int seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  std::uint32_t nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000;
  std::stringstream ss;
  ss << "[" << seconds << "." << std::setfill('0') << std::setw(9) << nanoseconds << "]";
  return ss.str();
}

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(std::string topic_name, rclcpp::NodeOptions options, rclcpp::QoS qos)
    : Node(topic_name, options), options_(options), qos_(qos)
    {
      get_node_options(options_);
      qos_.keep_last(2);
      get_qos(qos_);
      
      publisher_ = this->create_publisher<devastator_perception_msgs::msg::MyTest>("/my/qos/topic", qos_);
      timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void get_node_options(rclcpp::NodeOptions & options) {
      std::cout << "================== get_node_options ==================" << std::endl;
      const rcl_node_options_t * node_options = options.get_rcl_node_options();
      std::cout << std::dec << "rcl_node_options_t domain_id: " << node_options->domain_id << std::endl;
      std::cout << std::boolalpha << "rcl_node_options_t use_global_arguments: " << node_options->use_global_arguments << std::endl;
      std::cout << std::boolalpha << "rcl_node_options_t enable_rosout: " << node_options->enable_rosout << std::endl;
      rcl_arguments_t rcl_arguments = node_options->arguments;
      std::cout << "rcl_arguments_t.impl address: " << rcl_arguments.impl << std::endl;

      std::cout << "=================== context ==================" << std::endl;
      rclcpp::Context::SharedPtr context = options.context();
      bool context_is_vaild = context->is_valid();
      std::cout << std::boolalpha << "Context is valid: " << context_is_vaild << std::endl;
      rclcpp::InitOptions init_options = context->get_init_options();
      std::cout << std::boolalpha << "InitOptions shutdown_on_sigint: " << init_options.shutdown_on_sigint << std::endl;
      bool auto_initialize_logging = init_options.auto_initialize_logging();
      std::cout << std::boolalpha << "InitOptions auto_initialize_logging: " << auto_initialize_logging << std::endl;
      const rcl_init_options_t * rcl_init_options = init_options.get_rcl_init_options();
      std::cout << "rcl_init_options_t.impl address: " << rcl_init_options->impl << std::endl;

      std::cout << "=================== arguments ==================" << std::endl;
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

    void timer_callback()
    {
      static std::uint64_t counter = 0;
      counter++;
      auto message = devastator_perception_msgs::msg::MyTest();
      message.a = counter;
      message.b = 2;
      message.c = true;
      RCLCPP_INFO(this->get_logger(), "Publishing %d!", message.a);
      publisher_->publish(message);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<devastator_perception_msgs::msg::MyTest>::SharedPtr publisher_;
    rclcpp::NodeOptions options_;
    rclcpp::QoS qos_;
};

void set_node_options([[maybe_unused]]rclcpp::NodeOptions & options) {

}

void set_qos([[maybe_unused]]rclcpp::QoS & qos) {

}

int main(int argc, char * argv[])
{
  std::string node_name = "minimal_publisher";
  rclcpp::NodeOptions options = rclcpp::NodeOptions();

  /**
   * typedef struct rmw_qos_profile_t
   * {
   *   enum rmw_qos_history_policy_t history;
   *   size_t depth;
   *   enum rmw_qos_reliability_policy_t reliability;
   *   enum rmw_qos_durability_policy_t durability;
   *   rmw_time_t deadline;
   *   rmw_time_t lifespan;
   *   enum rmw_qos_liveliness_policy_t liveliness;
   *   rmw_time_t liveliness_lease_duration;
   *   bool avoid_ros_namespace_conventions;
   * }
   */
  rclcpp::QoSInitialization qos_init1 = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default);
  rclcpp::QoSInitialization qos_init2 = rclcpp::QoSInitialization(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST, 20);
  rclcpp::QoS qos0(10);
  rclcpp::QoS qos1(qos_init1);
  rclcpp::QoS qos2(qos_init2, rmw_qos_profile_default);

  rclcpp::InitOptions init_options;
#if 0
  // 会导致 Ctrl+C 无法正常关闭节点
  init_options.shutdown_on_sigint = false;
#endif
  
  rclcpp::init(argc, argv, init_options);
  rclcpp::spin(std::make_shared<MinimalPublisher>(node_name, options, qos2));
  rclcpp::shutdown();
  return 0;
}