#include <unistd.h>

#include <thread>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "devastator_perception_msgs/msg/radar_object_array.hpp"
#include "devastator_perception_msgs/msg/my_test.hpp"
#include "devastator_perception_msgs/msg/frame.hpp"
#include "e171_msgs/msg/e171.hpp"

// 可以参考ros2的官方文档：https://docs.ros.org/en/foxy/Concepts/About-Executors.html

class RemappingNode : public rclcpp::Node {

  using RadarObjectArray = devastator_perception_msgs::msg::RadarObjectArray;
  using MyTestMsg = devastator_perception_msgs::msg::MyTest;
  using FrameMsg = devastator_perception_msgs::msg::Frame;
  using E171Msg = e171_msgs::msg::E171;

public:
    RemappingNode(std::string node_name)
    :Node(node_name) 
    {
        // 创建回调组
        callback_group_subscriber1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber3_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
    
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;

        auto sub3_opt = rclcpp::SubscriptionOptions();
        sub3_opt.callback_group = callback_group_subscriber3_;

        subscription_1_ = this->create_subscription<MyTestMsg>("/my/test", 10, std::bind(&RemappingNode::SubCall_1, this, std::placeholders::_1), sub1_opt);
        subscription_2_ = this->create_subscription<MyTestMsg>("/my/test", 10, std::bind(&RemappingNode::SubCall_2, this, std::placeholders::_1), sub2_opt);
        subscription_3_ = this->create_subscription<E171Msg>("/my/e171", 10, std::bind(&RemappingNode::SubCall_3, this, std::placeholders::_1), sub3_opt);
        subscription_4_ = this->create_subscription<FrameMsg>("/my/frame", 10, std::bind(&RemappingNode::SubCall_4, this, std::placeholders::_1));
    }
    
    ~RemappingNode() {

    }

    void SubCall_1(const MyTestMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << "SUB_1: ";
        ss << std::to_string(obj->a);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << std::endl;
        std::cerr << ss.str() << std::endl;
    }

    void SubCall_2(const MyTestMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << "SUB_2: ";
        ss << std::to_string(obj->a);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << std::endl;
        std::cerr << ss.str() << std::endl;
    }

    void SubCall_3(const E171Msg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << "SUB_3: ";
        ss << std::to_string(obj->data.size());
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << std::endl;
        std::cerr << ss.str() << std::endl;
    }

    void SubCall_4(const FrameMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << "SUB_4: ";
        ss << std::to_string(obj->counter);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << std::endl;
        std::cerr << ss.str() << std::endl;
    }
private:
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_{nullptr};
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_{nullptr};
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber3_{nullptr};

    rclcpp::Subscription<MyTestMsg>::SharedPtr subscription_1_{nullptr};
    rclcpp::Subscription<MyTestMsg>::SharedPtr subscription_2_{nullptr};
    rclcpp::Subscription<E171Msg>::SharedPtr subscription_3_{nullptr};
    rclcpp::Subscription<FrameMsg>::SharedPtr subscription_4_{nullptr};
    rclcpp::Publisher<MyTestMsg>::SharedPtr publisher_{nullptr};

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    std::cerr << "================ros2_remapping======================" << std::endl;
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::executor::ExecutorArgs(), 4, true};

    auto subnode = std::make_shared<RemappingNode>("ros2_remapping");
    executor.add_node(subnode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

