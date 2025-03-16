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

class MultiThread : public rclcpp::Node {

  using RadarObjectArray = devastator_perception_msgs::msg::RadarObjectArray;
  using MyTestMsg = devastator_perception_msgs::msg::MyTest;
  using FrameMsg = devastator_perception_msgs::msg::Frame;
  using E171Msg = e171_msgs::msg::E171;

public:
    MultiThread(std::string node_name)
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

        subscription_1_ = this->create_subscription<MyTestMsg>("/my/test", 10, std::bind(&MultiThread::SubCall_1, this, std::placeholders::_1), sub1_opt);
        subscription_2_ = this->create_subscription<MyTestMsg>("/my/test", 10, std::bind(&MultiThread::SubCall_2, this, std::placeholders::_1), sub2_opt);
        subscription_3_ = this->create_subscription<E171Msg>("/my/e171", 10, std::bind(&MultiThread::SubCall_3, this, std::placeholders::_1), sub3_opt);
        subscription_4_ = this->create_subscription<FrameMsg>("/my/frame", 10, std::bind(&MultiThread::SubCall_4, this, std::placeholders::_1));
    }
    
    ~MultiThread() {

    }

    void SubCall_1(const MyTestMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << "SUB_1: ";
        ss << std::to_string(obj->a);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << std::endl;
        std::cout << ss.str() << std::endl;
    }

    void SubCall_2(const MyTestMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << "SUB_2: ";
        ss << std::to_string(obj->a);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << std::endl;
        std::cout << ss.str() << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    void SubCall_3(const E171Msg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << "SUB_3: ";
        ss << std::to_string(obj->data.size());
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << std::endl;
        std::cout << ss.str() << std::endl;
    }

    void SubCall_4(const FrameMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << "SUB_4: ";
        ss << std::to_string(obj->counter);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << std::endl;
        std::cout << ss.str() << std::endl;
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

    std::cout << "main thread id: " << gettid() << std::endl;

    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::executor::ExecutorArgs(), 2, true};
    // auto pubnode = std::make_shared<PublisherNode>();
    auto subnode = std::make_shared<MultiThread>("multi_thread");  // This contains BOTH subscriber callbacks.
                                                            // They will still run on different threads
                                                            // One Node. Two callbacks. Two Threads
    // executor.add_node(subnode);
    executor.add_node(subnode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

// 结论
// 1.可以通过增加callback_group来实现多线程
// 2.ros2的spin是单线程的，如果需要多线程，需要使用MultiThreadedExecutor
// 3.总的线程数量在创建MultiThreadedExecutor时指定，单个或者多个callback_group共享一个线程(callbck_group的数量>executor的线程数量时，会有多个callback_group共享一个线程)
// 5.一个或者多个callback_group会跟主线程保持一致，即使executor的线程数量大于callback_group的数量，也会有一个callback_group跟主线程保持一致
// 6.对于同一个消息，每接收到一次，所有的订阅该消息的回调函数都会被调用，但是调用的顺序是不确定的
// 7.同一个callback不断被执行中，其内部打印tid的话是变化的
// 8.同一个线程，同一时间，只会处理某一个callback
