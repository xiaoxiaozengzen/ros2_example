#include <unistd.h>

#include <thread>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "devastator_perception_msgs/msg/radar_object_array.hpp"
#include "devastator_perception_msgs/msg/my_test.hpp"
#include "devastator_perception_msgs/msg/frame.hpp"
#include "e171_msgs/msg/e171.hpp"

/**
 * 可以参考ros2的官方文档：https://docs.ros.org/en/foxy/Concepts/About-Executors.html
 */

std::string GetCurrentTime() {
    auto now = std::chrono::system_clock::now();
    int seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    std::uint32_t nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000;
    std::stringstream ss;
    ss << "[" << seconds << "." << std::setfill('0') << std::setw(9) << nanoseconds << "]";
    return ss.str();
}

// 结论
// 1.可以通过增加callback_group来实现多线程
// 2.ros2的spin是单线程的，如果需要多线程，需要使用MultiThreadedExecutor
// 3.总的线程数量在创建MultiThreadedExecutor时指定，单个或者多个callback_group共享一个线程(callbck_group的数量>executor的线程数量时，会有多个callback_group共享一个线程)
// 5.一个或者多个callback_group会跟主线程保持一致，即使executor的线程数量大于callback_group的数量，也会有一个callback_group跟主线程保持一致
// 6.对于同一个消息，每接收到一次，所有的订阅该消息的回调函数都会被调用，但是调用的顺序是不确定的
// 7.同一个callback不断被执行中，其内部打印tid的话是变化的
// 8.同一个线程，同一时间，只会处理某一个callback
// 9.回调组有两种类型：
//   - MutuallyExclusive：互斥的，在该回调组中，回调函数顺序执行，只有一个回调可以在同一时间被执行
//   - Reentrant：可重入的，在该回调组中多个回调可以在同一时间被执行。每来一条消息就调用一个回调
// 10.属于不同回调组的回调函数总是可以并发执行

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
        callback_group_1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_2_ = this->create_callback_group(
        // rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::CallbackGroupType::Reentrant);
        callback_group_3_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_1_;
    
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_2_;

        auto sub3_opt = rclcpp::SubscriptionOptions();
        sub3_opt.callback_group = callback_group_3_;

        sub_group_1_num_1 = this->create_subscription<MyTestMsg>("/my/test", 10, std::bind(&MultiThread::callback_group_1_num_1, this, std::placeholders::_1), sub1_opt);
        sub_group_2_num_1 = this->create_subscription<MyTestMsg>("/my/test", 10, std::bind(&MultiThread::callback_group_2_num_1, this, std::placeholders::_1), sub2_opt);
        sub_group_2_num_2 = this->create_subscription<MyTestMsg>("/my/test", 10, std::bind(&MultiThread::callback_group_2_num_2, this, std::placeholders::_1), sub2_opt);
        sub_group_3_num_1 = this->create_subscription<E171Msg>("/my/e171", 10, std::bind(&MultiThread::callback_group_3_num_1, this, std::placeholders::_1), sub3_opt);
        sub_group_no_num_1 = this->create_subscription<FrameMsg>("/my/frame", 10, std::bind(&MultiThread::callback_group_no_num_1, this, std::placeholders::_1));
    }
    
    ~MultiThread() {

    }

    void callback_group_1_num_1(const MyTestMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << GetCurrentTime() << ": ";
        ss << __FUNCTION__ << ", ";
        ss << std::to_string(obj->a);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        std::cout << ss.str() << std::endl;
    }

    void callback_group_2_num_1(const MyTestMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << GetCurrentTime() << ": ";
        ss << __FUNCTION__ << ", ";
        ss << std::to_string(obj->a);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << ", counter: ";
        ss << std::to_string(counter_);
        std::cout << ss.str() << std::endl;

        counter_++;
    }

    void callback_group_2_num_2(const MyTestMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << GetCurrentTime() << ": ";
        ss << __FUNCTION__ << ", ";
        ss << std::to_string(obj->a);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        ss << ", counter: ";
        ss << std::to_string(counter_);
        std::cout << ss.str() << std::endl;

        counter_++;

        std::this_thread::sleep_for(std::chrono::milliseconds(5000));  // 模拟耗时操作
    }

    void callback_group_3_num_1(const E171Msg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << GetCurrentTime() << ": ";
        ss << __FUNCTION__ << ", ";
        ss << std::to_string(obj->data.size());
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        std::cout << ss.str() << std::endl;
    }

    void callback_group_no_num_1(const FrameMsg::SharedPtr obj) {
        auto this_tid = gettid();
        std::stringstream ss;
        ss << GetCurrentTime() << ": ";
        ss << __FUNCTION__ << ", ";
        ss << std::to_string(obj->counter);
        ss << ", tid: ";
        ss << std::to_string(static_cast<std::uint64_t>(this_tid));
        std::cout << ss.str() << std::endl;
    }
private:
    rclcpp::CallbackGroup::SharedPtr callback_group_1_{nullptr};
    rclcpp::CallbackGroup::SharedPtr callback_group_2_{nullptr};
    rclcpp::CallbackGroup::SharedPtr callback_group_3_{nullptr};

    int counter_{0};

    rclcpp::Subscription<MyTestMsg>::SharedPtr sub_group_1_num_1{nullptr};
    rclcpp::Subscription<MyTestMsg>::SharedPtr sub_group_2_num_1{nullptr};
    rclcpp::Subscription<MyTestMsg>::SharedPtr sub_group_2_num_2{nullptr};
    rclcpp::Subscription<E171Msg>::SharedPtr sub_group_3_num_1{nullptr};
    rclcpp::Subscription<FrameMsg>::SharedPtr sub_group_no_num_1{nullptr};

    rclcpp::Publisher<MyTestMsg>::SharedPtr publisher_{nullptr};

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    int thread_num = 0;
    if (argc >= 2) {
        thread_num = atoi(argv[1]);
    }

    std::cout << "main thread id: " << gettid() << ", thread_num: " << thread_num << std::endl;

    std::chrono::nanoseconds timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(2));

    /**
     * @brief 创建一个多线程的Executor
     * 
     * @param options 执行器选项
     * @param number_of_threads 线程池中线程的数量。默认为0，将会使用跟系统核心数量相同的线程数。
     * @param yield_before_execute 默认为false。如果为true，则在执行回调之前会调用std::this_thread::yield()，允许其他线程运行。这个选项可以用于提高多线程执行器的性能。
     * @param timeout 最大的等待时间，单位ns，默认为-1，表示没有超时限制。
     */
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::executor::ExecutorArgs(), static_cast<std::size_t>(thread_num), true, timeout};
    
    auto subnode = std::make_shared<MultiThread>("multi_thread");

    executor.add_node(subnode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
