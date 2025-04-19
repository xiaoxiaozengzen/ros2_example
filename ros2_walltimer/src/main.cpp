#include <iostream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

/**
 * 创建两个1s为周期的定时器，其中一个函数会阻塞5s。
 * 运行程序发现：
 * 1. callback是阻塞函数的话，定时器会受到函数阻塞影响
 * 2. 其他正常定时器也会被影响，导致周期变为5s
 */

class TimerNode : public rclcpp::Node
{
public:
  TimerNode()
  : Node("timer_node")
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TimerNode::timer_callback, this));
      
    over_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TimerNode::over_timer_callback, this));

    client_thread_ = std::thread(&TimerNode::client_thread, this);
    service_thread_ = std::thread(&TimerNode::service_thread, this);
  }

  ~TimerNode()
  {
    is_running_ = false;
    cv_.notify_all();

    if (client_thread_.joinable()) {
      client_thread_.join();
    }
    if (service_thread_.joinable()) {
      service_thread_.join();
    }
  }
private:
  void timer_callback()
  {
    count_++;
    RCLCPP_INFO(this->get_logger(), "Timer callback executed %u times", count_);
  }

  void over_timer_callback()
  {
    over_count_++;
    RCLCPP_INFO(this->get_logger(), "Over Timer callback executed %u times", over_count_);
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  void service_thread()
  {
    is_running_ = true;
    while (is_running_) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return is_over_; });
        RCLCPP_INFO(this->get_logger(), "Service thread wake up");
        is_over_ = false;
      }

      // 可以直接调用定时器的注册的回调函数
      timer_->execute_callback();
    }
  }

  void client_thread()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while (is_running_) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(this->get_logger(), "Client thread running");
      std::unique_lock<std::mutex> lock(mutex_);
      is_over_ = true;
      cv_.notify_one();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::uint32_t count_ = 0;

  rclcpp::TimerBase::SharedPtr over_timer_;
  std::uint32_t over_count_ = 0;

  std::mutex mutex_;
  std::condition_variable cv_;
  bool is_over_{false};
  bool is_running_{false};
  std::thread client_thread_;
  std::thread service_thread_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerNode>());
  rclcpp::shutdown();
  return 0;
}