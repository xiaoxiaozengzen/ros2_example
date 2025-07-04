#pragma once

#include <thread>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "devastator_perception_msgs/msg/my_test.hpp"

namespace cgz {

class MyPub : public rclcpp::Node {
using MyTest = devastator_perception_msgs::msg::MyTest;
public:
    MyPub(const rclcpp::NodeOptions& options);
    ~MyPub();

    void Init();

    rclcpp::Publisher<MyTest>::SharedPtr publisher_{nullptr};
    // rclcpp::TimerBase::SharedPtr timer_{nullptr};

    std::thread send_th_;
};
}

