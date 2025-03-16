#pragma once

#include <thread>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "devastator_perception_msgs/msg/my_test.hpp"
#include "devastator_perception_msgs/msg/frame.hpp"
#include "e171_msgs/msg/e171.hpp"

namespace cgz {

using MyTestMsg = devastator_perception_msgs::msg::MyTest;
using FrameMsg = devastator_perception_msgs::msg::Frame;
using E171Msg = e171_msgs::msg::E171;

class MyPub : public rclcpp::Node {
public:
    MyPub(const rclcpp::NodeOptions& options);
    ~MyPub();

    void MytestPub();
    void FramePub();
    void E171Pub();

    rclcpp::Publisher<MyTestMsg>::SharedPtr mytest_pub_{nullptr};
    rclcpp::Publisher<FrameMsg>::SharedPtr frame_pub_{nullptr};
    rclcpp::Publisher<E171Msg>::SharedPtr e171_pub_{nullptr};
    
    std::thread mytest_th_;
    std::thread frame_th_;
    std::thread e171_th_;
};
}

