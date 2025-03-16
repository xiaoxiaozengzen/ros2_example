#pragma once

#include <thread>
#include <iostream>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "devastator_perception_msgs/msg/my_test.hpp"
#include "devastator_perception_msgs/msg/radar_object.hpp"
#include "devastator_perception_msgs/msg/radar_object_array.hpp"

namespace cgz {
using MyTest = devastator_perception_msgs::msg::MyTest;
using RadarObjectMsg = devastator_perception_msgs::msg::RadarObject;
using RadarObjectArrayMsg = devastator_perception_msgs::msg::RadarObjectArray;

class MyPub : public rclcpp::Node {
public:
    MyPub(const rclcpp::NodeOptions& options);
    ~MyPub();

    void Init();

    rclcpp::Publisher<MyTest>::SharedPtr publisher_{nullptr};
    rclcpp::Subscription<RadarObjectArrayMsg>::SharedPtr sub_{nullptr};
    
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    void Print();

    std::thread send_th_;

    std::string path_;
    int count_;
    std::vector<int> vec_;
};
}

