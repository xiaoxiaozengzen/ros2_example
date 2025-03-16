#include <thread>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "devastator_perception_msgs/msg/my_test.hpp"
#include "devastator_perception_msgs/msg/frame.hpp"
#include "e171_msgs/msg/e171.hpp"

using MyTestMsg = devastator_perception_msgs::msg::MyTest;
using FrameMsg = devastator_perception_msgs::msg::Frame;
using E171Msg = e171_msgs::msg::E171;

class MyPubSecondNode : public rclcpp::Node {
public:
    MyPubSecondNode(const rclcpp::NodeOptions& options);
    ~MyPubSecondNode();

    void MytestPub();
    // void FramePub();
    // void E171Pub();

    rclcpp::Publisher<MyTestMsg>::SharedPtr mytest_pub_{nullptr};
    // rclcpp::Publisher<FrameMsg>::SharedPtr frame_pub_{nullptr};
    // rclcpp::Publisher<E171Msg>::SharedPtr e171_pub_{nullptr};
    
    std::thread mytest_th_;
    // std::thread frame_th_;
    // std::thread e171_th_;
};

MyPubSecondNode::MyPubSecondNode(const rclcpp::NodeOptions& options)
    :Node("my_second_publish", options) {
      mytest_pub_ = this->create_publisher<MyTestMsg>("/your/test", 10);
      mytest_th_ = std::thread(std::bind(&MyPubSecondNode::MytestPub, this));

      // frame_pub_ = this->create_publisher<FrameMsg>("/my/frame", 10);
      // frame_th_ = std::thread(std::bind(&MyPubSecondNode::FramePub, this));

      // e171_pub_ = this->create_publisher<E171Msg>("/my/e171", 10);
      // e171_th_ = std::thread(std::bind(&MyPubSecondNode::E171Pub, this));
    }

MyPubSecondNode::~MyPubSecondNode() {
  if (mytest_th_.joinable()) {
    mytest_th_.join();
  }
  // if (frame_th_.joinable()) {
  //   frame_th_.join();
  // }
  // if (e171_th_.joinable()) {
  //   e171_th_.join();
  // }
}

void MyPubSecondNode::MytestPub() {
  int count = 0;
  while(rclcpp::ok()) {
      count++;
      MyTestMsg msg;
      msg.a = 0x2;
      msg.b = 0x10;

      std::cerr << "my_second_publish msg.a: " << (std::uint32_t)msg.a << std::endl;
      mytest_pub_->publish(msg);
      std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

// void MyPubSecondNode::FramePub() {
//   std::string frame = "frame";
//   FrameMsg msg;
//   msg.frame_id = frame;
//   std::uint32_t count = 0;
//   while(rclcpp::ok()) {
//     count++;
//     msg.counter = count;
    
//     frame_pub_->publish(msg);
    
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//   }
// }
// void MyPubSecondNode::E171Pub() {
//   E171Msg msg;
//   msg.data.push_back(1);

//   while(rclcpp::ok()) {
//     e171_pub_->publish(msg);

//     std::this_thread::sleep_for(std::chrono::seconds(2));
//   }
// }


int main(int argc, char** argv) {
  rclcpp::NodeOptions options = rclcpp::NodeOptions();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPubSecondNode>(options));
  rclcpp::shutdown();

  return 0;
}