#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include "my_publish/Publish.hpp"
#include "interface_cmake/plus/plus.hpp"
#include "interface_cmake_dependcy/sub/sub.hpp"

namespace cgz {

  MyPub::MyPub(const rclcpp::NodeOptions& options)
    :Node("my_publish_node", options) {
      mytest_pub_ = this->create_publisher<MyTestMsg>("/my/test", 10);
      mytest_th_ = std::thread(std::bind(&MyPub::MytestPub, this));

      frame_pub_ = this->create_publisher<FrameMsg>("/my/frame", 10);
      frame_th_ = std::thread(std::bind(&MyPub::FramePub, this));

      e171_pub_ = this->create_publisher<E171Msg>("/my/e171", 10);
      e171_th_ = std::thread(std::bind(&MyPub::E171Pub, this));

      first_marker_pub_ = this->create_publisher<MarkerArrayMsg>("/my/first_marker", 10);
      second_marker_pub_ = this->create_publisher<MarkerArrayMsg>("/my/second_marker", 10);
      first_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyPub::FirstTimer, this));
      second_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyPub::SecondTimer, this));
    }

MyPub::~MyPub() {
  if (mytest_th_.joinable()) {
    mytest_th_.join();
  }
  if (frame_th_.joinable()) {
    frame_th_.join();
  }
  if (e171_th_.joinable()) {
    e171_th_.join();
  }
}

void MyPub::MytestPub() {
  int count = 0;
  while(rclcpp::ok()) {
      count++;
      MyTestMsg msg;
      msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      msg.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 1000000000;
      msg.a = count;
      msg.b = 0x20;

      mytest_pub_->publish(msg);
      int a = 10;
      int b = 20;
      int result = plus(a, b);
      RCLCPP_INFO(this->get_logger(), "Result of plus: %d", result);
      int sub_result = sub(a, b);
      RCLCPP_INFO(this->get_logger(), "Result of sub: %d", sub_result);
      std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void MyPub::FramePub() {
  std::string frame = "frame";
  FrameMsg msg;
  msg.frame_id = frame;
  std::uint32_t count = 0;
  while(rclcpp::ok()) {
    count++;
    msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    msg.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 1000000000;
    msg.counter = count;
    
    frame_pub_->publish(msg);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));


  }
}
void MyPub::E171Pub() {
  E171Msg msg;
  auto now = std::chrono::system_clock::now();
  msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  msg.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000;
  msg.header.frame_id = "e171";
  msg.data.push_back('1');
  msg.data.push_back('2');
  msg.data.push_back('3');
  msg.data.push_back('4');
  msg.data.push_back('5');
  msg.data.push_back('6');
  msg.data.push_back('7');
  msg.data.push_back('8');

  while(rclcpp::ok()) {
    e171_pub_->publish(msg);

    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

void MyPub::FirstTimer() {
  static int count = 0;
  count++;

  MarkerArrayMsg msg;
  MarkerMsg marker;
  marker.header.frame_id = "ego_vehicle";
  marker.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  marker.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 1000000000;
  marker.ns = "first";
  marker.id = count;
  marker.type = MarkerMsg::SPHERE;
  marker.action = MarkerMsg::ADD;
  marker.pose.position.x = 10.0 + count;
  marker.pose.position.y = 20.0;
  marker.pose.position.z = 30.0;
  marker.scale.x = 5.0;
  marker.scale.y = 5.0;
  marker.scale.z = 5.0;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 1000000000;

  msg.markers.push_back(marker);
  
  first_marker_pub_->publish(msg);
}

void MyPub::SecondTimer() {
  static int count = 0;
  count++;

  MarkerArrayMsg msg;
  MarkerMsg marker;
  marker.header.frame_id = "ego_vehicle";
  marker.header.stamp.sec = 0;
  marker.header.stamp.nanosec = 0;
  marker.ns = "second";
  marker.id = count;
  marker.type = MarkerMsg::SPHERE;
  marker.action = MarkerMsg::ADD;
  marker.pose.position.x = 40.0 + count;
  marker.pose.position.y = 50.0;
  marker.pose.position.z = 60.0;
  marker.scale.x = 5.0;
  marker.scale.y = 5.0;
  marker.scale.z = 5.0;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 1000000000;

  msg.markers.push_back(marker);
  
  second_marker_pub_->publish(msg);
}

} // namespace cgz




// int main(int argc, char** argv) {

//   auto single_handle = [](int sig) {
//     (void)sig;
//     rclcpp::shutdown();
//   };
//   signal(SIGINT, single_handle);

//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Pub>());
//   rclcpp::shutdown();

//   return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(cgz::MyPub)