#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include "my_publish/Publish.hpp"

namespace cgz {

  MyPub::MyPub(const rclcpp::NodeOptions& options)
    :Node("my_publish_node", options) {
      mytest_pub_ = this->create_publisher<MyTestMsg>("/my/test", 10);
      mytest_th_ = std::thread(std::bind(&MyPub::MytestPub, this));

      frame_pub_ = this->create_publisher<FrameMsg>("/my/frame", 10);
      frame_th_ = std::thread(std::bind(&MyPub::FramePub, this));

      e171_pub_ = this->create_publisher<E171Msg>("/my/e171", 10);
      e171_th_ = std::thread(std::bind(&MyPub::E171Pub, this));
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
      msg.a = count;
      msg.b = 0x20;

      mytest_pub_->publish(msg);
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
    msg.counter = count;
    
    frame_pub_->publish(msg);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
void MyPub::E171Pub() {
  E171Msg msg;
  auto now = std::chrono::system_clock::now();
  msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
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