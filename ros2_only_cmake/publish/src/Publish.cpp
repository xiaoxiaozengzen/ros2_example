#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include "my_test/Publish.hpp"

namespace cgz {
MyPub::MyPub(const rclcpp::NodeOptions& options)
    :Node("my_test_node", options) {
        publisher_ =
            this->create_publisher<MyTest>("/my/test", 10);

        send_th_ = std::thread(std::bind(&MyPub::Init, this));

        // timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MyPub::Init, this));
    }

MyPub::~MyPub() {
    if (send_th_.joinable()) {
        send_th_.join();
    }
}

void MyPub::Init() {
    while(rclcpp::ok()) {
        MyTest msg;
        msg.a = 0x10;

        publisher_->publish(msg);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

}
}




int main(int argc, char** argv) {

  auto single_handle = [](int sig) {
    (void)sig;
    rclcpp::shutdown();
  };
  signal(SIGINT, single_handle);

  rclcpp::NodeOptions options;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cgz::MyPub>(options));
  rclcpp::shutdown();

  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(cgz::MyPub)