#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include "para_example/Publish.hpp"

namespace cgz {
MyPub::MyPub(const rclcpp::NodeOptions& options)
    :Node("para_example_node", options) {

        this->declare_parameter("path", "");
        if(this->has_parameter("path")) {
            this->get_parameter("path", path_);
            std::cerr << "path: " << path_ << std::endl;
        } else {
          std::cerr << "No path parameter" << std::endl;
        }

        publisher_ =
            this->create_publisher<MyTest>("/my/test", 10);

        send_th_ = std::thread(std::bind(&MyPub::Init, this));

        sub_ = this->create_subscription<RadarObjectArrayMsg>("/radar/objects", 10, [](const RadarObjectArrayMsg::SharedPtr msg) {
            std::cerr << "sub: " << msg->objects.size() << std::endl;
        });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MyPub::Print, this));
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
        msg.b = 0x20;
        // msg.c = true;

        publisher_->publish(msg);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

}

void MyPub::Print() {
    std::cerr << "timer" << std::endl;
}

}


int main(int argc, char** argv) {

  auto single_handle = [](int sig) {
    (void)sig;
    rclcpp::shutdown();
  };
  signal(SIGINT, single_handle);

  for(int i = 0; i < argc; ++i) {
    std::cerr << "argv[" << i << "]: " << argv[i] << std::endl;
  }

  rclcpp::NodeOptions options;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cgz::MyPub>(options));
  rclcpp::shutdown();

  return 0;
}