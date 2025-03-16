#include <iostream>
#include <sstream>
#include <memory>
#include <functional>
#include <string>
#include <fstream>

#include <opencv4/opencv2/core.hpp> // OpenCV核心功能
#include <opencv4/opencv2/imgcodecs.hpp> // 图像编解码
#include <opencv4/opencv2/highgui.hpp> // GUI
#include <opencv4/opencv2/imgproc.hpp> // 图像处理

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "foxglove_msgs/msg/compressed_video.hpp"

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        sub_ = this->create_subscription<foxglove_msgs::msg::CompressedVideo>(
            "/sensor/cam_front_120/h264", 10, std::bind(&CameraNode::Callback, this, std::placeholders::_1));
    }

    void Callback(const foxglove_msgs::msg::CompressedVideo::SharedPtr msg) {
        std::cerr << "=============== " << count_++ << " ===============" << std::endl;
        std::stringstream ss;
        ss << "timestamp.sec: " << msg->timestamp.sec
           << ", timestamp.nanosec: " << msg->timestamp.nanosec
           << ", frame id: " << msg->frame_id
           << ", data size: " << msg->data.size()
           << ", format: " << msg->format
           << std::endl;
        std::cerr << ss.str();
        ss.str("");
        ss << "header.sec: " << msg->header.stamp.sec
           << ", header.nanosec: " << msg->header.stamp.nanosec
           << ", header.frame id: " << msg->header.frame_id
           << std::endl;
        std::cerr << ss.str();

        // std::string filename = dir_ + "/" + std::to_string(count_) + ".264";
        // std::ofstream ofs;
        // ofs.open(filename, std::ios::out | std::ios::binary);
        // if(ofs.good()) {
        //   ofs.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
        // } else {
        //   std::cerr << "Failed to open file: " << filename << std::endl;
        // }
        // ofs.close();
    }

private:
    rclcpp::Subscription<foxglove_msgs::msg::CompressedVideo>::SharedPtr sub_;
    std::uint64_t count_ = 0;
    std::string dir_ = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/ros2_camera/video";
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}