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
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "deva_perception_msgs/msg/lane_arrayv2.hpp"

/**
 * ros2中常用的坐标系有以下几种：
 * 1. base_link坐标系，其跟机器人的质心对齐，x轴指向前方，y轴指向左侧，z轴指向上方。类比汽车坐标系的ego坐标系。
 * 2. base_footprint坐标系，其跟机器人底部接触地面的点对齐，x轴指向前方，y轴指向左侧，z轴指向上方。类比汽车坐标系的rfu坐标系。
 * 3. map坐标系，其为全局坐标系，即世界坐标系(UTM)
 * 4. odom坐标系，其为局部坐标系，通常是机器人启动时的位置为原。此时其在map中是固定的，但机器人在odom中是运动的，通过里程计等方式进行更新。
 *    时间久了会有漂移，因此其在局部范围是比较准确的，但在全局范围是不准确的。
 * 
 */

/**
 * 一般对于车道线，会使用三次多项式来进行拟合，这样可以不用传输所有的点，只需要传输多项式的系数即可。
 * 以y= ax^3 + bx^2 + cx + d为例，传输a,b,c,d四个系数即可。其中：
 * - a: 三次项系数，表示曲线的曲率的变化率，a越大，曲线的弯曲程度变化越明显。
 * - b: 二次项系数，表示曲线的曲率，b越大，曲线的弯曲程度越明显。
 * - c: 一次项系数，表示曲线的斜率。c越大，曲线整体倾斜越陡。
 * - d: 常数项，决定曲线在x=0时的截距，即曲线的上下平移量。例如在自车坐标系下，车道线距离自车位置的远近。
 */

class LaneLineNode : public rclcpp::Node {
  using LaneArrayV2 = deva_perception_msgs::msg::LaneArrayv2;
public:
    LaneLineNode() : Node("lane_line_node") {
        sub_ = this->create_subscription<LaneArrayV2>(
            "/perception/lane_array_result", 10, std::bind(&LaneLineNode::Callback, this, std::placeholders::_1));
    }

    void Callback(const LaneArrayV2::SharedPtr msg) {
        std::cerr << "=============== " << count_++ << " ===============" << std::endl;
        std::stringstream ss;
        ss << "header.sec: " << msg->header.stamp.sec
           << ", header.nanosec: " << msg->header.stamp.nanosec
           << ", header.frame id: " << msg->header.frame_id
           << std::endl;
        std::cerr << ss.str();

        cv::Mat image = cv::Mat::ones(2160, 3840, CV_8UC3);
        for(auto lane : msg->lane_array) {
            for(auto point : lane.waypoints) {
                Eigen::Vector4d point_ego(point.x, point.y, point.z, 1);
                Eigen::Vector4d point_rfu = ego2rfu_matrix4d_ * point_ego;
                Eigen::Vector4d point_camera = rfu2camera_matrix4d_ * point_rfu;
                Eigen::Vector3d point_pixel = camera2pixel_matrix3d_ * point_camera.head(3);
                point_pixel /= point_pixel(2);  // 归一化

                cv::Point2d point2d(point_pixel(0), point_pixel(1));
                cv::circle(image, point2d, 5, cv::Scalar(0, 0, 255), -1);
            }
        }

        std::string frame_num = "frame " + std::to_string(count_);
        cv::putText(image, frame_num, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        
        if(count_ < 200) {
          video_writer_.write(image);
        }

        if(count_ == 200) {
          video_writer_.release();
          std::cerr << "Video saved." << std::endl;
        }
    }

    int GetMatrix() {
      // ego2rfu
      std::string rfu2ego_file = calibration_path_ + "/lidar_params/lidar_ego.json";
      std::ifstream ifs_rfu2ego(rfu2ego_file);
      if (!ifs_rfu2ego.is_open()) {
          std::cerr << "Failed to open file: " << rfu2ego_file << std::endl;
          return -1;
      }
      std::stringstream ss_rfu2ego;
      ss_rfu2ego << ifs_rfu2ego.rdbuf();
      std::string json_str_rfu2ego = ss_rfu2ego.str();
      nlohmann::json json_data_rfu2ego = nlohmann::json::parse(json_str_rfu2ego);
      std::array<double, 3> translation_rfu2ego;
      std::array<double, 4> rotation_rfu2ego;
      translation_rfu2ego[0] = json_data_rfu2ego.at("transform").at("translation").at("x");
      translation_rfu2ego[1] = json_data_rfu2ego.at("transform").at("translation").at("y");
      translation_rfu2ego[2] = json_data_rfu2ego.at("transform").at("translation").at("z");
      rotation_rfu2ego[0] = json_data_rfu2ego.at("transform").at("rotation").at("w");
      rotation_rfu2ego[1] = json_data_rfu2ego.at("transform").at("rotation").at("x");
      rotation_rfu2ego[2] = json_data_rfu2ego.at("transform").at("rotation").at("y");
      rotation_rfu2ego[3] = json_data_rfu2ego.at("transform").at("rotation").at("z");
      Eigen::Vector3d translation_eigen_rfu2ego(translation_rfu2ego[0], translation_rfu2ego[1], translation_rfu2ego[2]);
      Eigen::Translation3d translation_matrix_rfu2ego(translation_eigen_rfu2ego);
      Eigen::Quaterniond rotation_eigen_rfu2ego(rotation_rfu2ego[0], rotation_rfu2ego[1], rotation_rfu2ego[2], rotation_rfu2ego[3]);
      Eigen::Affine3d rfu2ego_affine = translation_matrix_rfu2ego * rotation_eigen_rfu2ego;
      Eigen::Matrix4d rfu2ego_matrix4d = rfu2ego_affine.matrix();
      ego2rfu_matrix4d_ = rfu2ego_matrix4d.inverse();
      std::cout << "ego2rfu_matrix4d: " << std::endl << ego2rfu_matrix4d_ << std::endl;

      // rfu2camera
      std::string camera_name = "cam_front_30";
      std::string rfu2camera_file = calibration_path_ + "/camera_params/" + camera_name + "_extrinsic.json";
      std::ifstream ifs_rfu2camera(rfu2camera_file);
      if (!ifs_rfu2camera.is_open()) {
          std::cerr << "Failed to open file: " << rfu2camera_file << std::endl;
          return -1;
      }
      std::stringstream ss_rfu2camera;
      ss_rfu2camera << ifs_rfu2camera.rdbuf();
      std::string json_str_rfu2camera = ss_rfu2camera.str();
      nlohmann::json json_data_rfu2camera = nlohmann::json::parse(json_str_rfu2camera);
      std::array<double, 3> translation_rfu2camera;
      std::array<double, 4> rotation_rfu2camera;
      translation_rfu2camera[0] = json_data_rfu2camera.at("transform").at("translation").at("x");
      translation_rfu2camera[1] = json_data_rfu2camera.at("transform").at("translation").at("y");
      translation_rfu2camera[2] = json_data_rfu2camera.at("transform").at("translation").at("z");
      rotation_rfu2camera[0] = json_data_rfu2camera.at("transform").at("rotation").at("w");
      rotation_rfu2camera[1] = json_data_rfu2camera.at("transform").at("rotation").at("x");
      rotation_rfu2camera[2] = json_data_rfu2camera.at("transform").at("rotation").at("y");
      rotation_rfu2camera[3] = json_data_rfu2camera.at("transform").at("rotation").at("z");
      Eigen::Vector3d translation_eigen_rfu2camera(translation_rfu2camera[0], translation_rfu2camera[1], translation_rfu2camera[2]);
      Eigen::Translation3d translation_matrix_rfu2camera(translation_eigen_rfu2camera);
      Eigen::Quaterniond rotation_eigen_rfu2camera(rotation_rfu2camera[0], rotation_rfu2camera[1], rotation_rfu2camera[2], rotation_rfu2camera[3]);
      Eigen::Affine3d rfu2camera_affine = translation_matrix_rfu2camera * rotation_eigen_rfu2camera;
      Eigen::Matrix4d rfu2camera_matrix4d = rfu2camera_affine.matrix();
      rfu2camera_matrix4d_ = rfu2camera_matrix4d;
      std::cout << "rfu2camera_matrix4d: " << std::endl << rfu2camera_matrix4d << std::endl;

      // camera2pixel
      std::string camera2pixel_file = calibration_path_ + "/camera_params/" + camera_name + "_intrinsic.json";
      std::ifstream ifs_camera2pixel(camera2pixel_file);
      if (!ifs_camera2pixel.is_open()) {
          std::cerr << "Failed to open file: " << camera2pixel_file << std::endl;
          return -1;
      }
      std::stringstream ss_camera2pixel;
      ss_camera2pixel << ifs_camera2pixel.rdbuf();
      std::string json_str_camera2pixel = ss_camera2pixel.str();
      nlohmann::json json_data_camera2pixel = nlohmann::json::parse(json_str_camera2pixel);
      double fx = json_data_camera2pixel["K"][0][0];
      double fy = json_data_camera2pixel["K"][1][1];
      double cx = json_data_camera2pixel["K"][0][2];
      double cy = json_data_camera2pixel["K"][1][2];
      Eigen::Matrix3d camera2pixel_matrix3d;
      camera2pixel_matrix3d << fx, 0, cx,
                               0, fy, cy,
                               0, 0, 1;
      camera2pixel_matrix3d_ = camera2pixel_matrix3d;
      std::cout << "camera2pixel_matrix3d: " << std::endl << camera2pixel_matrix3d << std::endl;

      video_writer_.open(video_file_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(3840, 2160));
      if (!video_writer_.isOpened()) {
          std::cerr << "Failed to open video file: " << video_file_ << std::endl;
          return -1;
      }

      return 0;
    }

private:
    rclcpp::Subscription<LaneArrayV2>::SharedPtr sub_;
    std::uint64_t count_ = 0;
    std::string video_file_ = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/ros2_laneline/output/output_1.avi";
    std::string calibration_path_ = "/mnt/workspace/cgz_workspace/Exercise/eigen_example/calibration";

    Eigen::Matrix4d ego2rfu_matrix4d_;
    Eigen::Matrix4d rfu2camera_matrix4d_;
    Eigen::Matrix3d camera2pixel_matrix3d_;

    cv::VideoWriter video_writer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneLineNode>();
    int ret = node->GetMatrix();
    if(ret != 0) {
        std::cerr << "Failed to get matrix." << std::endl;
        return -1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}