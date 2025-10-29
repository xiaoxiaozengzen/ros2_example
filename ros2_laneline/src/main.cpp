#include <iostream>
#include <sstream>
#include <memory>
#include <functional>
#include <string>
#include <fstream>
#include <thread>
#include <atomic>
#include <deque>
#include <iomanip>

extern "C" {
#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libavutil/pixfmt.h"
#include "libavutil/avutil.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libswscale/swscale.h"
}

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
#include "foxglove_msgs/msg/compressed_video.hpp"
#include "ros2_laneline/message_queue.hpp"

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
  using CompressedVideo = foxglove_msgs::msg::CompressedVideo;
public:
    LaneLineNode() : Node("lane_line_node") {

    }

    ~LaneLineNode() {
        running_.store(false);

        if(sync_thread_.joinable()) {
            sync_thread_.join();
        }

        if(video_writer_.isOpened()) {
            video_writer_.release();
        }

        if(packet) {
            av_packet_free(&packet);
            packet = nullptr;
        }

        if(frame) {
            av_frame_free(&frame);
            frame = nullptr;
        }

        if(parser) {
            av_parser_close(parser);
            parser = nullptr;
        }

        if(codec_context) {
            avcodec_free_context(&codec_context);
            codec_context = nullptr;
        }

        if(format_context) {
            avformat_free_context(format_context);
            format_context = nullptr;
        }
    }

    bool Init() {
        int get_matrix_result = GetMatrix();
        if(get_matrix_result != 0) {
            std::cerr << "Failed to get matrix." << std::endl;
            return false;
        }

        lane_sub_ = this->create_subscription<LaneArrayV2>(
            "/perception/lane_array_result", 10, std::bind(&LaneLineNode::LaneCallback, this, std::placeholders::_1));

        video_sub_ = this->create_subscription<CompressedVideo>(
            "/sensor/cam_front_120/h265", 10, std::bind(&LaneLineNode::CompressedVideoCallback, this, std::placeholders::_1));

        format_context = avformat_alloc_context();
        if(format_context == nullptr) {
            std::cerr << "Could not allocate format context." << std::endl;
            return false;
        }

        codec_context = avcodec_alloc_context3(nullptr);
        if(codec_context == nullptr) {
            std::cerr << "Could not allocate codec context." << std::endl;
            return false;
        }
        codec_context->codec_type = AVMEDIA_TYPE_VIDEO;
        codec_context->codec_id = AV_CODEC_ID_H265;

        AVCodec *codec = avcodec_find_decoder(codec_context->codec_id);
        if(codec == nullptr) {
            std::cerr << "Could not find decoder for codec id " << codec_context->codec_id << std::endl;
            return false;
        }

        int ret = avcodec_open2(codec_context, codec, nullptr);
        if(ret < 0) {
            std::cerr << "Could not open codec." << std::endl;
            return false;
        }

        parser = av_parser_init(codec_context->codec_id);
        if(parser == nullptr) {
            std::cerr << "Could not initialize parser." << std::endl;
            return false;
        }

        packet = av_packet_alloc();
        frame = av_frame_alloc();
        if(packet == nullptr || frame == nullptr) {
            std::cerr << "Could not allocate packet or frame." << std::endl;
            return false;
        }

        running_.store(true);
        sync_thread_ = std::thread(&LaneLineNode::Sync, this);

        return true;
    }

    void LaneCallback(const LaneArrayV2::SharedPtr msg) {
        static double last_timestamp = 0.0;
        double current_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if(current_timestamp - last_timestamp < 0.05) {
            return;
        }
        last_timestamp = current_timestamp;

        {
            std::lock_guard<std::mutex> lock(lane_mutex_);
            if(lane_queue_.size() >= max_queue_size_) {
                lane_queue_.pop_front();
            }

            lane_queue_.push_back(*msg.get());
        }
        
    }

    void CompressedVideoCallback(const CompressedVideo::SharedPtr msg) {
        static double last_timestamp = 0.0;
        double current_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if(current_timestamp - last_timestamp < 0.05) {
            return;
        }
        last_timestamp = current_timestamp;

        {
            std::lock_guard<std::mutex> lock(video_mutex_);
            if(video_queue_.size() >= max_queue_size_) {
                video_queue_.pop_front();
            }

            video_queue_.push_back(*msg.get());
        }

        video_msg_queue_.enqueue(*msg.get());
    }

    static cv::Mat avframe_to_bgr_mat(AVFrame* frame) {
        if (!frame) {
            return cv::Mat();
        }

        std::vector<uint8_t> yuv_data;
        // Y
        for(int i = 0; i < frame->height; i++) {
            uint8_t y[frame->width];
            memcpy(y, frame->data[0] + i * frame->linesize[0], frame->width);
            yuv_data.insert(yuv_data.end(), y, y + frame->width);
        }
        // U
        for(int i = 0; i < frame->height / 2; i++) {
            uint8_t u[frame->width / 2];
            memcpy(u, frame->data[1] + i * frame->linesize[1], frame->width / 2);
            yuv_data.insert(yuv_data.end(), u, u + frame->width / 2);
        }
        // V
        for(int i = 0; i < frame->height / 2; i++) {
            uint8_t v[frame->width / 2];
            memcpy(v, frame->data[2] + i * frame->linesize[2], frame->width / 2);
            yuv_data.insert(yuv_data.end(), v, v + frame->width / 2);
        }
        
        cv::Mat yuv(frame->height + frame->height / 2, frame->width, CV_8UC1, yuv_data.data());
        cv::Mat bgr;
        cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
        cv::Mat out = bgr.clone(); // clone because buf will be freed

        return out;
    }

    void DecodeImage(const CompressedVideo& video_msg, cv::Mat& image) {
        size_t data_size = video_msg.data.size();
        std::vector<uint8_t> data(data_size);
        memcpy(data.data(), video_msg.data.data(), data_size);
        for(int i = 0; i < 8; ++i) {
            printf("%02x ", data[i]);
        }
        printf("\n");
        printf("Data size: %zu\n", data_size);
        printf("NALU type: %d\n", (data[4] & 0x7E) >> 1); // H265 NALU type

        uint8_t out_data[data_size];
        uint8_t* out_data_ptr = &out_data[0];
        int out_size = 0;

        while(data.size() > 0) {
            /**
             * @brief Parse a Packet
             *
             * @param s parser context
             * @param avctx codec context
             * @param poutbuf 解析后得到的数据，或者nullptr(如果还没有解析出完整的一帧)
             * @param poutbuf_size 解析后数据的大小，如果为0，表示还没有解析出完整的一帧
             * @param buf 输入数据缓冲区
             * @param buf_size 输入数据缓冲区大小
             * @param pts 输入数据的PTS
             * @param dts 输入数据的DTS
             * @param pos 输入数据在文件中的位置
             * 
             * @return 返回使用了input数据流中的字节数，如果返回值等于buf_size，表示所有数据都被使用了。
             */
            int len = av_parser_parse2(
                parser, codec_context,
                &out_data_ptr, &out_size,
                data.data(), static_cast<int>(data.size()),
                AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
            if(len < 0) {
                std::cerr << "av_parser_parse2 error" << std::endl;
                break;
            } else {
                std::cout << "Parsed " << len << " bytes, output size: " << out_size << std::endl;
            }
            data.erase(data.begin(), data.begin() + len);
            if(out_size > 0) {
                if(av_new_packet(packet, out_size) < 0) {
                    std::cerr << "av_new_packet failed" << std::endl;
                    break;
                }
                memcpy(packet->data, out_data_ptr, out_size);
                packet->size = out_size;

                int ret = avcodec_send_packet(codec_context, packet);
                av_packet_unref(packet);
                if(ret < 0) {
                    std::cerr << "Could not send packet to decoder." << std::endl;
                    break;
                } else {
                    while(true) {
                        ret = avcodec_receive_frame(codec_context, frame);
                        if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                            break;
                        } else if(ret < 0) {
                            std::cerr << "Could not receive frame from decoder." << std::endl;
                            break;
                        }
                        // 成功得到一帧 -> 转 BGR 并 push_back
                        image_count_++;
                        cv::Mat bgr = avframe_to_bgr_mat(frame);
                        std::string output_path = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/ros2_laneline/output";
                        std::string image_name = "/frame_" + std::to_string(image_count_) + ".jpg";
                        std::string image_path = output_path + image_name;
                        cv::imwrite(image_path, bgr);
                        if(!bgr.empty()) {
                            image = bgr.clone();
                        }
                        av_frame_unref(frame);
                    }
                }
            }
        }
    }

    void Print(const CompressedVideo& video_msg, const LaneArrayV2& lane_msg, bool sync_lane) {
        cv::Mat image = cv::Mat::ones(2160, 3840, CV_8UC3);

        DecodeImage(video_msg, image);

        if(sync_lane) {
            for(auto lane : lane_msg.lane_array) {
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
        }

        count_++;
        std::string frame_num = "frame " + std::to_string(count_);
        cv::putText(image, frame_num, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        
        video_writer_.write(image);
    }

    bool SyncLane(double match_time, LaneArrayV2& lane_msg) {
        std::lock_guard<std::mutex> lock2(lane_mutex_);
        if(lane_queue_.empty()) {
            std::cerr << "Lane queue is empty." << std::endl;
            return false;
        }

        std::deque<LaneArrayV2>::iterator iter = std::upper_bound(
            lane_queue_.begin(), lane_queue_.end(), match_time,
            [](double timestamp, const LaneArrayV2& lane_msg) {
                double lane_timestamp = lane_msg.header.stamp.sec + lane_msg.header.stamp.nanosec * 1e-9;
                return timestamp < lane_timestamp;
            });

        if(iter == lane_queue_.end()) {
            std::deque<LaneArrayV2>::iterator prev_iter = std::prev(iter);
            double video_time = match_time;
            double lane_time = prev_iter->header.stamp.sec + prev_iter->header.stamp.nanosec * 1e-9;
            double time_diff = std::abs(lane_time - video_time);

            if(time_diff < 0.3) {
                lane_msg = *prev_iter;
                lane_queue_.erase(lane_queue_.begin(), std::next(prev_iter));
                return true;
            } else {
                std::cerr <<  __LINE__ << ", No matching Lane msg found. video time " 
                            << std::to_string(video_time)
                            << ", lane time " << std::to_string(lane_time)
                            << std::endl;
                return false;
            }
        }

        if(iter == lane_queue_.begin()) {
            std::cerr << "No matching Lane msg found. " << __LINE__ << std::endl;
            return false;
        }

        if(lane_queue_.size() < 2) {
            std::cerr << "Not enough Lane msgs in queue." << std::endl;
            return false;
        }

        std::deque<LaneArrayV2>::iterator prev_iter = std::prev(iter);
        double video_time = match_time;
        double lane_time = prev_iter->header.stamp.sec + prev_iter->header.stamp.nanosec * 1e-9;
        double time_diff = std::abs(lane_time - video_time);

        if(time_diff < 0.3) {
            lane_msg = *prev_iter;
            lane_queue_.erase(lane_queue_.begin(), std::next(prev_iter));
            return true;
        } else {
                std::cerr <<  __LINE__ << ", No matching Lane msg found. video time " 
                            << std::to_string(video_time)
                            << ", lane time " << std::to_string(lane_time)
                            << std::endl;
            return false;
        }

        return false;
    }

    void Sync() {
        while (running_.load()) {
            LaneArrayV2 lane_msg;
            bool lane_available = false;
            CompressedVideo video_msg;
            bool video_available = false;

            {
                bool got_video = video_msg_queue_.dequeue(video_msg, std::chrono::milliseconds(100));
                if(!got_video) {
                    continue;
                }
                video_available = true;
            }
            
            {
                lane_available = SyncLane(
                    video_msg.header.stamp.sec + video_msg.header.stamp.nanosec * 1e-9, lane_msg); 
            }


            if(video_available) {
                double video_time = video_msg.header.stamp.sec + video_msg.header.stamp.nanosec * 1e-9;
                double lane_time = lane_msg.header.stamp.sec + lane_msg.header.stamp.nanosec * 1e-9;
                std::cerr << "-------Synchronized msgs at " << count_
                          << " num, time: video " 
                          << std::to_string(video_time)
                          << ", lane " << std::to_string(lane_time)
                          << std::endl;
                Print(video_msg, lane_msg, lane_available);
            }
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
      std::array<double, 3> translation_rfu2camera;
      std::array<double, 4> rotation_rfu2camera;
      translation_rfu2camera[0] = 0.02120796734209838;
      translation_rfu2camera[1] = 1.5566029441557836;
      translation_rfu2camera[2] = -2.0829265465698885;
      rotation_rfu2camera[0] = 0.7044463824383106;
      rotation_rfu2camera[1] = 0.7097165737575503;
      rotation_rfu2camera[2] = -0.006408297327490251;
      rotation_rfu2camera[3] = -0.004075896071251731;
      Eigen::Vector3d translation_eigen_rfu2camera(translation_rfu2camera[0], translation_rfu2camera[1], translation_rfu2camera[2]);
      Eigen::Translation3d translation_matrix_rfu2camera(translation_eigen_rfu2camera);
      Eigen::Quaterniond rotation_eigen_rfu2camera(rotation_rfu2camera[0], rotation_rfu2camera[1], rotation_rfu2camera[2], rotation_rfu2camera[3]);
      Eigen::Affine3d rfu2camera_affine = translation_matrix_rfu2camera * rotation_eigen_rfu2camera;
      Eigen::Matrix4d rfu2camera_matrix4d = rfu2camera_affine.matrix();
      rfu2camera_matrix4d_ = rfu2camera_matrix4d;
      std::cout << "rfu2camera_matrix4d: " << std::endl << rfu2camera_matrix4d << std::endl;

      // camera2pixel
      double fx = 7314.1068504287596;
      double fy = 7310.9106790563401;
      double cx = 1912.5121782628;
      double cy = 1066.95209113283;
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
    std::uint64_t count_ = 0;
    std::uint64_t image_count_ = 0;
    std::string video_file_ = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/ros2_laneline/output/output_just_laneline.avi";
    std::string calibration_path_ = "/mnt/workspace/cgz_workspace/Exercise/eigen_example/calibration";

    Eigen::Matrix4d ego2rfu_matrix4d_;
    Eigen::Matrix4d rfu2camera_matrix4d_;
    Eigen::Matrix3d camera2pixel_matrix3d_;

    cv::VideoWriter video_writer_;

    rclcpp::Subscription<LaneArrayV2>::SharedPtr lane_sub_;
    rclcpp::Subscription<CompressedVideo>::SharedPtr video_sub_;
    std::deque<LaneArrayV2> lane_queue_;
    std::mutex lane_mutex_;
    std::deque<CompressedVideo> video_queue_;
    std::mutex video_mutex_;
    std::uint16_t max_queue_size_ = 500;

    BoundedBlockingQueue<CompressedVideo> video_msg_queue_{2};

    std::atomic<bool> running_{false};
    std::thread sync_thread_;

    AVFormatContext *format_context = nullptr;
    AVCodecContext *codec_context = nullptr;
    AVCodec* decoder = nullptr;
    AVPacket* packet = nullptr;
    AVFrame* frame = nullptr;
    AVCodecParserContext* parser = nullptr;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneLineNode>();
    bool ret = node->Init();
    if(!ret) {
        std::cerr << "Failed to initialize node." << std::endl;
        return -1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}