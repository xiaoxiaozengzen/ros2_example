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
#include <mutex>
#include <condition_variable>

#include "libyuv.h"

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

class Frequency {
 public:
  /**
   * @brief
   * 记录n个数量的t(两帧的时间差值)，当n=50的时候就不在增长(最久的会删除，最新的会添加到末尾)。然后算平均频率
   *
   *
   * @return double
   */
  double update() {
    auto time = std::chrono::steady_clock::now();
    double diff = std::chrono::duration<double>(time - last_).count();
    last_ = time;
    q_.push_back(diff);
    double del = 0;
    if (q_.size() > window_size_) {
      del = q_.front();
      q_.pop_front();
    }
    sum_ = sum_ + diff - del;
    return 1 / sum_ * static_cast<double>(q_.size());
  }

 private:
  size_t window_size_ = 50;
  double sum_ = 0;
  // 存储相邻两针时间差的序列
  std::deque<double> q_;
  std::chrono::steady_clock::time_point last_ = std::chrono::steady_clock::now();
};

class ImageEncoderNode : public rclcpp::Node {
  using CompressedVideo = foxglove_msgs::msg::CompressedVideo;
public:
    ImageEncoderNode() : Node("image_encoder_node") {

    }

    ~ImageEncoderNode() {
        is_running_.store(false);
        if (encoding_thread_.joinable()) {
            encoding_thread_.join();
        }

        av_packet_free(&packet);
        av_frame_free(&frame);
        avcodec_close(codec_context);
        avcodec_free_context(&codec_context);
        avformat_free_context(format_context);
    }

    bool Init() {
        video_publisher_ = this->create_publisher<CompressedVideo>("/sensor/cam_front_120/h265", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ImageEncoderNode::TimerCallback, this));

        encode = avcodec_find_encoder(AV_CODEC_ID_H265);
        if(!encode) {
            std::cerr << "Could not find H.265 encoder." << std::endl;
            return false;
        }
        codec_context = avcodec_alloc_context3(encode);
        if(!codec_context) {
            std::cerr << "Could not allocate codec context." << std::endl;
            return false;
        }
        codec_context->codec_type = AVMEDIA_TYPE_VIDEO;
        codec_context->bit_rate = 4000000; // 4Mbps
        codec_context->framerate = AVRational{20, 1};
        codec_context->gop_size = 10;
        codec_context->max_b_frames = 0;
        codec_context->profile = FF_PROFILE_HEVC_MAIN;
        codec_context->time_base = AVRational{1, 20};
        codec_context->width = 3840;
        codec_context->height = 2160;
        codec_context->pix_fmt = AV_PIX_FMT_YUV420P;
        av_opt_set(codec_context->priv_data, "preset", "superfast", 0);
        av_opt_set(codec_context->priv_data, "tune", "zerolatency", 0);

        int ret = avcodec_open2(codec_context, encode, nullptr);
        if(ret < 0) {
            std::cerr << "Could not open codec, because: " << AVERROR(ENOMEM) << std::endl;
            return false;
        }

        packet = av_packet_alloc();
        if(!packet) {
            std::cerr << "Could not allocate AVPacket." << std::endl;
            return false;
        }

        frame = av_frame_alloc();
        if(!frame) {
            std::cerr << "Could not allocate AVFrame." << std::endl;
            return false;
        }

        encoding_thread_ = std::thread(&ImageEncoderNode::PublishEncodedVideo, this);
        is_running_.store(true);

        return true;
    }

    void PublishEncodedVideo() {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待节点完全初始化
        while(is_running_.load()) {
            frame_count++;
            std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
            bool get_frame = GetAVFrame(frame, frame_count);
            if(!get_frame) {
                std::cerr << "Failed to get AVFrame for frame index: " << frame_count << std::endl;
                error_count++;
                if(error_count > 3) {
                    std::cerr << "Too many errors, stopping encoding thread." << std::endl;
                    is_running_.store(false);
                }
                continue;
            }
            std::cout << "Get AVFrame for frame index: " << frame_count << ", with frequency: " << get_image_freq_.update() << " Hz" << std::endl;

            int ret = avcodec_send_frame(codec_context, frame);
            if(ret < 0) {
                std::cerr << "Error sending frame to encoder: " << AVERROR(ENOMEM) << std::endl;
                av_frame_free(&frame);
                continue;
            }
            av_frame_unref(frame);
  
            ret = avcodec_receive_packet(codec_context, packet);
            if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                std::cout << "No packet received for frame index: " << frame_count << std::endl;
                break; // 没有更多数据包可接收
            } else if(ret < 0) {
                std::cerr << "Error receiving packet: " << AVERROR(ENOMEM) << std::endl;
                continue;
            }
            packet_count++;

            std::chrono::time_point<std::chrono::system_clock> encode_time = std::chrono::system_clock::now();
            auto encode_duration = std::chrono::duration_cast<std::chrono::milliseconds>(encode_time - start_time).count();
            std::cout << "Encoded packet count: " << packet_count << " into packet size: " << packet->size << " bytes, took " << encode_duration << " ms" << std::endl;
            
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]() { return is_frame_ready_; });
                is_frame_ready_ = false;
            }

            CompressedVideo video_msg;
            video_msg.header.stamp = this->now();
            video_msg.header.frame_id = std::to_string(frame_count);
            video_msg.format = "h265";
            video_msg.data.resize(packet->size);
            std::memcpy(video_msg.data.data(), packet->data, packet->size);

            video_publisher_->publish(video_msg);

            av_packet_unref(packet);

            std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << "Encoding and publishing frame index: " << frame_count << " took " << duration << " ms" << std::endl;
            
        }
    }

    void TimerCallback() {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!is_frame_ready_) {
            std::cout << "Timer notifying encoding thread with frequency: " << timer_freq_.update() << " Hz" << std::endl;
            is_frame_ready_ = true;
            cv_.notify_one();
        }
    }

    bool GetAVFrame(AVFrame * frame, std::uint64_t frame_index) {
        std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
        std::stringstream ss;
        ss << image_path_ << "/frame_" << std::to_string(frame_index) << ".yuv";
        std::string yuv_file_path = ss.str();

        std::ifstream yuv_file(yuv_file_path, std::ios::binary);
        if(!yuv_file.is_open()) {
            std::cerr << "Could not open YUV file: " << yuv_file_path << std::endl;
            return false;
        }

        frame->format = codec_context->pix_fmt;
        frame->width = codec_context->width;
        frame->height = codec_context->height;
        frame->pts = frame_index;

        int ret = av_frame_get_buffer(frame, 0);
        if(ret < 0) {
            std::cerr << "Could not allocate frame data buffer, because: " << AVERROR(ENOMEM) << std::endl;
            yuv_file.close();
            return false;
        }

        int y_size = frame->width * frame->height;
        int uv_size = y_size / 4;
        yuv_file.read(reinterpret_cast<char*>(frame->data[0]), y_size);
        yuv_file.read(reinterpret_cast<char*>(frame->data[1]), uv_size);
        yuv_file.read(reinterpret_cast<char*>(frame->data[2]), uv_size);
        yuv_file.close();
        std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Read frame index: " << frame_index << " from YUV file took " << duration << " ms" << std::endl;

        return true;
    }

private:
    rclcpp::Publisher<CompressedVideo>::SharedPtr video_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> is_running_{false};
    std::thread encoding_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool is_frame_ready_ = false;
    Frequency get_image_freq_;
    Frequency timer_freq_;

    std::string image_path_ = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/ros2_laneline/output_copy";

    AVFormatContext *format_context = nullptr;
    AVCodecContext *codec_context = nullptr;
    AVCodec* encode = nullptr;
    AVPacket *packet = nullptr;
    AVFrame *frame = nullptr;
    std::uint64_t frame_count = 0;
    std::uint64_t packet_count = 0;
    std::uint64_t error_count = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageEncoderNode>();
    bool ret = node->Init();
    if(!ret) {
        std::cerr << "Failed to initialize node." << std::endl;
        return -1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
