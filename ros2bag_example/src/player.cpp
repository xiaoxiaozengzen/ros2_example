// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <example_interfaces/msg/int32.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include "moodycamel/readerwriterqueue.h"
#include "rclcpp/qos.hpp"
#include "rosbag2_transport/play_options.hpp"
// #include "replayable_message.hpp"

#include <chrono>
#include <memory>
#include <fstream>
#include <deque>
#include <thread>
#include <future>
#include <unordered_map>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "e171_msgs/msg/e171.hpp"

class Stat {
public:
  float update() {
    auto time = std::chrono::steady_clock::now();
    float diff = std::chrono::duration<float>(time - last_).count();
    last_ = time;
    q_.push_back(diff);
    float del = 0;
    if (q_.size() > window_size_) {
      del = q_.front();
      q_.pop_front();
    }
    sum_ = sum_ + diff - del;
    return 1 / sum_ * q_.size();
  }

private:
  size_t window_size_ = 50;
  float sum_ = 0;
  std::deque<float> q_;
  std::chrono::steady_clock::time_point last_ =
      std::chrono::steady_clock::now();
};

struct PlayOptions
{
public:
  size_t read_ahead_queue_size;
  std::string node_prefix = "";
  float rate = 1.0;

  // Topic names to whitelist when playing a bag.
  // Only messages matching these specified topics will be played.
  // If list is empty, the filter is ignored and all messages are played.
  std::vector<std::string> topics_to_filter = {};

  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides = {};
  bool loop = false;
  std::vector<std::string> topic_remapping_options = {};
};

struct ReplayableMessage
{
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
  std::chrono::nanoseconds time_since_start;
};


class Player : public rclcpp::Node
{
public:
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
public:
    explicit Player()
    :Node("player")
    {
        std::string path = "/home/user/cgz_workspace/Bag/rosbag2_2024_01_28-15_56_33";
        std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader;
        reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
        const rosbag2_cpp::StorageOptions storage_options({path, "mcap"});
        const rosbag2_cpp::ConverterOptions converter_options(
        {rmw_get_serialization_format(),
            rmw_get_serialization_format()});

        reader_ = std::make_shared<rosbag2_cpp::Reader>(std::move(reader));
        reader_->open(storage_options, converter_options);

        PlayOptions options;
        options.read_ahead_queue_size = 100;
        options.topics_to_filter.push_back("/front_4d_radar_data");
        play_th_ = std::thread(&Player::play, this, options);
    }

    ~Player()
    {
        if (play_th_.joinable()) {
            play_th_.join();
        }
    }

    void play(const PlayOptions & options)
    {
        while(!print_message_callback_) {
            std::cerr << "print_message_callback_ is null" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        // topic_qos_profile_overrides_ = options.topic_qos_profile_overrides;
        prepare_publishers(options);

        storage_loading_future_ = std::async(
            std::launch::async,
            [this, options]() {load_storage_content(options);});

        wait_for_filled_queue(options);

        play_messages_from_queue(options);
    }

    void load_storage_content(const PlayOptions & options)
    {
        TimePoint time_first_message;

        ReplayableMessage message;
        if (reader_->has_next()) {
            message.message = reader_->read_next();
            message.time_since_start = std::chrono::nanoseconds(0);
            time_first_message = TimePoint(std::chrono::nanoseconds(message.message->time_stamp));
            message_queue_.enqueue(message);
        }

        auto queue_lower_boundary =
            static_cast<size_t>(options.read_ahead_queue_size * read_ahead_lower_bound_percentage_);
        auto queue_upper_boundary = options.read_ahead_queue_size;

        while (reader_->has_next() && rclcpp::ok()) {
            if (message_queue_.size_approx() < queue_lower_boundary) {
                enqueue_up_to_boundary(time_first_message, queue_upper_boundary);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    bool is_storage_completely_loaded() const
    {
        if (storage_loading_future_.valid() &&
            storage_loading_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            storage_loading_future_.get();
        }
        return !storage_loading_future_.valid();
    }
    void enqueue_up_to_boundary(const TimePoint & time_first_message, uint64_t boundary)
    {
        ReplayableMessage message;
        for (size_t i = message_queue_.size_approx(); i < boundary; i++) {
            if (!reader_->has_next()) {
                break;
            }
            message.message = reader_->read_next();
            message.time_since_start =
            TimePoint(std::chrono::nanoseconds(message.message->time_stamp)) - time_first_message;

            message_queue_.enqueue(message);
        }
    }
    void wait_for_filled_queue(const PlayOptions & options) const
    {
        while (
            message_queue_.size_approx() < options.read_ahead_queue_size &&
            !is_storage_completely_loaded() && rclcpp::ok())
        {
            std::this_thread::sleep_for(queue_read_wait_period_);
        }
    }
    void play_messages_from_queue(const PlayOptions & options)
    {
        start_time_ = std::chrono::system_clock::now();
        do {
            play_messages_until_queue_empty(options);
            if (!is_storage_completely_loaded() && rclcpp::ok()) {
                std::cerr << 
                    "Message queue starved. Messages will be delayed. Consider "
                    "increasing the --read-ahead-queue-size option." << std::endl;
            }
        } while (!is_storage_completely_loaded() && rclcpp::ok());
    }
    void play_messages_until_queue_empty(const PlayOptions & options)
    {
        ReplayableMessage message;

        float rate = 1.0;
        // Use rate if in valid range
        if (options.rate > 0.0) {
            rate = options.rate;
        }

        while (message_queue_.try_dequeue(message) && rclcpp::ok()) {
            std::this_thread::sleep_until(
            start_time_ + std::chrono::duration_cast<std::chrono::nanoseconds>(
                1.0 / rate * message.time_since_start));
            if (rclcpp::ok()) {
                print_message_callback_(message.message);
            }
        }
    }
    void prepare_publishers(const PlayOptions & options)
    {
        rosbag2_storage::StorageFilter storage_filter;
        storage_filter.topics = options.topics_to_filter;
        reader_->set_filter(storage_filter);
    }
    void register_call(const std::function<void(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& message)> & callback)
    {
        std::cout << "register_call" << std::endl;
        print_message_callback_ = std::move(callback);
    }
  

private:
  static constexpr double read_ahead_lower_bound_percentage_ = 0.9;
  static const std::chrono::milliseconds queue_read_wait_period_;

  std::shared_ptr<rosbag2_cpp::Reader> reader_;
  moodycamel::ReaderWriterQueue<ReplayableMessage> message_queue_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  mutable std::future<void> storage_loading_future_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;

  std::function<void(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& message)> print_message_callback_;
  std::thread play_th_;
};

const std::chrono::milliseconds Player::queue_read_wait_period_ = std::chrono::milliseconds(100);



int main(int argc, char * argv[])
{
    Stat radar_stat;

    auto fun = [&radar_stat](const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& message) {
        std::cout << "radar_frequency: " << radar_stat.update() << ", buffer_length: " << message->serialized_data->buffer_length << std::endl;
    };

    rclcpp::init(argc, argv);
    auto player = std::make_shared<Player>();

    std::thread work(
        [&]() {
            rclcpp::spin(player);
            rclcpp::shutdown();
        }
    );

    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    
    player->register_call(fun);
    work.join();
    

    return 0;
}
