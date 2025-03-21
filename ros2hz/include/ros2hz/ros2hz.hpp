#pragma once

#include <chrono>
#include <deque>
#include <iostream>
#include <memory>
#include <thread>
#include <toml.hpp>
#include <unordered_map>

#include "ros2hz/generic_subscription.hpp"
#include "rcl/expand_topic_name.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/shared_library.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/types.h"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#ifdef WITH_REDIS
#include <hiredis/hiredis.h>
#endif

namespace devastator {
namespace topichz {

void example();

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

class TopicHz : public rclcpp::Node {
public:
  TopicHz(const std::vector<std::string> topics)
      : Node("topichz"), topics_(topics) {
#ifdef WITH_REDIS
    c_ = redisConnect("192.168.2.102", 6379);
    if (c_ == NULL || c_->err) {
      if (c_) {
        std::cerr << "Error: " << c_->errstr << std::endl;
        redisFree(c_);
        // handle error
      } else {
        std::cerr << "Can't allocate redis context\n" << std::endl;
      }
    }
#endif
    auto discovery = std::bind(&TopicHz::topics_discovery, this,
                               std::chrono::milliseconds(1000), topics, false);
    std::thread(discovery).detach();
  }
  ~TopicHz() {
#ifdef WITH_REDIS
    if (c_) {
      redisFree(c_);
    }
#endif
  }

  std::unordered_map<std::string, std::string> get_missing_topics(
      const std::unordered_map<std::string, std::string> &all_topics) {
    std::unordered_map<std::string, std::string> missing_topics;
    for (const auto &i : all_topics) {
      if (subscriptions_.find(i.first) == subscriptions_.end()) {
        missing_topics.emplace(i.first, i.second);
      }
    }
    return missing_topics;
  }

  void topics_discovery(std::chrono::milliseconds topic_polling_interval,
                        const std::vector<std::string> &requested_topics,
                        bool include_hidden_topics) {
    while (rclcpp::ok()) {
      auto topics_to_subscribe = this->get_topics_with_types(requested_topics);
      auto missing_topics = get_missing_topics(topics_to_subscribe);

      for (const auto &topic : missing_topics) {
        const std::string &topic_type = topic.second;
        std::cout << "create subscription " << topic.first << " " << topic_type
                  << std::endl;
        auto subscription = this->create_generic_subscription(
            topic.first, topic_type, rclcpp::QoS(10),
            [this,
             topic](const std::shared_ptr<rclcpp::SerializedMessage> message) {
              float hz = this->stats_[topic.first].update();
#ifdef WITH_REDIS
              redisReply *reply = static_cast<redisReply *>(
                  redisCommand(c_, "SETEX %s 3 %f", topic.first.c_str(), hz));
              if (reply == nullptr) {
                if (c_) {
                  std::cerr << "Error: " << c_->errstr << std::endl;
                  redisFree(c_);
                } else {
                  std::cerr << "null returned" << std::endl;
                }
                // reconnect
                c_ = redisConnect("192.168.2.102", 6379);
              } else {
                freeReplyObject(reply);
              }
#endif
              //std::cout << topic.first << " " << hz << std::endl;
            });
        if (subscription) {
          subscriptions_[topic.first] = subscription;
          stats_[topic.first] = Stat();
        }
      }

      if (!requested_topics.empty() &&
          subscriptions_.size() == requested_topics.size()) {
        std::cout
            << "All requested topics are subscribed. Stopping discovery..."
            << std::endl;
        return;
      }
      std::this_thread::sleep_for(topic_polling_interval);
    }
  }

  std::shared_ptr<GenericSubscription> create_generic_subscription(
      const std::string &topic, const std::string &type, const rclcpp::QoS &qos,
      std::function<void(const std::shared_ptr<rclcpp::SerializedMessage>)>
          callback) {
    std::shared_ptr<rcpputils::SharedLibrary> library_generic_subscriptor_ =
        rosbag2_cpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
    auto type_support = rosbag2_cpp::get_typesupport_handle(
        type, "rosidl_typesupport_cpp", library_generic_subscriptor_);
    auto subscription = std::shared_ptr<GenericSubscription>();

    try {
      subscription = std::make_shared<GenericSubscription>(
          get_node_base_interface().get(), *type_support, topic, qos, callback);

      get_node_topics_interface()->add_subscription(subscription, nullptr);
    } catch (const std::runtime_error &ex) {
      std::cout << "error" << std::endl;
      // ROSBAG2_TRANSPORT_LOG_ERROR_STREAM(
      //  "Error subscribing to topic '" << topic << "'. Error: " << ex.what());
    }
    return subscription;
  }

  std::shared_ptr<rcutils_string_map_t> get_initialized_string_map() {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto substitutions_map = new rcutils_string_map_t;
    *substitutions_map = rcutils_get_zero_initialized_string_map();
    rcutils_ret_t map_init =
        rcutils_string_map_init(substitutions_map, 0, allocator);
    if (map_init != RCUTILS_RET_OK) {
      std::cerr << "Failed to initialize string map within rcutils."
                << std::endl;
      return std::shared_ptr<rcutils_string_map_t>();
    }
    return std::shared_ptr<rcutils_string_map_t>(
        substitutions_map, [](rcutils_string_map_t *map) {
          rcl_ret_t cleanup = rcutils_string_map_fini(map);
          delete map;
          if (cleanup != RCL_RET_OK) {
            std::cerr << "Failed to deallocate string map when expanding topic."
                      << std::endl;
          }
        });
  }

  std::string expand_topic_name(const std::string &topic_name) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    auto substitutions_map = get_initialized_string_map();
    if (!substitutions_map) {
      std::cerr << "Failed to initialize string map within rcutils."
                << std::endl;
      return "";
    }
    rcl_ret_t ret =
        rcl_get_default_topic_name_substitutions(substitutions_map.get());
    if (ret != RCL_RET_OK) {
      std::cerr << "Failed to initialize map with default values." << std::endl;
      return "";
    }
    char *expanded_topic_name = nullptr;
    ret = rcl_expand_topic_name(topic_name.c_str(), get_name(), get_namespace(),
                                substitutions_map.get(), allocator,
                                &expanded_topic_name);

    if (ret != RCL_RET_OK) {
      std::cerr << "Failed to expand topic name " << topic_name
                << " with error: " << rcutils_get_error_string().str;
      return "";
    }
    std::string expanded_topic_name_std(expanded_topic_name);
    allocator.deallocate(expanded_topic_name, allocator.state);
    return expanded_topic_name_std;
  }

  std::unordered_map<std::string, std::string>
  get_topics_with_types(const std::vector<std::string> &topic_names) {
    std::vector<std::string> sanitized_topic_names;
    for (const auto &topic_name : topic_names) {
      auto sanitized_topic_name = expand_topic_name(topic_name);
      if (!sanitized_topic_name.empty()) {
        sanitized_topic_names.push_back(sanitized_topic_name);
      }
    }

    auto topics_and_types = this->get_topic_names_and_types();
    std::map<std::string, std::vector<std::string>> filtered_topics_and_types;
    for (const auto &topic_and_type : topics_and_types) {
      if (std::find(sanitized_topic_names.begin(), sanitized_topic_names.end(),
                    topic_and_type.first) != sanitized_topic_names.end()) {
        filtered_topics_and_types.insert(topic_and_type);
      }
    }

    return filter_topics_with_more_than_one_type(filtered_topics_and_types,
                                                 true);
  }

  std::unordered_map<std::string, std::string>
  get_all_topics_with_types(bool include_hidden_topics) {
    return filter_topics_with_more_than_one_type(
        this->get_topic_names_and_types(), include_hidden_topics);
  }

  std::unordered_map<std::string, std::string>
  filter_topics_with_more_than_one_type(
      const std::map<std::string, std::vector<std::string>> &topics_and_types,
      bool include_hidden_topics) {
    std::unordered_map<std::string, std::string> filtered_topics_and_types;
    for (const auto &topic_and_type : topics_and_types) {
      if (topic_and_type.second.size() > 1) {
        std::cerr << "Topic '" << topic_and_type.first
                  << "' has several types associated. Only topics with one "
                     "type are supported";
        continue;
      }

      // According to rclpy's implementation, the indicator for a hidden topic
      // is a leading '_'
      // https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/topic_or_service_is_hidden.py#L15
      if (!include_hidden_topics) {
        auto tokens =
            rcpputils::split(topic_and_type.first, '/', true); // skip empty
        auto is_hidden = std::find_if(
            tokens.begin(), tokens.end(),
            [](const auto &token) -> bool { return token[0] == '_'; });
        if (is_hidden != tokens.end()) {
          RCLCPP_WARN_ONCE(rclcpp::get_logger("rosbag2_transport"),
                           "Hidden topics are not recorded. Enable them with "
                           "--include-hidden-topics");
          continue;
        }
      }

      filtered_topics_and_types.insert(
          {topic_and_type.first, topic_and_type.second[0]});
    }
    return filtered_topics_and_types;
  }

private:
  std::vector<std::string> topics_;
  std::unordered_map<std::string, std::shared_ptr<GenericSubscription>>
      subscriptions_;
  std::unordered_map<std::string, Stat> stats_;
#ifdef WITH_REDIS
  redisContext *c_ = nullptr;
#endif
};
} // namespace topichz
} // namespace devastator
