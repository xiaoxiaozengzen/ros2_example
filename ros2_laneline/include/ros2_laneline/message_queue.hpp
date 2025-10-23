#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>

template <typename T>
class BoundedBlockingQueue {
 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable cv_not_full_;
  std::condition_variable cv_not_empty_;
  std::size_t capacity_;
  bool closed_;

  bool can_enqueue() const { return queue_.size() < capacity_ && !closed_; }

  bool can_dequeue() const { return !queue_.empty() && !closed_; }

 public:
  BoundedBlockingQueue(std::size_t capacity = 4) : capacity_(capacity), closed_(false) {}

  ~BoundedBlockingQueue() { close(); }

  void set_capacity(std::size_t capacity) { capacity_ = capacity; }

  bool try_enqueue(const T& item) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (can_enqueue()) {
      queue_.push(item);
      cv_not_empty_.notify_one();
      return true;
    }
    return false;
  }

  bool try_enqueue(T&& item) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!can_enqueue() || closed_) {
      return false;
    }

    queue_.push(std::move(item));

    cv_not_empty_.notify_one();
    return true;
  }

  void enqueue(const T& item) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_not_full_.wait(lock, [this] { return can_enqueue(); });

    if (closed_) {
      return;
    }

    queue_.push(item);
    cv_not_empty_.notify_one();
  }

  bool enqueue(T&& item, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!cv_not_full_.wait_for(lock, timeout, [this] { return can_enqueue(); })) {
      return false;
    }

    if (closed_) {
      return false;
    }

    queue_.push(std::move(item));  // 移动语义操作
    cv_not_empty_.notify_one();
    return true;
  }

  bool enqueue_force(T&& item) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!can_enqueue() && !queue_.empty()) {
      queue_.pop();
    }

    if (closed_) {
      return false;
    }
    queue_.push(std::move(item));  // 移动语义操作

    cv_not_empty_.notify_one();
    return true;
  }

  void enqueue_force(const T& item) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.size() == capacity_) {
      queue_.pop();
    }

    queue_.push(item);
    cv_not_empty_.notify_one();
  }

  bool try_dequeue(T& item) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!can_dequeue()) {
      return false;
    }

    item = std::move(queue_.front());  // 移动元素到传入的item参数
    queue_.pop();

    cv_not_full_.notify_one();
    return true;
  }

  bool dequeue(T& item, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!cv_not_empty_.wait_for(lock, timeout, [this] { return can_dequeue(); })) {
      return false;
    }

    if (closed_) {
      return false;
    }

    item = std::move(queue_.front());  // 移动元素到传入的item参数
    queue_.pop();

    cv_not_full_.notify_one();
    return true;
  }

  bool empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  std::size_t size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  void close() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    cv_not_full_.notify_all();
    cv_not_empty_.notify_all();
  }
};
