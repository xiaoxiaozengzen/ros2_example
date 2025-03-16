#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

#include "concurrentqueue.h"
#include "blockingconcurrentqueue.h"

void ConcurrentTest() {
  moodycamel::ConcurrentQueue<int> q(4);
  std::cout << "Queue size: " << q.size_approx() << std::endl;
  std::cout << "Queue is lock free: " << std::boolalpha << q.is_lock_free() << std::endl;

  bool success = q.enqueue(1);
  if (success) {
    std::cout << "Enqueue 1" << std::endl;
  } else {
    std::cout << "Enqueue failed" << std::endl;
  }
  q.enqueue(2);
  q.enqueue(3);
  q.enqueue(4);
  success = q.enqueue(5);
  if (success) {
    std::cout << "Enqueue 5" << std::endl;
  } else {
    std::cout << "Enqueue failed" << std::endl;
  }
  std::cout << "Queue size: " << q.size_approx() << std::endl;

  // 接口描述是尝试将元素插入队列，如果队列已满，则返回false。但是实际表现是任然会扩大队列的容量
  success = q.try_enqueue(6);
  if (success) {
    std::cout << "Try Enqueue 6" << std::endl;
  } else {
    std::cout << "Try Enqueue failed" << std::endl;
  }
  std::cout << "Queue size: " << q.size_approx() << std::endl;

  int item;
  success = q.try_dequeue(item);
  if (success) {
    std::cout << "Try Dequeue: " << item << std::endl;
  } else {
    std::cout << "Try Dequeue failed" << std::endl;
  }
  std::cout << "Queue size: " << q.size_approx() << std::endl;
}

void BlockConcurrentTest() {
  moodycamel::BlockingConcurrentQueue<int> q;
  std::thread producer([&]() {
      for (int i = 0; i != 100; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(i % 10));
          q.enqueue(i);
      }
  });
  std::thread consumer([&]() {
      for (int i = 0; i != 100; ++i) {
          int item;
          q.wait_dequeue(item);
          std::cerr << "Dequeued: " << item  << ", i: " << i << std::endl;

          if (q.wait_dequeue_timed(item, std::chrono::milliseconds(5))) {
            ++i;
            std::cerr << "Time Dequeued: " << item  << ", i: " << i << std::endl;
          } else {
            std::cerr << "Time Dequeue failed: " << i << std::endl;
          }
      }
  });

  producer.join();
  consumer.join();
}

int main() {
  std::cout << "========== ConcurrentQueue Test ==========" << std::endl;
  ConcurrentTest();
  std::cout << "========== BlockConcurrentQueue Test ==========" << std::endl;
  BlockConcurrentTest();
  return 0;
}