#include <iostream>
#include <chrono>
#include <thread>

#include "rcl/rcl.h"
#include "rcl/subscription.h"
#include "rcl/error_handling.h"
#include "rosidl_runtime_c/string_functions.h" 

#include "my_rcl_test_msgs/msg/my_rcl_msg.h"

int main() {
  rcl_ret_t ret;

  // Initialize rcl with rcl_init().
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_init_options_init() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_init_options_init() failed." << std::endl;
    return 1;
  }

  rcl_context_t context = rcl_get_zero_initialized_context();
  ret = rcl_init(0, nullptr, &init_options, &context);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_init() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_init() failed." << std::endl;
    return 1;
  }

  rcl_node_t node = rcl_get_zero_initialized_node();
  constexpr char name[] = "test_subscription_node";
  rcl_node_options_t node_options = rcl_node_get_default_options();
  ret = rcl_node_init(&node, name, "", &context, &node_options);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_node_init() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_node_init() failed." << std::endl;
    return 1;
  }

  // Create a subscription with rcl_subscription_init().
  const rosidl_message_type_support_t * ts =
    ROSIDL_GET_MSG_TYPE_SUPPORT(my_rcl_test_msgs, msg, MyRclMsg);
  const char * topic_name = "/test_publisher_topic";
  rcl_subscription_options_t subscription_options;
  rcl_subscription_t subscription;
  rcl_subscription_t subscription_zero_init;
  rcutils_allocator_t allocator;

  allocator = rcutils_get_default_allocator();
  subscription_options = rcl_subscription_get_default_options();
  subscription = rcl_get_zero_initialized_subscription();
  subscription_zero_init = rcl_get_zero_initialized_subscription();
  ret = rcl_subscription_init(&subscription, &node, ts, topic_name, &subscription_options);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_subscription_init() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_subscription_init() failed." << std::endl;
    return 1;
  }

  // wait_for_subscription_to_be_ready
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, allocator);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_wait_set_init() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_wait_set_init() failed." << std::endl;
    return 1;
  }
  std::size_t max_tries = 10;
  std::int64_t period_ms = 100;
  std::size_t iteration = 0;
  while(iteration < max_tries) {
    ++iteration;
    ret = rcl_wait_set_clear(&wait_set);
    if(ret == RCL_RET_OK) {
      std::cout << "rcl_wait_set_clear() succeeded." << std::endl;
    } else {
      std::cerr << "rcl_wait_set_clear() failed." << std::endl;
    }

    ret = rcl_wait_set_add_subscription(&wait_set, &subscription, NULL);
    if(ret == RCL_RET_OK) {
      std::cout << "rcl_wait_set_add_subscription() succeeded." << std::endl;
    } else {
      std::cerr << "rcl_wait_set_add_subscription() failed." << std::endl;
    }

    ret = rcl_wait(&wait_set, period_ms * 1000 * 1000);
    if(ret == RCL_RET_TIMEOUT) {
      continue;
    }
    if(ret != RCL_RET_OK) {
      std::cerr << "rcl_wait() failed." << std::endl;
      return 1;
    }
    for(std::size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
      if(wait_set.subscriptions[i] == &subscription) {
        std::cout << "Subscription is ready." << std::endl;
        break;
      }
    }
  }
  if(iteration > max_tries) {
    std::cerr << "Subscription is not ready." << std::endl;
    return 1;
  }
  
  ret = rcl_wait_set_fini(&wait_set);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_wait_set_fini() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_wait_set_fini() failed." << std::endl;
    return 1;
  }

  while(true) {
    // Create a message and publish it.
    my_rcl_test_msgs__msg__MyRclMsg msg;
    my_rcl_test_msgs__msg__MyRclMsg__init(&msg);

    rmw_message_info_t message_info = rmw_get_zero_initialized_message_info();
    ret = rcl_take(&subscription, &msg, &message_info, nullptr);
    if(ret == RCL_RET_OK) {
      std::cout << "rcl_take() succeeded." << std::endl;
      std::cout << "msg.a: " << static_cast<std::uint32_t>(msg.a) << std::endl;
      std::cout << "msg.b: " << static_cast<std::uint32_t>(msg.b) << std::endl;
      std::cout << "msg.c: " << msg.c << std::endl;
    } else {
      // std::cerr << "rcl_take() failed." << std::endl;
    }

    my_rcl_test_msgs__msg__MyRclMsg__fini(&msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }


  // end of the subscription
  ret = rcl_subscription_fini(&subscription, &node);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_subscription_fini() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_subscription_fini() failed." << std::endl;
    return 1;
  }

  // end of the program
  ret = rcl_node_fini(&node);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_node_fini() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_node_fini() failed." << std::endl;
    return 1;
  }
  ret = rcl_shutdown(&context);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_shutdown() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_shutdown() failed." << std::endl;
    return 1;
  }
  ret = rcl_context_fini(&context);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_context_fini() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_context_fini() failed." << std::endl;
    return 1;
  }
  ret = rcl_init_options_fini(&init_options);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_init_options_fini() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_init_options_fini() failed." << std::endl;
    return 1;
  }
}