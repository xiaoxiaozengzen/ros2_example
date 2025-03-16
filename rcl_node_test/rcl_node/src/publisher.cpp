#include <iostream>
#include <chrono>
#include <thread>

#include "rcl/rcl.h"
#include "rcl/publisher.h"
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
  constexpr char name[] = "test_publisher_node";
  rcl_node_options_t node_options = rcl_node_get_default_options();
  ret = rcl_node_init(&node, name, "", &context, &node_options);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_node_init() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_node_init() failed." << std::endl;
    return 1;
  }

  // Create a publisher with rcl_publisher_init().
  const rosidl_message_type_support_t * ts =
    ROSIDL_GET_MSG_TYPE_SUPPORT(my_rcl_test_msgs, msg, MyRclMsg);
  const char * topic_name = "/test_publisher_topic";
  rcl_publisher_t publisher;
  rcl_publisher_options_t publisher_options;
  publisher = rcl_get_zero_initialized_publisher();
  publisher_options = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(
    &publisher, &node, ts, topic_name, &publisher_options);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_publisher_init() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_publisher_init() failed." << std::endl;
    return 1;
  }

  // utility
  {
    const char* node_name = rcl_node_get_name(&node);
    if(node_name) {
      std::cout << "rcl_node_get_name() succeeded." << std::endl;
      std::cout << "node_name: " << node_name << std::endl;
    } else {
      std::cerr << "rcl_node_get_name() failed." << std::endl;
    }

    const char* node_namespace = rcl_node_get_namespace(&node);
    if(node_namespace) {
      std::cout << "rcl_node_get_namespace() succeeded." << std::endl;
      std::cout << "node_namespace: " << node_namespace << std::endl;
    } else {
      std::cerr << "rcl_node_get_namespace() failed." << std::endl;
    }

    const char* logger_name = rcl_node_get_logger_name(&node);
    if(logger_name) {
      std::cout << "rcl_node_get_logger_name() succeeded." << std::endl;
      std::cout << "logger_name: " << logger_name << std::endl;
    } else {
      std::cerr << "rcl_node_get_logger_name() failed." << std::endl;
    }

    const char* fully_qualified_name = rcl_node_get_fully_qualified_name(&node);
    if(fully_qualified_name) {
      std::cout << "rcl_node_get_fully_qualified_name() succeeded." << std::endl;
      std::cout << "fully_qualified_name: " << fully_qualified_name << std::endl;
    } else {
      std::cerr << "rcl_node_get_fully_qualified_name() failed." << std::endl;
    }

    std::size_t domain_id = 0;
    ret = rcl_node_get_domain_id(&node, &domain_id);
    if(ret == RCL_RET_OK) {
      std::cout << "rcl_node_get_domain_id() succeeded." << std::endl;
      std::cout << "domain_id: " << domain_id << std::endl;
    } else {
      std::cerr << "rcl_node_get_domain_id() failed." << std::endl;
    }

    const char * topic_name = rcl_publisher_get_topic_name(&publisher);
    if(topic_name) {
      std::cout << "rcl_publisher_get_topic_name() succeeded." << std::endl;
      std::cout << "topic_name: " << topic_name << std::endl;
    } else {
      std::cerr << "rcl_publisher_get_topic_name() failed." << std::endl;
    }
  }

  int count = 10;
  while(count > 0) {
    // Create a message and publish it.
    my_rcl_test_msgs__msg__MyRclMsg msg;
    my_rcl_test_msgs__msg__MyRclMsg__init(&msg);
    msg.a = 10;
    msg.b = 20;
    msg.c = true;
    ret = rcl_publish(&publisher, &msg, nullptr);
    if(ret == RCL_RET_OK) {
      std::cout << "rcl_publish() succeeded." << std::endl;
    } else {
      std::cerr << "rcl_publish() failed." << std::endl;
      return 1;
    }
    my_rcl_test_msgs__msg__MyRclMsg__fini(&msg);
    count--;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }


  // end of the publisher
  ret = rcl_publisher_fini(&publisher, &node);
  if(ret == RCL_RET_OK) {
    std::cout << "rcl_publisher_fini() succeeded." << std::endl;
  } else {
    std::cerr << "rcl_publisher_fini() failed." << std::endl;
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