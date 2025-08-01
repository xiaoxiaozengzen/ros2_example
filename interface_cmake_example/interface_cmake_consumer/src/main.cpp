#include <iostream>
#include <thread>
#include <chrono>

#include "interface_cmake_dependcy/sub/sub.hpp"
#include "interface_cmake/add/add.hpp"
#include "interface_cmake/plus/plus.hpp"


int main() {
  int a = 10;
  int b = 20;
  int result = plus(a, b);
  std::cout << "Result of plus: " << result << std::endl;

  int sub_result = sub(a, b);
  std::cout << "Result of sub: " << sub_result << std::endl;

  int add_result = add(a, b);
  std::cout << "Result of add: " << add_result << std::endl;
}
