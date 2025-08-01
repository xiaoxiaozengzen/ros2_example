#include "double_lib_second/add/add.hpp"
#include "double_lib_first/add/add.hpp"

int main(int argc, char **argv) {
  std::cout << "argc: " << argc << std::endl;
  std::cout << "argv[0]: " << argv[0] << std::endl;
  
  int a = 5;
  int b = 10;

  // 使用 double_lib_first 中的 add 函数
  int result_first = double_lib_first::add(a, b);
  std::cout << "Result from double_lib_first: " << result_first << std::endl;

  // 使用 double_lib_second 中的 add 函数
  int result_second = double_lib_second::add(a, b);
  std::cout << "Result from double_lib_second: " << result_second << std::endl;

  return 0;
}