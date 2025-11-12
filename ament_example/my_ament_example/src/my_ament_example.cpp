#include "my_ament_example/my_ament_example.hpp"

#include "world/world.hpp"

void HelloWorld()
{
  Hello();
  World();
  int result = plus(3, 5);
  std::cout << "3 + 5 = " <<  result << std::endl;
}