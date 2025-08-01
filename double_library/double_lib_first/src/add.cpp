#include "add/add.hpp"

namespace double_lib_first {

int add(int a, int b) {
    std::cout << "First Adding" << a << " and " << b << ": " << (a + b) << std::endl;
    return a + b;
}

}  // namespace double_lib_first