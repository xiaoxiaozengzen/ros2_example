#include "add/add.hpp"

namespace double_lib_second {

int add(int a, int b) {
    std::cout << "Second Adding " << a << " and " << b << ": " << (a + b) << std::endl;
    return a + b;
}

}  // namespace double_lib_second