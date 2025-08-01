#include "add/add.hpp"

namespace double_lib_second {

int add(int a, int b) {
    std::cout << "Second Adding " << a << " and " << b << ": " << (a + b) << std::endl;
    int c = plus(a, b);
    return a + b + c;
}

}  // namespace double_lib_second