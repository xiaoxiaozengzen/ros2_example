#include "add/add.hpp"

namespace double_lib_first {

int add(int a, int b) {
    std::cout << "First Adding" << a << " and " << b << ": " << (a + b) << std::endl;
    int c = plus(a, b);
    return a + b + c;
}

}  // namespace double_lib_first