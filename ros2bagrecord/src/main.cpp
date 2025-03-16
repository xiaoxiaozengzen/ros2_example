#include <string>
#include <iostream>
#include <chrono>
#include <thread>

#include "ros2bagrecord/BagRecord.hpp"

int main(int argc, char** argv) {
    std::string path = argv[1];
    // std::string path = "/home/user/cgz_workspace/dag/geely1/20221123/ppl_bag_20221123_082344_0.db3";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BagRecord>(path));
    rclcpp::shutdown();
    return 0;
}