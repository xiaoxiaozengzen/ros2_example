#include <unistd.h>

#include <thread>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

extern "C" {
#include "zlgcan.h"
}

class ZLGCanNode : public rclcpp::Node {

    using Int8MultiArray = std_msgs::msg::Int8MultiArray;

public:
    ZLGCanNode(std::string node_name)
    :Node(node_name) 
    {
        subscription_ = this->create_subscription<Int8MultiArray>("/canbus/rawcan", 10, std::bind(&ZLGCanNode::Callback, this, std::placeholders::_1));
    }
    
    ~ZLGCanNode() {

    }

    bool Init() {
        std::cout << "sizeof(ZCANDataObj): " << sizeof(ZCANDataObj) << std::endl;
        std::size_t date_0 = offsetof(ZCANDataObj, dataType);
        std::cout << "offsetof(ZCANDataObj, dataType): " << date_0 << std::endl;
        std::size_t date_1 = offsetof(ZCANDataObj, chnl);
        std::cout << "offsetof(ZCANDataObj, chnl): " << date_1 << std::endl;
        std::size_t date_2 = offsetof(ZCANDataObj, flag);
        std::cout << "offsetof(ZCANDataObj, flag): " << date_2 << std::endl;
        std::size_t date_3 = offsetof(ZCANDataObj, data);
        std::cout << "offsetof(ZCANDataObj, data): " << date_3 << std::endl;

        return true;
    }

    void Callback(const Int8MultiArray::SharedPtr obj) {
        static std::uint64_t msg_count = 0;
        msg_count++;
        if(msg_count > 5) {
            return; // Limit the output to 5 messages
        }

        std::cout << "================" << msg_count << "================" << std::endl;
        std::cout << "data size: " << obj->data.size() << std::endl;
        if (obj->data.size() < sizeof(ZCANDataObj)) {
            std::cerr << "Received data size is less than ZCANDataObj size." << std::endl;
            return;
        }
        std::size_t data_size = obj->data.size();
        std::size_t offest = 0;

        std::uint64_t len = 0;
        std::uint64_t timestamp = 0;
        std::uint64_t count = 0;

        memcpy(&len, obj->data.data() + offest, sizeof(std::uint64_t));
        std::cout << "len: " << len << std::endl;
        offest += sizeof(std::uint64_t);
        memcpy(&timestamp, obj->data.data() + offest, sizeof(std::uint64_t));
        std::cout << "timestamp: " << timestamp << ", sec: " << std::fixed << std::setprecision(9) << timestamp * 1e-9 << std::endl;
        offest += sizeof(std::uint64_t);
        memcpy(&count, obj->data.data() + offest, sizeof(std::uint64_t));
        std::cout << "count: " << count << std::endl;
        offest += sizeof(std::uint64_t);
        
        ZCANDataObj can_data;
        memset(&can_data, 0, sizeof(ZCANDataObj));
        memcpy(&can_data, obj->data.data() + offest, sizeof(ZCANDataObj));
        std::cout << "dataType: " << static_cast<std::uint32_t>(can_data.dataType) << std::endl;
        std::cout << "channel: " << static_cast<std::uint32_t>(can_data.chnl) << std::endl;

    }

private:
    rclcpp::Subscription<Int8MultiArray>::SharedPtr subscription_{nullptr};
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto subnode = std::make_shared<ZLGCanNode>("zlg_can_bus");
    subnode->Init();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(subnode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

