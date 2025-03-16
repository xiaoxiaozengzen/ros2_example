#include "ros2bagparse/BagParse.hpp"

BagParse::BagParse(std::string path)
    :Node("my_test"),
    path_(path) {
        storage_options.uri = path_;
        storage_options.storage_id = "sqlite3";

        converter_options.input_serialization_format = "cdr";  // bag 包序列化类型
        converter_options.output_serialization_format = "cdr";

        filter_name = "/sensor/radar0_object";
        storage_filter.topics.push_back(filter_name);

        ReaderOpen();
    }

BagParse::~BagParse() {}

void BagParse::ReaderOpen() {
    reader.open(storage_options, converter_options);

    std::cout << "file = " << reader.get_current_file() << std::endl;
    std::cout << "uri = " << reader.get_current_uri() << std::endl;

    auto topics = reader.get_all_topics_and_types();  // 获取bag包的topic以及序列化类型
    for (auto t : topics) {
        std::cout << "meta name: " << t.name << std::endl;
        std::cout << "meta type: " << t.type << std::endl;
        std::cout << "meta serialization_format: " << t.serialization_format << std::endl;
    }

    reader.set_filter(storage_filter);

    devastator_perception_msgs::msg::RadarObjectArray radar_array_msg;
    auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
    ros_message->time_stamp = 0;
    ros_message->message = nullptr;
    ros_message->allocator = rcutils_get_default_allocator();
    ros_message->message = &radar_array_msg;

    auto library = rosbag2_cpp::get_typesupport_library(
      "devastator_perception_msgs/msg/RadarObjectArray", "rosidl_typesupport_cpp");
    
    auto type_support = rosbag2_cpp::get_typesupport_handle(
      "devastator_perception_msgs/msg/RadarObjectArray", "rosidl_typesupport_cpp", library);

    rosbag2_cpp::SerializationFormatConverterFactory factory;
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;
    cdr_deserializer_ = factory.load_deserializer("cdr");

    int count = 0;

    while (reader.has_next()) {
        // serialized data
        auto serialized_message = reader.read_next();
        std::cout << "serialized_message topic_name : " << serialized_message->topic_name << std::endl;
        std::cout << "serialized_message data : " << serialized_message->serialized_data->buffer_length << std::endl;
        // for(int i=0; i<serialized_message->serialized_data->buffer_length; i++) {
        //     std::cout << std::hex << (std::uint32_t)(*(serialized_message->serialized_data->buffer + i)) << " ";
        // }
        std::cout << std::endl;
        // deserialization and conversion to ros message
        cdr_deserializer_->deserialize(serialized_message, type_support, ros_message);

        // std::cout << "a = " << std::hex << (std::uint32_t)radar_array_msg.a  << " ,c = " << radar_array_msg.c << std::endl;


        std::cout << "length = " << radar_array_msg.objects.size() << std::endl;

        for(auto msg : radar_array_msg.objects) {
            std::cout << "id = " << static_cast<std::uint32_t>(msg.id) << std::endl;
            break;
        }

        for(auto msg : radar_array_msg.objects) {
            std::cout << "confidence = " << static_cast<std::uint32_t>(msg.prob_of_exist) << std::endl;
            break;
        }


        count ++;
    }

    std::cout << "count = " << count << std::endl;

}