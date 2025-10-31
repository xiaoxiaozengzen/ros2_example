#include <iostream>
#include <string>
#include <fstream>
#include <optional>
#include <unordered_map>

#include <mcap/reader.hpp>

int main(int argc, char** argv) {
    if(argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <mcap_file>" << std::endl;
        return -1;
    }
    std::string mcap_file = argv[1];

    std::ifstream file_stream(mcap_file, std::ios::binary);
    if(!file_stream.is_open()) {
        std::cerr << "Failed to open file: " << mcap_file << std::endl;
        return -1;
    }

    mcap::McapReader reader;
    mcap::Status status = reader.open(file_stream);
    if(status.code != mcap::StatusCode::Success) {
        std::cerr << "Failed to open MCAP file: " << status.message << std::endl;
        return -1;
    } else {
        std::cout << "Successfully opened MCAP file with " << status.message << std::endl;
    }

    // 必须读取summary才能获取header/footer/statistics等信息
    status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
    if(status.code != mcap::StatusCode::Success) {
        std::cerr << "Failed to read MCAP summary: " << status.message << std::endl;
        return -1;
    } else {
        std::cout << "Successfully read MCAP summary: " << status.message << std::endl;
    }

    // header information
    std::optional<mcap::Header> header = reader.header();
    if(header) {
        std::cout << "MCAP Header:" << std::endl;
        std::cout << "  Profile: " << header->profile << std::endl;
        std::cout << "  Library: " << header->library << std::endl;
    } else {
        std::cout << "No MCAP header found." << std::endl;
    }

    // footer information
    std::optional<mcap::Footer> footer = reader.footer();
    if(footer) {
        std::cout << "MCAP Footer:" << std::endl;
        std::cout << "  SummaryStart: " << footer->summaryStart << std::endl;
        std::cout << "  SummaryOffsetStart: " << footer->summaryOffsetStart << std::endl;
        std::cout << "  SummaryCrc: " << footer->summaryCrc << std::endl;
    } else {
        std::cout << "No MCAP footer found." << std::endl;
    }

    // statistics information
    std::optional<mcap::Statistics> statistics = reader.statistics();
    if(statistics) {
        std::cout << "MCAP Statistics:" << std::endl;
        std::cout << "  MessageCount: " << statistics->messageCount << std::endl;
        std::cout << "  SchemaCount: " << statistics->schemaCount << std::endl;
        std::cout << "  ChannelCount: " << statistics->channelCount << std::endl;
        std::cout << "  AttachmentCount: " << statistics->attachmentCount << std::endl;
        std::cout << "  MetadataCount: " << statistics->metadataCount << std::endl;
        std::cout << "  ChunkCount: " << statistics->chunkCount << std::endl;
        std::cout << "  messageStartTime: " << statistics->messageStartTime << ", seconds: " << std::to_string(statistics->messageStartTime / 1e9) << std::endl;
        std::cout << "  messageEndTime: " << statistics->messageEndTime << ", seconds: " << std::to_string(statistics->messageEndTime / 1e9) << std::endl;
        std::cout << "  channelMessageCounts:" << std::endl;
        for(const auto& value : statistics->channelMessageCounts) {
            std::cout << "      Channel ID: " << value.first << ", Message Count: " << value.second << std::endl;
        }
    } else {
        std::cout << "No MCAP statistics found." << std::endl;
    }
    
    // channels information
    std::unordered_map<mcap::ChannelId, mcap::ChannelPtr> channels = reader.channels();
    std::cout << "MCAP Channels:" << std::endl;
    for (const auto& value : channels) {
        std::cout << "Channel ID: " << value.first << std::endl;
        std::cout << "  id: " << value.second->id << std::endl;
        std::cout << "  topic: " << value.second->topic << std::endl;
        std::cout << "  messageEncoding: " << value.second->messageEncoding << std::endl;
        std::cout << "  schemaId: " << value.second->schemaId << std::endl;
        std::cout << "  metadata: " << std::endl;
        for (const auto& meta : value.second->metadata) {
            std::cout << "      key: " << meta.first << ", value: " << meta.second << std::endl;
        }
    }

    // schemas information
    std::unordered_map<mcap::SchemaId, mcap::SchemaPtr> schemas = reader.schemas();
    std::cout << "MCAP Schemas:" << std::endl;
    for (const auto& value : schemas) {
        std::cout << "Schema ID: " << value.first << std::endl;
        std::cout << "  id: " << value.second->id << std::endl;
        std::cout << "  name: " << value.second->name << std::endl;
        std::cout << "  encoding: " << value.second->encoding << std::endl;
        std::cout << "  data size: " << value.second->data.size() << std::endl;
        std::vector<char> schema_data_vec(value.second->data.size());
        for(std::size_t i = 0; i < value.second->data.size(); ++i) {
            schema_data_vec[i] = static_cast<char>(std::to_integer<uint8_t>(value.second->data[i]));
        }
        std::string schema_data_str(schema_data_vec.data(), schema_data_vec.size());
        std::cout << "  data(string): " << schema_data_str << std::endl;
    }

    // 解析消息
    std::string target_topic = "/sensor/cam_front_120/h265";
    mcap::ChannelId target_channel_id = 0;
    mcap::SchemaId target_schema_id = 0;
    for(const auto& value : channels) {
        if(value.second->topic == target_topic) {
            target_channel_id = value.first;
            target_schema_id = value.second->schemaId;
            break;
        }
    }
    if(target_channel_id == 0) {
        std::cerr << "Target topic not found: " << target_topic << std::endl;
        return -1;
    }

    mcap::LinearMessageView message_view = reader.readMessages();
    mcap::LinearMessageView::Iterator it = message_view.begin();
    mcap::LinearMessageView::Iterator end = message_view.end();

    std::uint64_t message_count = 0;
    for(int i = 0; it != end; ++it, ++i) {
        const mcap::MessageView& msg_view = *it;
        if(msg_view.channel->id != target_channel_id) {
            continue;
        }

        message_count++;
        std::cout << " =========== Message " << message_count << " ====" << std::endl;

        std::cout << " channelId: " << msg_view.message.channelId << std::endl;
        std::cout << " sequence: " << msg_view.message.sequence << std::endl;
        std::cout << " logTime: " << msg_view.message.logTime << ", seconds: " << std::to_string(msg_view.message.logTime / 1e9) << std::endl;
        std::cout << " publishTime: " << msg_view.message.publishTime << ", seconds: " << std::to_string(msg_view.message.publishTime / 1e9) << std::endl;
        std::cout << " data size: " << msg_view.message.dataSize << std::endl;
        std::cout << " data ptr: " << static_cast<const void*>(msg_view.message.data) << std::endl;

        if(message_count == 1) {
            // 当前mcap只是解析得到序列化好的ros2msg消息。具体的反序列化需要使用ros2的接口来完成，mcap本身并不支持反序列化
        }

    }

    reader.close();

    return 0;
}