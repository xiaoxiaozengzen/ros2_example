import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def read_messages(args):
    with open(args.input, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])

        header = reader.get_header()
        print(f"profile: {header.profile}")
        print(f"library: {header.library}")

        s = reader.get_summary()
        print("==========statistics==========")
        statistic = s.statistics
        if statistic is not None:
            print("statistic.attachment_count: ", statistic.attachment_count)
            print("statistic.channel_count: ", statistic.channel_count)
            print("statistic.chunk_count: ", statistic.chunk_count)
            print("statistic.message_count: ", statistic.message_count)
            print("statistic.message_end_time: ", statistic.message_end_time)
            print("statistic.message_start_time: ", statistic.message_start_time)
            print("statistic.metadata_count: ", statistic.metadata_count)
            print("statistic.schema_count: ", statistic.schema_count)
            for key, value in statistic.channel_message_counts.items():
                print(f"channel_message_counts: {key} = {value}")

        print("==========schemas==========")
        print("size: ", len(s.schemas))
        schema = s.schemas[args.schema_id]
        print("schema.id: ", schema.id)
        print("schema.data: ", schema.data)
        print("schema.encoding: ", schema.encoding)
        print("schema.name: ", schema.name)

        print("==========channels==========")
        print("size: ", len(s.channels))
        channel = s.channels[args.schema_id]
        print("channel.id: ", channel.id)
        print("channel.topic: ", channel.topic)
        print("channel.message_encoding: ", channel.message_encoding)
        print("channel.schema_id: ", channel.schema_id)
        # qos
        for metadata_key, metadata_value in channel.metadata.items():
            print(f"metadata: {metadata_key} = {metadata_value}")

        print("==========chunk_indexes==========")
        print("size: ", len(s.chunk_indexes))
        for chunk_index in s.chunk_indexes:
            print("chunk_index.chunk_length: ", chunk_index.chunk_length)
            print("chunk_index.chunk_start_offset: ", chunk_index.chunk_start_offset)
            print("chunk_index.compression: ", chunk_index.compression)
            print("chunk_index.compressed_size: ", chunk_index.compressed_size)
            print("chunk_index.message_end_time: ", chunk_index.message_end_time)
            print(
                "chunk_index.message_index_length: ", chunk_index.message_index_length
            )
            for key, value in chunk_index.message_index_offsets.items():
                print(f"message_index_offsets: {key} = {value}")
            print("chunk_index.message_start_time: ", chunk_index.message_start_time)
            print("chunk_index.uncompressed_size: ", chunk_index.uncompressed_size)

        print("==========attachment_indexes==========")
        print("size: ", len(s.attachment_indexes))

        print("==========metadata_indexes==========")
        print("size: ", len(s.metadata_indexes))

        print("==========McapReader::iter_decoded_messages==========")
        for schema, channel, message, decoded_message in reader.iter_decoded_messages(
            topics=[args.topic_name]
        ):
            print("----------McapReader::iter_decoded_messages::message----------")
            print("message.channel_id: ", message.channel_id)
            print("message.log_time: ", message.log_time)
            print("message.publish_time: ", message.publish_time)
            print("message.sequence: ", message.sequence)
            print("message.data: ", message.data)
            print(
                "----------McapReader::iter_decoded_messages::decoded_message----------"
            )
            print("decoded_message: ", decoded_message)
            print("decoded_message.header: ", decoded_message.header)
            header = decoded_message.header
            print("decoded_message.header.stamp.sec: ", header.stamp.sec)
            print("decoded_message.header.stamp.nanosec: ", header.stamp.nanosec)
            print("decoded_message.header.frame_id: ", header.frame_id)
            print("decoded_message.data: ", decoded_message.data)

        del reader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "-t",
        "--topic_name",
        type=str,
        default="/my/e171",
        help="output bag path (folder or filepath) to write to",
    )
    parser.add_argument(
        "-s", "--schema_id", type=int, default=1, help="schema id to write to"
    )

    args = parser.parse_args()
    topic_name = args.topic_name
    print("topic_name: ", topic_name)
    schema_id = args.schema_id
    print("schema_id: ", schema_id)
    read_messages(args)


if __name__ == "__main__":
    main()
