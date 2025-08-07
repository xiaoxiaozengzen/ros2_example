import argparse
import struct
import os
import can
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String, Int8MultiArray

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

# Size of ZCANDataObj in bytes
size_of_ZCANDataObj = 100

# Packet types
PACKET_TYPE_CAN = 0
PACKET_TYPE_CANFD = 1

# Channel IDs
CAN_CHANNEL_ADREDUNDANCY = 0
CAN_CHANNEL_CANFD2 = 1
CAN_CHANNEL_CHASSIS = 2
CAN_CHANNEL_CANFD3 = 3
CAN_CHANNEL_CANFD1 = 4

def parse_fmr(channel_id: int, data: memoryview, blf_file: can.io.BLFWriter, offest: int):
    count_offest = offest
    count_offest += 4
    message_timestamp = struct.unpack('<Q', data[count_offest:count_offest + 8])[0]

    count_offest += 8
    # flag

    count_offest += 4
    # extraData[4]

    count_offest += 4
    message_can_id = struct.unpack('<I', data[count_offest:count_offest + 4])[0]

    count_offest += 4
    message_data_length = struct.unpack('<B', data[count_offest:count_offest + 1])[0]
    print("message_id: {:X}, mesage_channel: {}, message_dlc: {}, message_timestamp {}".format(
        message_can_id, channel_id, message_data_length, message_timestamp / 1e6
    ))

    count_offest += 1
    # flags

    count_offest += 1
    # res0

    count_offest += 1
    # res2

    count_offest += 1
    # data

    fmr_can_message = can.Message(
        timestamp=message_timestamp / 1e6,  # Convert to seconds
        arbitration_id=message_can_id,
        is_extended_id=False,
        is_remote_frame=False,
        is_error_frame=False,
        channel=channel_id,
        dlc=message_data_length,
        data=data[count_offest:count_offest + message_data_length],
        is_fd=True,
        is_rx=True,
        bitrate_switch=True,
        error_state_indicator=False
    )
    blf_file.on_message_received(fmr_can_message)

def read_messages(args):
    with open(args.input, "rb") as f, open(args.output, "wb+") as f2:
        blf_file = can.io.BLFWriter(f2)
        reader = make_reader(f, decoder_factories=[DecoderFactory()])

        header = reader.get_header()
        print(f"profile: {header.profile}")
        print(f"library: {header.library}")

        s = reader.get_summary()
        print("==========statistics==========")
        statistic = s.statistics
        if statistic is None:
            print("No statistics available.")
            exit(-1)
            
        aim_topic_name = args.topic_name
        aim_chanel_id = 0
        aim_schema_id = 0
        
        print("aim_topic_name: ", aim_topic_name)       
        for id, single_channel in s.channels.items():
            if single_channel.topic == aim_topic_name:
                aim_chanel_id = single_channel.id
                print("aim_chanel_id: ", aim_chanel_id)
                aim_schema_id = single_channel.schema_id
                print("aim_schema_id: ", aim_schema_id)
                break
        
        print("==========channels==========")
        print("size: ", len(s.channels))
        channel = s.channels[aim_chanel_id]
        print("channel.id: ", channel.id)
        print("channel.topic: ", channel.topic)
        print("channel.message_encoding: ", channel.message_encoding)
        print("channel.schema_id: ", channel.schema_id)

        print("==========schemas==========")
        print("size: ", len(s.schemas))
        schema = s.schemas[aim_schema_id]
        print("schema.id: ", schema.id)
        print("schema.data: ", schema.data)
        print("schema.encoding: ", schema.encoding)
        print("schema.name: ", schema.name)

        print("==========McapReader::iter_decoded_messages==========")
        count = 0
        for schema, channel, message, decoded_message in reader.iter_decoded_messages(
            topics=[args.topic_name]
        ):
            data_bytes = bytearray((x % 256) for x in decoded_message.data)  # 兼容 int8 和 uint8
            data = memoryview(data_bytes)
            data_size = len(data)
            offest = 0
            
            length = 0
            timestamp = 0
            count_of_data = 0
            count += 1
            
            print("--------------count: {}--------------".format(count))
            # print("data_size: ", data_size)
            # print("decoded_message.data: ", data)
            
            offest += 0
            length = struct.unpack('<Q', data[offest:offest + 8])[0]
            # print("length: ", length)
            
            offest += 8
            timestamp = struct.unpack('<Q',data[offest:offest +8])[0]
            timestamp_s = timestamp / 1e9
            print("timestamp: ", timestamp_s, "s")
            
            offest += 8
            count_of_data = struct.unpack('<Q', data[offest:offest + 8])[0]
            # print("count_of_data: ", count_of_data)
            
            offest += 8
            # all_data
            
            for i in range(count_of_data):
                count_offest = offest + i * size_of_ZCANDataObj
                
                count_offest += 0
                data_type = struct.unpack('<B', data[count_offest:count_offest + 1])[0]
                # print("{}_th data_type: ".format(i), data_type)
                
                count_offest += 1
                channel = struct.unpack('<B', data[count_offest:count_offest + 1])[0]
                # print("{}_th channel: ".format(i), channel)
                
                count_offest += 1
                flag = struct.unpack('<H', data[count_offest:count_offest + 2])[0]
                # print("{}_th flag: ".format(i), flag)
                
                count_offest += 2
                # extraData[4]
                
                if data_type == PACKET_TYPE_CANFD and channel == args.channel_id:
                    parse_fmr(channel, data, blf_file, count_offest)
                else:
                  continue
        
        blf_file.stop()
        del reader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-i",
        "--input",
        type=str,
        required=True,
        help="input mcap file path to read from",
    )
    parser.add_argument(
        "-t",
        "--topic_name",
        type=str,
        default="/canbus/rawcan",
        help="output bag path (folder or filepath) to write to",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        required=True,
        help="output bag path (folder or filepath) to write to",
    )
    parser.add_argument(
        "-c",
        "--channel_id",
        type=int,
        default=0,
        help="channel ID to filter messages",
    )

    args = parser.parse_args()
    input_file = args.input
    output_file = args.output
    topic_name = args.topic_name
    channel_id = args.channel_id
    
    if not os.path.exists(input_file):
        print(f"Input file {input_file} does not exist.")
        exit(1)
    if os.path.exists(output_file):
        print(f"Output file {output_file} already exists.")
        exit(1)
    print(f"Reading messages from {input_file} on topic {topic_name} with channel ID {channel_id}...")
    print(f"Output will be written to {output_file}.")
    
    read_messages(args)


if __name__ == "__main__":
    main()
