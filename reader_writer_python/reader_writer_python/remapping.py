from mcap_ros2.writer import Writer
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from e171_msgs.msg._e171 import E171
from rclpy.serialization import serialize_message

def remapping(input_bag: str, output_bag: str):
    with open(input_bag, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        
        print("==========header==========")
        header = reader.get_header()
        print(f"profile: {header.profile}")
        print(f"library: {header.library}")
        
        s = reader.get_summary()
        print("==========schemas==========")
        print("size: ", len(s.schemas))
        my_schema = s.schemas[1]
        print("schema.id: ", my_schema.id)
        print("schema.data: ", my_schema.data)
        print("schema.encoding: ", my_schema.encoding)
        print("schema.name: ", my_schema.name)
        
        writer_sequence = 0
        
        with open(output_bag, "wb") as f2:
            writer = Writer(f2)
            writer_schema = writer.register_msgdef(
                my_schema.name,
                my_schema.data.decode("utf-8"),
            )

            for schema, channel, message, decoded_message in reader.iter_decoded_messages(topics=["/my/e171"]):
                print("message: ", message)
                print("decoded_message: ", decoded_message)
                print("message.sequence: ", message.sequence)
                print("message.data: ", message.data)
                writer_sequence += 1
                writer.write_message(
                    topic="/my/e171/remapping",
                    schema=writer_schema,
                    message=decoded_message,
                    sequence=writer_sequence,
                    log_time=message.log_time,
                    publish_time=message.publish_time,
                )
            writer.finish()
            
def main():
    input_bag = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/rosbag2_2025_04_27-20_01_07/rosbag2_2025_04_27-20_01_07_0.mcap"
    output_bag = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/rosbag2_2025_04_27-20_01_07/rosbag2_2025_04_27-20_01_07_0_remapping.mcap"
    remapping(input_bag, output_bag)
    
if __name__ == "__main__":
    main()