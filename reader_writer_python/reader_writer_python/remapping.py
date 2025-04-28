from mcap_ros2.writer import Writer
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import argparse


def remapping(args):
    with open(args.input_bag, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])

        print("==========header==========")
        header = reader.get_header()
        print(f"profile: {header.profile}")
        print(f"library: {header.library}")

        s = reader.get_summary()
        single_channle = None
        single_schema = None
        
        for key, value in s.channels.items():
            if value.topic == args.topic_name:
                single_channle = value
                print("find topic {} in mcap".format(args.topic_name))
                break
              
        if single_channle is None:
            raise ValueError(
                "The topic {} is not found in the input bag file.".format(
                    args.topic_name
                )
            )
            
        for key, value in s.schemas.items():
            if value.id == single_channle.schema_id:
                single_schema = value
                print("find schema {} in mcap".format(value.name))
                break
              
        if single_schema is None:
            raise ValueError(
                "The schema {} is not found in the input bag file.".format(
                    single_channle.schema_id
                )
            )
        
        # 计数
        writer_sequence = 0
        with open(args.output_bag, "wb") as f2:
            writer = Writer(f2)
            writer_schema = writer.register_msgdef(
                single_schema.name, single_schema.data.decode("utf-8")
            )

            for (
                schema,
                channel,
                message,
                decoded_message,
            ) in reader.iter_decoded_messages(topics=[args.topic_name]):
                writer_sequence += 1
                writer.write_message(
                    topic=args.remapping_topic_name,
                    schema=writer_schema,
                    message=decoded_message,
                    sequence=writer_sequence,
                    log_time=message.log_time,
                    publish_time=message.publish_time,
                )
            writer.finish()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-i", "--input_bag", type=str, required=True, help="Input bag file"
    )
    parser.add_argument(
        "-o", "--output_bag", type=str, required=True, help="Output bag file"
    )
    parser.add_argument(
        "-t",
        "--topic_name",
        type=str,
        help="Output topic name",
    )
    parser.add_argument(
        "-r",
        "--remapping_topic_name",
        type=str,
        help="Topic name to remap",
    )

    args = parser.parse_args()
    print("input_bag: ", args.input_bag)
    print("output_bag: ", args.output_bag)
    print("topic_name: ", args.topic_name)
    print("remapping_topic_name: ", args.remapping_topic_name)

    remapping(args)


if __name__ == "__main__":
    main()
