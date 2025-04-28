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
        print("==========schemas==========")
        print("size: ", len(s.schemas))
        my_schema = s.schemas[args.schema_id]
        print("schema.id: ", my_schema.id)
        print("schema.data: ", my_schema.data)
        print("schema.encoding: ", my_schema.encoding)
        print("schema.name: ", my_schema.name)

        writer_sequence = 0

        with open(args.output_bag, "wb") as f2:
            writer = Writer(f2)
            writer_schema = writer.register_msgdef(
                my_schema.name, my_schema.data.decode("utf-8")
            )

            for (
                schema,
                channel,
                message,
                decoded_message,
            ) in reader.iter_decoded_messages(topics=["/my/e171"]):
                print("message: ", message)
                print("decoded_message: ", decoded_message)
                print("message.sequence: ", message.sequence)
                print("message.data: ", message.data)
                writer_sequence += 1
                writer.write_message(
                    topic=args.topic_name,
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
        default="/my/e171/remapping",
        help="Topic name to remap",
    )
    parser.add_argument(
        "-s", "--schema_id", type=int, default=1, help="Schema ID to remap"
    )
    args = parser.parse_args()
    print("topic_name: ", args.topic_name)
    print("schema_id: ", args.schema_id)
    print("input_bag: ", args.input_bag)
    print("output_bag: ", args.output_bag)
    print("==========remapping==========")

    remapping(args)


if __name__ == "__main__":
    main()
