import sys
import argparse
import time

from mcap_ros2.writer import Writer as McapWriter
from mcap_ros2.writer import Writer as Ros2Writer

SCHEMA_NAME = "e171_msgs/msg/E171"
# 来自reader得到的schema.data，去掉多余的`/n`
SCHEMA_TEXT = """\
std_msgs/Header header
uint8[] data
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.
# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp
# Transform frame with which this data is associated.
string frame_id
================================================================================
MSG: builtin_interfaces/Time
# Time indicates a specific point in time, relative to a clock's 0 point.
# The seconds component, valid over all int32 values.
int32 sec
# The nanoseconds component, valid in the range [0, 10e9).
uint32 nanosec"""

"""This example demonstrates creating a ROS2 MCAP file without a ROS2 environment."""


def write_noenv(output_path: str):
    with open(output_path, "wb") as f:
        writer = McapWriter(f)
        schema = writer.register_msgdef(SCHEMA_NAME, SCHEMA_TEXT)
        for i in range(10):
            writer.write_message(
                topic="/my/e171/writer",
                schema=schema,
                message={
                    "header": {
                        "stamp": {"sec": 1111, "nanosec": 2222},
                        "frame_id": "fuck",
                    },
                    "data": [ord(c) for c in ["a", "b", "c", "d", "e"]],
                },
                sequence=i,
            )
            time.sleep(1)
        writer.finish()


SCHEMA_DATA = "std_msgs/Header header\nuint8[] data\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data\n# in a particular coordinate frame.\n\n# Two-integer timestamp that is expressed as seconds and nanoseconds.\nbuiltin_interfaces/Time stamp\n\n# Transform frame with which this data is associated.\nstring frame_id\n\n================================================================================\nMSG: builtin_interfaces/Time\n# Time indicates a specific point in time, relative to a clock's 0 point.\n\n# The seconds component, valid over all int32 values.\nint32 sec\n\n# The nanoseconds component, valid in the range [0, 10e9).\nuint32 nanosec\n"


def write_ros(output_path: str):
    with open(output_path, "wb") as f:
        ros2_writer = Ros2Writer(f)
        schema = ros2_writer.register_msgdef(SCHEMA_NAME, SCHEMA_DATA)
        for i in range(10):
            ros2_writer.write_message(
                topic="/my/e171/ros2writer",
                schema=schema,
                message={
                    "header": {
                        "stamp": {"sec": 1111, "nanosec": 2222},
                        "frame_id": "nihao",
                    },
                    "data": [ord(c) for c in ["a", "b", "c", "d", "e"]],
                },
                sequence=i,
            )
            time.sleep(1)
        ros2_writer.finish()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="output directory to create and write to")

    args = parser.parse_args()
    noenv_output = (
        "/mnt/workspace/cgz_workspace/Exercise/ros2_example/ros2_mcap_noenv.mcap"
    )
    write_noenv(noenv_output)

    ros2_output = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/ros2_mcap.mcap"
    write_ros(ros2_output)


if __name__ == "__main__":
    main()
