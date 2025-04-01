from __future__ import annotations
import argparse

from typing import TYPE_CHECKING, cast
from pathlib import Path

from rosbags.highlevel.anyreader import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from rosbags.interfaces import ConnectionExtRosbag1

# ROS1 CameraInfo message definition
CAMERAINFO_DEFINITION = """
std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] D
float64[9] K
float64[9] R
float64[12] P
uint32 binning_x
uint32 binning_y
sensor_msgs/RegionOfInterest roi
"""

# Missing definitions for RegionOfInterest and Header
ROI_DEFINITION = """
uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify
"""

HEADER_DEFINITION = """
uint32 seq
time stamp
string frame_id
"""

def downgrade_camerainfo_to_rosbag1(src: Path, dst: Path) -> None:
    """Edit message definitions in a rosbag1.

    Args:
        src: Rosbag1 source path.
        dst: Destination path.
    """
    typename_camera_info = 'sensor_msgs/msg/CameraInfo'  # Adjusted to correct name
    typename_header = 'std_msgs/Header'
    typename_roi = 'sensor_msgs/RegionOfInterest'

    # Initialize typestore and register message definitions
    typestore = get_typestore(Stores.EMPTY)

    # Register the missing message types
    typestore.register(get_types_from_msg(ROI_DEFINITION, typename_roi))
    typestore.register(get_types_from_msg(HEADER_DEFINITION, typename_header))
    typestore.register(get_types_from_msg(CAMERAINFO_DEFINITION, typename_camera_info))

    # Debugging: Print out registered message types
    print("Registered Types:", typestore.types.keys())

    # Fetch the CameraInfo type from the typestore
    if typename_camera_info not in typestore.types:
        print(f"Error: '{typename_camera_info}' not found in typestore.")
        return

    CameraInfo = typestore.types[typename_camera_info]  # noqa: N806

    with AnyReader([src]) as reader, Writer(dst) as writer:
        conn_map = {}

        for conn in reader.connections:
            ext = cast('ConnectionExtRosbag1', conn.ext)

            # Use the updated message definition and md5sum for CameraInfo
            if conn.msgtype == typename_camera_info:
                from_typestore = typestore
            else:
                from_typestore = reader.typestore

            conn_map[conn.id] = writer.add_connection(
                conn.topic,
                conn.msgtype,
                typestore=from_typestore,
                callerid=ext.callerid,
                latching=ext.latching,
            )

        for conn, timestamp, data in reader.messages():
            wconn = conn_map[conn.id]

            if conn.msgtype == typename_camera_info:
                msg = cast('sensor_msgs__msg__CameraInfo', reader.deserialize(data, conn.msgtype))
                converted_msg = CameraInfo(
                    header=msg.header,
                    height=msg.height,
                    width=msg.width,
                    distortion_model=msg.distortion_model,
                    # Map lower case names to upper case
                    D=msg.d,
                    K=msg.k,
                    R=msg.r,
                    P=msg.p,
                    binning_x=msg.binning_x,
                    binning_y=msg.binning_y,
                    roi=msg.roi,
                )
                # Manually serialize the message for ROS1
                outdata: memoryview | bytes = typestore.serialize_ros1(converted_msg, typename_camera_info)
            else:
                outdata = data

            writer.write(wconn, timestamp, outdata)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Downgrade CameraInfo messages in a ROS1 bag.")
    parser.add_argument("--source_bag", type=str, help="Path to the source ROS1 bag file.")
    parser.add_argument("--destination_bag", type=str, help="Path to the destination ROS1 bag file.")

    args = parser.parse_args()
    
    source_bag = Path(args.source_bag)
    destination_bag = Path(args.destination_bag)

    downgrade_camerainfo_to_rosbag1(source_bag, destination_bag)

