"""Read poses from rosbag2 (MCAP or sqlite3)."""

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def _create_reader(bag_path, storage_id="sqlite3"):
    storage_options = StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def _is_skip_pose(p0, p1, min_dist, max_dist):
    dist = ((p0.x - p1.x) ** 2 + (p0.y - p1.y) ** 2 + (p0.z - p1.z) ** 2) ** 0.5
    return dist < min_dist or dist > max_dist


def read_bag(path, interval=(0.1, 10000.0), storage_id=None):
    """
    Read poses from rosbag2 via /tf base_link.

    Args:
        path: Path to .mcap file or rosbag2 directory
        interval: (min_dist, max_dist) between consecutive poses [m]
        storage_id: "mcap" or "sqlite3"; auto-detected from path if None

    Returns:
        (N, 7) ndarray [x, y, z, qx, qy, qz, qw]
    """
    path_str = str(path).lower()
    if storage_id is None:
        storage_id = "mcap" if path_str.endswith(".mcap") else "sqlite3"

    reader = _create_reader(path, storage_id=storage_id)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    pose_list = []
    prev_trans = None

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/tf":
            continue
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        for transform in msg.transforms:
            if transform.child_frame_id != "base_link":
                continue
            trans = transform.transform.translation
            rot = transform.transform.rotation
            if prev_trans is None:
                prev_trans = trans
            elif _is_skip_pose(prev_trans, trans, interval[0], interval[1]):
                continue
            pose_list.append(np.r_[trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w])
            prev_trans = trans

    return np.array(pose_list) if pose_list else np.empty((0, 7))
