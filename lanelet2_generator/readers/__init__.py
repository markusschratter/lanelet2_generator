from lanelet2_generator.readers.base import load_path
from lanelet2_generator.readers.csv import read_csv
from lanelet2_generator.readers.ply import read_offset, read_ply
from lanelet2_generator.readers.yaml_waypoints import read_yaml

__all__ = ["load_path", "read_bag", "read_csv", "read_offset", "read_ply", "read_yaml"]


def __getattr__(name):
    if name == "read_bag":
        from lanelet2_generator.readers.bag import read_bag
        return read_bag
    raise AttributeError(f"module 'lanelet2_generator.readers' has no attribute {name}")
