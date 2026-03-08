from lanelet2_generator.readers.base import load_path
from lanelet2_generator.readers.csv import read_csv
from lanelet2_generator.readers.ply import read_ply

__all__ = ["load_path", "read_bag", "read_csv", "read_ply"]


def __getattr__(name):
    if name == "read_bag":
        from lanelet2_generator.readers.bag import read_bag
        return read_bag
    raise AttributeError(f"module 'lanelet2_generator.readers' has no attribute {name}")
