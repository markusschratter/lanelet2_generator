"""Lanelet2 OSM map builder."""

import os
from datetime import datetime
from xml.dom.minidom import parseString
import xml.etree.ElementTree as ET

import numpy as np
from pyproj import CRS, Transformer

from lanelet2_generator.geometry import pose2line, split_segments

_MGRS_BANDS = "CDEFGHJKLMNPQRSTUVWX"
_MGRS_COL_LETTERS = "ABCDEFGHJKLMNPQRSTUVWXYZ"
_MGRS_ROW_LETTERS = "ABCDEFGHJKLMNPQRSTUV"


def _parse_mgrs(mgrs_string):
    """
    Parse an MGRS string into UTM zone, hemisphere, easting, and northing.

    Accepts both full MGRS coordinates (e.g. "33TWN3497211656") and base grid
    zone designators (e.g. "33TWN"), in which case easting/northing offsets are 0
    and the result is the origin of the 100 km square.

    Note: Southern hemisphere support is approximate and not fully validated.
    """
    zone = int(mgrs_string[:2])
    band = mgrs_string[2]
    square = mgrs_string[3:5]
    coords = mgrs_string[5:]
    coord_len = len(coords) // 2
    easting_str = coords[:coord_len].ljust(5, "0")
    northing_str = coords[coord_len:].ljust(5, "0")
    easting_offset = int(easting_str)
    northing_offset = int(northing_str)

    col_letter, row_letter = square[0], square[1]

    set_number = (zone - 1) % 3
    col_index = _MGRS_COL_LETTERS.index(col_letter)
    col_base = (set_number * 8) % 24
    col_in_zone = (col_index - col_base) % 24
    easting_100km = col_in_zone + 1

    set_row = (zone - 1) % 2
    row_index = _MGRS_ROW_LETTERS.index(row_letter)
    if set_row == 1:
        row_index = (row_index - 5) % 20

    hemisphere = "north" if band >= "N" else "south"
    band_index = _MGRS_BANDS.index(band)

    raw_northing = row_index * 100000 + northing_offset

    if hemisphere == "north":
        # Band N (index 10) starts at equator.  Each band spans ~8 deg (~888 km).
        band_start_approx = (band_index - 10) * 888889
        cycle = max(0, int((band_start_approx - raw_northing + 1000000) // 2000000))
        northing = raw_northing + cycle * 2000000
    else:
        # Southern hemisphere: false northing is 10,000,000
        northing = raw_northing
        if northing < 1000000:
            northing += 10000000

    easting = easting_100km * 100000 + easting_offset
    return zone, hemisphere, easting, northing, band


def mgrs_to_wgs(mgrs_string):
    """Convert a full MGRS string to (lat, lon) WGS84."""
    zone, hemisphere, easting, northing, _ = _parse_mgrs(mgrs_string)
    is_south = hemisphere == "south"
    utm_crs = CRS.from_dict({"proj": "utm", "zone": zone, "south": is_south, "datum": "WGS84"})
    wgs84_crs = CRS.from_epsg(4326)
    transformer = Transformer.from_crs(utm_crs, wgs84_crs, always_xy=True)
    lon, lat = transformer.transform(easting, northing)
    return (lat, lon)


class LaneletMap:
    def __init__(self, mgrs="33TWN"):
        self.mgrs = mgrs
        self.element_num = 0
        self.root = ET.Element("osm", {"generator": "lanelet2_generator"})
        ET.SubElement(self.root, "MetaInfo", {"format_version": "1", "map_version": "2"})

        # Parse the base MGRS grid zone once, create a single Transformer
        zone, hemisphere, base_e, base_n, _ = _parse_mgrs(mgrs)
        self._base_easting = base_e
        self._base_northing = base_n
        is_south = hemisphere == "south"
        utm_crs = CRS.from_dict({"proj": "utm", "zone": zone, "south": is_south, "datum": "WGS84"})
        wgs84_crs = CRS.from_epsg(4326)
        self._transformer = Transformer.from_crs(utm_crs, wgs84_crs, always_xy=True)

    def _local_to_wgs84(self, x, y):
        """Convert local MGRS coordinates to (lat, lon) with sub-meter precision."""
        easting = self._base_easting + float(x)
        northing = self._base_northing + float(y)
        lon, lat = self._transformer.transform(easting, northing)
        return lat, lon

    def add_node(self, x, y, z):
        self.element_num += 1
        lat, lon = self._local_to_wgs84(x, y)
        mgrs_code_short = self.mgrs + ("%05d" % int(round(x)))[:3] + ("%05d" % int(round(y)))[:3]
        node = ET.SubElement(
            self.root, "node",
            {"id": str(self.element_num), "lat": str(lat), "lon": str(lon)},
        )
        for k, v in [
            ("type", ""), ("subtype", ""), ("mgrs_code", mgrs_code_short),
            ("local_x", str(x)), ("local_y", str(y)), ("ele", str(z)),
        ]:
            ET.SubElement(node, "tag", {"k": k, "v": v})
        return self.element_num

    def add_way(self, node_list):
        self.element_num += 1
        way = ET.SubElement(self.root, "way", {"id": str(self.element_num)})
        for nd in node_list:
            ET.SubElement(way, "nd", {"ref": str(nd)})
        for k, v in [("type", "line_thin"), ("subtype", "solid")]:
            ET.SubElement(way, "tag", {"k": k, "v": v})
        return self.element_num

    def add_relation(self, left_id, right_id, center_id=None, speed_limit=30):
        self.element_num += 1
        relation = ET.SubElement(self.root, "relation", {"id": str(self.element_num)})
        ET.SubElement(relation, "member", {"type": "way", "ref": str(left_id), "role": "left"})
        ET.SubElement(relation, "member", {"type": "way", "ref": str(right_id), "role": "right"})
        if center_id:
            ET.SubElement(relation, "member", {"type": "way", "ref": str(center_id), "role": "centerline"})
        for k, v in [
            ("type", "lanelet"), ("subtype", "road"), ("location", "urban"),
            ("participant:vehicle", "yes"), ("one_way", "yes"), ("speed_limit", str(speed_limit)),
        ]:
            ET.SubElement(relation, "tag", {"k": k, "v": v})
        return self.element_num

    def save(self, filename):
        parsed = parseString(ET.tostring(self.root, encoding="unicode")).toprettyxml(indent="  ")
        with open(filename, "w") as f:
            f.write(parsed)


def to_lanelet(
    poses,
    output_dir,
    *,
    width=2.0,
    mgrs="33TWN",
    offset=(0.0, 0.0, 0.0),
    use_centerline=False,
    split_distance=500,
    max_direction_change_deg=None,
    direction_change_window_m=None,
    speed_limit=30,
):
    """
    Build lanelet2 map from poses and save to output_dir.

    Returns:
        Path to saved .osm file
    """
    poses = np.asarray(poses, dtype=np.float64)
    if len(poses) == 0:
        raise ValueError("No poses to convert")

    left, right, center = pose2line(poses, width=width, offset=offset)
    m = LaneletMap(mgrs=mgrs)
    segments = split_segments(
        center, poses,
        split_distance=split_distance,
        max_direction_change_deg=max_direction_change_deg,
        direction_change_window_m=direction_change_window_m,
    )

    prev_left = prev_right = prev_center = None
    for start, end in segments:
        left_seg = left[start:end]
        right_seg = right[start:end]
        center_seg = center[start:end]

        if prev_left is not None:
            left_nodes = [prev_left] + [m.add_node(*n) for n in left_seg[1:]]
            right_nodes = [prev_right] + [m.add_node(*n) for n in right_seg[1:]]
            center_nodes = ([prev_center] + [m.add_node(*n) for n in center_seg[1:]]) if use_centerline else None
        else:
            left_nodes = [m.add_node(*n) for n in left_seg]
            right_nodes = [m.add_node(*n) for n in right_seg]
            center_nodes = [m.add_node(*n) for n in center_seg] if use_centerline else None

        prev_left = left_nodes[-1]
        prev_right = right_nodes[-1]
        prev_center = center_nodes[-1] if center_nodes else None

        left_way = m.add_way(left_nodes)
        right_way = m.add_way(right_nodes)
        center_way = m.add_way(center_nodes) if use_centerline else None
        m.add_relation(left_way, right_way, center_way, speed_limit=speed_limit)

    os.makedirs(output_dir, exist_ok=True)
    filename = os.path.join(output_dir, datetime.now().strftime("%y-%m-%d-%H-%M-%S") + "-lanelet2_map.osm")
    m.save(filename)
    return filename
