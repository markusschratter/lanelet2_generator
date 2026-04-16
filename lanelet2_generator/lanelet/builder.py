"""Lanelet2 OSM map builder."""

import os
from datetime import datetime
from xml.dom.minidom import parseString
import xml.etree.ElementTree as ET

import numpy as np
from pyproj import CRS, Transformer

from lanelet2_generator.geometry import pose2line, split_segments
from lanelet2_generator.mgrs_utils import mgrs_to_wgs, parse_mgrs


class LaneletMap:
    def __init__(self, mgrs="33TWN"):
        self.mgrs = mgrs
        self.element_num = 0
        self.root = ET.Element("osm", {"generator": "lanelet2_generator"})
        ET.SubElement(self.root, "MetaInfo", {"format_version": "1", "map_version": "2"})

        # Parse the base MGRS grid zone once, create a single Transformer
        zone, hemisphere, base_e, base_n, _ = parse_mgrs(mgrs)
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
    geo_origin=None,
    use_centerline=False,
    split_distance=500,
    max_direction_change_deg=None,
    direction_change_window_m=None,
    speed_limit=30,
    bidirectional=True,
):
    """
    Build lanelet2 map from poses and save to output_dir.

    Args:
        geo_origin: Optional UTM origin (easting, northing, elevation) of the
            input local frame, e.g. from a .offset companion file.  When set,
            poses are shifted from local-frame to MGRS-local before processing.

    Returns:
        Path to saved .osm file
    """
    poses = np.asarray(poses, dtype=np.float64)
    if len(poses) == 0:
        raise ValueError("No poses to convert")

    if geo_origin is not None:
        _, _, base_e, base_n, _ = parse_mgrs(mgrs)
        poses = poses.copy()
        poses[:, 0] += geo_origin[0] - base_e
        poses[:, 1] += geo_origin[1] - base_n
        poses[:, 2] += geo_origin[2]

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

    if bidirectional:
        # Build reverse-direction lanelets in reverse segment order and share
        # boundary nodes between adjacent segments for routing connectivity.
        prev_rev_left = prev_rev_right = prev_rev_center = None
        for start, end in reversed(segments):
            left_seg = left[start:end]
            right_seg = right[start:end]
            center_seg = center[start:end]

            rev_left_geom = right_seg[::-1]
            rev_right_geom = left_seg[::-1]
            rev_center_geom = center_seg[::-1]

            if prev_rev_left is not None:
                rev_left_nodes = [prev_rev_left] + [m.add_node(*n) for n in rev_left_geom[1:]]
                rev_right_nodes = [prev_rev_right] + [m.add_node(*n) for n in rev_right_geom[1:]]
                rev_center_nodes = (
                    [prev_rev_center] + [m.add_node(*n) for n in rev_center_geom[1:]]
                ) if use_centerline else None
            else:
                rev_left_nodes = [m.add_node(*n) for n in rev_left_geom]
                rev_right_nodes = [m.add_node(*n) for n in rev_right_geom]
                rev_center_nodes = [m.add_node(*n) for n in rev_center_geom] if use_centerline else None

            prev_rev_left = rev_left_nodes[-1]
            prev_rev_right = rev_right_nodes[-1]
            prev_rev_center = rev_center_nodes[-1] if rev_center_nodes else None

            rev_left_way = m.add_way(rev_left_nodes)
            rev_right_way = m.add_way(rev_right_nodes)
            rev_center_way = m.add_way(rev_center_nodes) if use_centerline else None
            m.add_relation(rev_left_way, rev_right_way, rev_center_way, speed_limit=speed_limit)

    os.makedirs(output_dir, exist_ok=True)
    filename = os.path.join(output_dir, datetime.now().strftime("%y-%m-%d-%H-%M-%S") + "-lanelet2_map.osm")
    m.save(filename)
    return filename
