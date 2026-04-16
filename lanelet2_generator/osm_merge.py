"""Merge multiple Lanelet2 OSM XML files by remapping element IDs so they stay unique."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import List, Optional, Sequence, Union
import xml.etree.ElementTree as ET
from xml.dom.minidom import parseString

PathLike = Union[str, Path]


def _local_tag(tag: str) -> str:
    if tag.startswith("{"):
        return tag.split("}", 1)[1]
    return tag


def max_element_id(root: ET.Element) -> int:
    """Largest numeric ``id`` among all ``node`` / ``way`` / ``relation`` elements."""
    m = 0
    for elem in root.iter():
        if _local_tag(elem.tag) not in ("node", "way", "relation"):
            continue
        eid = elem.get("id")
        if eid is None:
            continue
        try:
            m = max(m, int(eid))
        except ValueError:
            continue
    return m


def compute_auto_offsets(paths: Sequence[PathLike]) -> List[int]:
    """
    Cumulative offsets so remapped IDs do not overlap between files.

    Let ``M_k`` be the maximum ``node``/``way``/``relation`` id in file ``k``.
    Offset for file ``k`` is ``O_k = sum_{i=0}^{k-1} M_i`` (and ``O_0 = 0``).
    """
    offsets: List[int] = []
    cumulative = 0
    for p in paths:
        root = ET.parse(str(p)).getroot()
        offsets.append(cumulative)
        cumulative += max_element_id(root)
    return offsets


def compute_step_offsets(num_files: int, step: int) -> List[int]:
    """Offsets ``0, step, 2*step, ...`` for ``num_files`` inputs."""
    if num_files < 0:
        raise ValueError("num_files must be non-negative")
    if step < 0:
        raise ValueError("step must be non-negative")
    return [i * step for i in range(num_files)]


def apply_id_offset(subtree: ET.Element, offset: int) -> None:
    """
    Add ``offset`` to every ``node``/``way``/``relation`` ``id`` and every
    ``nd``/``member`` ``ref`` in ``subtree`` (in-place).
    """
    if offset == 0:
        return
    for elem in subtree.iter():
        tag = _local_tag(elem.tag)
        if tag in ("node", "way", "relation") and "id" in elem.attrib:
            elem.attrib["id"] = str(int(elem.attrib["id"]) + offset)
        if tag in ("nd", "member") and "ref" in elem.attrib:
            elem.attrib["ref"] = str(int(elem.attrib["ref"]) + offset)


def merge_lanelet_osm_files(
    input_paths: Sequence[PathLike],
    output_path: PathLike,
    *,
    offsets: Optional[Sequence[int]] = None,
    step_offset: Optional[int] = None,
) -> Path:
    """
    Merge Lanelet2 OSM files into one document (any number of inputs: 2, 3, or more).

    ``node``, ``way``, and ``relation`` elements get new ``id`` values; ``nd``
    and ``member`` ``ref`` attributes are updated to match. Only one ``MetaInfo``
    and one ``bound`` (if present) are kept (first occurrence across inputs).

    Args:
        input_paths: OSM files to merge (order is preserved).
        output_path: Destination ``.osm`` path.
        offsets: Optional per-file integer offsets (same length as ``input_paths``).
            If omitted, offsets are chosen with :func:`compute_auto_offsets`
            unless ``step_offset`` is set.
        step_offset: If set and ``offsets`` is ``None``, offsets are
            ``0, step_offset, 2*step_offset, ...``.

    Returns:
        Path to the written file.
    """
    paths = [Path(p) for p in input_paths]
    if not paths:
        raise ValueError("At least one input OSM file is required")
    out = Path(output_path)

    if offsets is not None:
        if len(offsets) != len(paths):
            raise ValueError(
                f"offsets length ({len(offsets)}) must match input_paths ({len(paths)})"
            )
        use_offsets = [int(x) for x in offsets]
    elif step_offset is not None:
        use_offsets = compute_step_offsets(len(paths), int(step_offset))
    else:
        use_offsets = compute_auto_offsets(paths)

    first_root = ET.parse(str(paths[0])).getroot()
    merged = ET.Element(first_root.tag, dict(first_root.attrib))

    metainfo_seen = False
    bounds_seen = False
    for idx, path in enumerate(paths):
        root = ET.parse(str(path)).getroot()
        off = use_offsets[idx]
        for child in root:
            tag = _local_tag(child.tag)
            if tag == "MetaInfo":
                if metainfo_seen:
                    continue
                metainfo_seen = True
            elif tag in ("bound", "bounds"):
                if bounds_seen:
                    continue
                bounds_seen = True
            block = copy.deepcopy(child)
            apply_id_offset(block, off)
            merged.append(block)

    _write_osm_pretty(merged, out)
    return out


def _write_osm_pretty(root: ET.Element, path: Path) -> None:
    parsed = parseString(ET.tostring(root, encoding="unicode")).toprettyxml(indent="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(parsed)
