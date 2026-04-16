"""CLI: merge multiple Lanelet2 OSM files with unique IDs (offset remapping)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lanelet2_generator.osm_merge import merge_lanelet_osm_files


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Merge any number of Lanelet2 OSM files into one map (e.g. two, three, or more). "
            "Remaps node/way/relation ids and nd/member refs so IDs stay unique "
            "(default: cumulative offset from each file's max id; optional fixed step per file)."
        ),
        epilog=(
            "Examples:\n"
            "  %(prog)s map_a.osm map_b.osm map_c.osm -o merged.osm\n"
            "  %(prog)s a.osm b.osm c.osm d.osm -o all.osm --step-offset 2000000"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        metavar="FILE",
        help="Input .osm files (order preserved; pass as many as needed)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        required=True,
        help="Output merged .osm path",
    )
    g = parser.add_mutually_exclusive_group()
    g.add_argument(
        "--offsets",
        type=int,
        nargs="+",
        metavar="N",
        help="Explicit integer offset per input file (same count as inputs)",
    )
    g.add_argument(
        "--step-offset",
        type=int,
        metavar="N",
        help="Use offsets 0, N, 2N, ... for file 0, 1, 2, ... (can overlap if maps are large)",
    )
    args = parser.parse_args(argv)

    if args.offsets is not None and len(args.offsets) != len(args.inputs):
        parser.error(
            f"--offsets: need one integer per input file ({len(args.inputs)} files, got {len(args.offsets)} values)"
        )

    for p in args.inputs:
        if not p.exists():
            print(f"Input not found: {p}", file=sys.stderr)
            return 1

    merge_lanelet_osm_files(
        args.inputs,
        args.output,
        offsets=args.offsets,
        step_offset=args.step_offset,
    )
    print(f"Merged {len(args.inputs)} file(s) -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
