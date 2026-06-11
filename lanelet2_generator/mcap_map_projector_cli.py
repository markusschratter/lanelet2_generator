"""CLI: generate map YAML sidecars from GNSS NavSatFix messages in an MCAP file."""

import argparse
from pathlib import Path

from lanelet2_generator.mcap_map_projector import (
    DEFAULT_GNSS_TOPIC,
    generate_map_projector_from_mcap,
)


def main():
    parser = argparse.ArgumentParser(
        description="Generate map_projector_info.yaml and map_config.yaml from "
        "GNSS NavSatFix messages in an MCAP recording (via the mcap CLI). "
        "Falls back to local YAML when the topic is missing or contains no valid fixes."
    )
    parser.add_argument(
        "mcap",
        type=Path,
        help="Input MCAP recording",
    )
    parser.add_argument(
        "output_dir",
        nargs="?",
        type=Path,
        default=None,
        help="Output directory for YAML files (or use --output)",
    )
    parser.add_argument(
        "--output",
        dest="output_opt",
        type=Path,
        default=None,
        help="Output directory (alternative to positional)",
    )
    parser.add_argument(
        "--gnss-topic",
        "--topic",
        dest="topic",
        default=DEFAULT_GNSS_TOPIC,
        help=f"NavSatFix topic in the MCAP (default: {DEFAULT_GNSS_TOPIC})",
    )
    parser.add_argument(
        "--min-fixes",
        type=int,
        default=1,
        metavar="N",
        help="Minimum valid fixes required for georef YAML (default: 1)",
    )
    parser.add_argument(
        "--max-fixes",
        type=int,
        default=500,
        metavar="N",
        help="Stop after collecting this many valid fixes (default: 500)",
    )
    parser.add_argument(
        "--max-messages",
        type=int,
        default=50_000,
        metavar="N",
        help="Stop after scanning this many topic messages (default: 50000)",
    )
    parser.add_argument(
        "--mcap-cmd",
        default=None,
        help="Path to mcap CLI (default: mcap on PATH)",
    )
    parser.add_argument("--yaml-name", default="map_projector_info.yaml")
    parser.add_argument("--map-config-name", default="map_config.yaml")

    args = parser.parse_args()
    output_dir = args.output_opt if args.output_opt is not None else args.output_dir
    if output_dir is None:
        parser.error("Output directory required (positional or --output)")

    yaml_path, map_cfg_path, info = generate_map_projector_from_mcap(
        output_dir,
        args.mcap,
        topic=args.topic,
        min_valid_fixes=args.min_fixes,
        max_valid_fixes=args.max_fixes,
        max_messages=args.max_messages,
        mcap_cmd=args.mcap_cmd,
        yaml_name=args.yaml_name,
        map_config_name=args.map_config_name,
    )

    print(f"Wrote {yaml_path}")
    print(f"Wrote {map_cfg_path}")
    if info["mode"] == "local":
        print(
            f"mode=local ({info['reason']})  topic={info['topic']}  "
            f"messages_seen={info['messages_seen']}  valid_fixes={info['valid_fixes']}"
        )
    else:
        origin = info["map_origin"]
        gnss = info["gnss_median"]
        print(
            f"mode=mgrs  mgrs_grid={info['mgrs_grid']}  utm_frame={info['utm_frame']}  "
            f"topic={info['topic']}  "
            f"map_origin=({origin[0]:.8f}, {origin[1]:.8f}, {origin[2]:.3f})  "
            f"gnss_median=({gnss[0]:.8f}, {gnss[1]:.8f}, {gnss[2]:.3f})  "
            f"valid_fixes={info['valid_fixes']}/{info['messages_seen']}"
        )


if __name__ == "__main__":
    main()
