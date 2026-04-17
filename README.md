# lanelet2_generator

Generate Lanelet2 maps from path data. Supports multiple input formats, a LAS/LAZ-to-PCD MGRS conversion tool, and a ROS 2 service for route-based generation.

This package is based on [bag2lanelet](https://github.com/autowarefoundation/autoware_tools/tree/main/bag2lanelet) by the Autoware Foundation.

## Quick Start

### Docker

**Point cloud (LAS/LAZ → PCD + YAML)** — source CRS uses `--utm-frame` (or `--epsg`); this is separate from lanelet MGRS.

| | |
|---|---|
| **input** | Positional path, or `--input`: `input.las` / `input.laz` |
| **output** | Positional path, or `--output`: output **directory**, or a full path ending in **`.pcd`**. If omitted, the input file’s directory is used. Writes `map_projector_info.yaml` and `map_config.yaml` next to the PCD. |

```bash
# LAS/LAZ -> pointcloud_map.pcd + map_projector_info.yaml + map_config.yaml (output defaults to input folder)
docker/pointcloud_converter.sh data/merged_clean.las --utm-frame 32N --voxel-size 0.1 --color-by auto

# Explicit input/output flags and exact PCD path:
docker/pointcloud_converter.sh --input data/merged_clean.las --output data/pointcloud_map.pcd \
  --utm-frame 32N --voxel-size 0.1 --color-by auto
```

**Lanelet (trajectory → `.osm`)** — map frame uses `--mgrs` or `--map-projector-info` (`mgrs_grid`); not `--utm-frame`.

| | |
|---|---|
| **input** | `<input>` (positional) or `--input`: CSV, PLY, YAML, MCAP, or rosbag2 directory |
| **output** | `<output_lanelet>` (positional) or `--output`: directory for the generated `.osm`. If omitted, the input file’s directory is used. |
| **`.osm` name** | `--output-file` (optional): exact filename; default is timestamped `YY-MM-DD-HH-MM-SS-lanelet2_map.osm` |

```bash
# PLY -> lanelet2 .osm (output defaults to input folder)
docker/lanelet2_generator.sh data/traj_fusion_gps.ply --map-projector-info data/map_projector_info.yaml --width 4.0 --speed-limit 20 --split-distance 100
# explicit input/output flags (optional):
# docker/lanelet2_generator.sh -- --input /input/traj.ply --output /output --map-projector-info /input/map_projector_info.yaml --width 4.0 --speed-limit 20
```

```bash
# Merge multiple Lanelet2 .osm files (two, three, …) — inputs and -o must sit under one directory tree
docker/merge_lanelets.sh data/map_a.osm data/map_b.osm data/map_c.osm -o data/merged.osm
```

### Direct Python 3

**Point cloud (`las_mgrs_cli`)** — same input/output idea as `docker/pointcloud_converter.sh`: `--utm-frame` / `--epsg` for the LAS/LAZ CRS.

| | |
|---|---|
| **input** | Positional path or `--input`: `input.las` / `input.laz` |
| **output** | Positional path or `--output`: output **directory**, or a path ending in **`.pcd`**. If omitted, the input file’s directory is used (PCD name from `--pcd-name`, default `pointcloud_map.pcd`). |

**Lanelet (`lanelet2_generator.cli`)** — same as Docker lanelet: `--mgrs` / `--map-projector-info`; optional `--input`, `--output`, `--output-file`.

| | |
|---|---|
| **input** | positional `input` or `--input` |
| **output** | positional `output_lanelet` or `--output` (directory for `.osm`) |
| **`.osm` name** | `--output-file` (optional) |

```bash
# LAS/LAZ -> pointcloud_map.pcd + map_projector_info.yaml + map_config.yaml
python3 -m lanelet2_generator.las_mgrs_cli data/merged_clean.las data --utm-frame 32N --voxel-size 0.1 --color-by auto

# Same with flags and an explicit .pcd path:
python3 -m lanelet2_generator.las_mgrs_cli --input data/merged_clean.las --output data/pointcloud_map.pcd \
  --utm-frame 32N --voxel-size 0.1 --color-by auto

# PLY -> lanelet2 .osm
python3 -m lanelet2_generator.cli data/traj_fusion_gps.ply data --map-projector-info data/map_projector_info.yaml --width 4.0 --speed-limit 20 --split-distance 100

# Merge multiple .osm maps (IDs remapped; default offsets avoid collisions)
python3 -m lanelet2_generator.merge_lanelets_cli data/map_a.osm data/map_b.osm -o data/merged.osm
```

## Architecture

### Package Layout

```
lanelet2_generator/
├── lanelet2_generator/               # Plain Python library (no ROS)
│   ├── __init__.py                   #   generate() entry point, lazy read_bag import
│   ├── cli.py                        #   CLI: argparse → generate()
│   ├── las_mgrs_cli.py               #   CLI: LAS/LAZ UTM -> local MGRS PCD + map_projector_info.yaml
│   ├── merge_lanelets_cli.py         #   CLI: merge multiple Lanelet2 .osm files (ID remap)
│   ├── osm_merge.py                  #   merge_lanelet_osm_files(), auto/step offsets
│   ├── mgrs_utils.py                 #   Shared MGRS parsing/origin helpers
│   ├── readers/
│   │   ├── base.py                   #   load_path() — dispatches by file extension
│   │   ├── csv.py                    #   read_csv() — vectorized yaw→quaternion
│   │   ├── ply.py                    #   read_ply() — with input validation
│   │   ├── yaml_waypoints.py         #   read_yaml() — waypoint YAML to (N,7)
│   │   └── bag.py                    #   read_bag() — lazily imported (requires ROS)
│   ├── filtering/
│   │   └── path.py                   #   filter_path(), min_distance, downsample
│   ├── geometry/
│   │   └── path.py                   #   pose2line() vectorized, split_segments() O(n)
│   └── lanelet/
│       └── builder.py                #   LaneletMap, to_lanelet(), cached Transformer
├── lanelet2_generator_node/          # ROS 2 node (only ROS component)
│   └── route_to_lanelet_node.py      #   /api/routing/set_route_points service
├── launch/
│   └── route_to_lanelet.launch.xml
├── docker/
│   ├── Dockerfile                     #   Container image for CLI tools
│   ├── pointcloud_converter.sh        #   Docker wrapper for LAS/LAZ -> PCD
│   ├── lanelet2_generator.sh          #   Docker wrapper for lanelet map generation
│   └── merge_lanelets.sh              #   Docker wrapper for merging multiple .osm maps
├── sample_data/
├── CMakeLists.txt
├── package.xml
├── pyproject.toml
└── requirements.txt
```

### Data Flow

```mermaid
flowchart LR
    subgraph inputs [Input Sources]
        CSV[".csv"]
        PLY[".ply"]
        MCAP[".mcap"]
        BAG["rosbag2 dir"]
        SVC["SetRoutePoints"]
    end

    subgraph entry [Entry Points]
        CLI["CLI\n(plain Python)"]
        Node["ROS 2 Node"]
    end

    subgraph pipeline ["generate() — unified pipeline"]
        direction LR
        LP["load_path()"]
        FP["filter_path()"]
        TL["to_lanelet()"]
    end

    subgraph lp_detail ["load_path() dispatch"]
        ReadCSV["read_csv()\nnumpy vectorized"]
        ReadPLY["read_ply()\nplyfile + validation"]
        ReadBag["read_bag()\nlazy import, requires ROS"]
    end

    subgraph fp_detail ["filter_path()"]
        DS["filter_downsample()"]
        MD["filter_by_min_distance()\npreserves last point"]
    end

    subgraph tl_detail ["to_lanelet()"]
        P2L["pose2line()\nvectorized rotation matrix"]
        SS["split_segments()\ncumulative dist + searchsorted"]
        LM["LaneletMap\ncached Transformer\nsub-meter precision"]
    end

    OSM[".osm\nLanelet2"]

    CSV --> CLI
    PLY --> CLI
    MCAP --> CLI
    BAG --> CLI
    SVC --> Node

    CLI --> LP
    Node --> LP

    LP --> ReadCSV
    LP --> ReadPLY
    LP -.-> ReadBag

    ReadCSV --> FP
    ReadPLY --> FP
    ReadBag --> FP

    FP --> DS --> MD

    MD --> TL

    TL --> P2L --> SS --> LM --> OSM
```

### Component Overview

| Component | Type | Dependencies | Description |
|-----------|------|--------------|-------------|
| **Library** | Plain Python | numpy, pyproj, plyfile | `import lanelet2_generator` — works without ROS |
| **CLI** | Plain Python | library only | `python -m lanelet2_generator.cli` or `lanelet2_generator` (pip) |
| **Bag reader** | Plain Python | rclpy, rosbag2_py | Lazily imported only when reading .mcap or rosbag2 dirs |
| **ROS Node** | ROS 2 | library + autoware_adapi_v1_msgs | `/api/routing/set_route_points` service |

### Pipeline

All inputs produce an `(N, 7)` pose array `[x, y, z, qx, qy, qz, qw]` that flows through a single unified pipeline:

1. **`load_path()`** — dispatches by file extension to `read_csv()`, `read_ply()`, or `read_bag()` (lazy)
2. **`filter_path()`** — downsample (keep every Nth point), then min-distance filter. Always preserves the last point.
3. **`to_lanelet()`** — vectorized `pose2line()` computes left/right/center boundaries, `split_segments()` splits by distance or direction change using cumulative distances, `LaneletMap` builds OSM XML with a cached `pyproj.Transformer` for sub-meter MGRS-to-WGS84 precision

## Features

- **Input formats:** CSV, PLY, MCAP bag, sqlite3 rosbag2
- **Waypoint YAML:** DJI-like waypoint YAML (`.yaml`, `.yml`) via `waypoints[].position` + `heading_degree`
- **Point cloud tool:** LAS/LAZ (UTM) -> local MGRS PCD + `map_projector_info.yaml`
- **Path filtering:** Min distance, downsampling (step)
- **Lanelet splitting:** Max length, direction-change split (as in bag2lanelet)
- **ROS 2 service:** `/api/routing/set_route_points` to generate lanelet2 from route waypoints

## Installation

### Library and CLI (standalone)

```bash
pip install -e .
# For bag/MCAP support, also: pip install -e ".[ros]"
# For LAS/LAZ -> PCD tool, also: pip install -e ".[las]"
```

### ROS 2 (node + launch)

```bash
pip install -r requirements.txt
cd /path/to/vifware_ws
colcon build --packages-select lanelet2_generator
source install/setup.bash
```

## Usage

### Docker helpers

Build the image:

```bash
docker build -f docker/Dockerfile -t lanelet2-generator:latest .
```

Run point cloud conversion (LAS/LAZ -> PCD + YAML):

```bash
docker/pointcloud_converter.sh \
  --input data/input_cloud.laz \
  --output data/pointcloud_map.pcd \
  --utm-frame 32N --voxel-size 0.1 --color-by auto
```

Or use positional arguments (output directory optional):

```bash
docker/pointcloud_converter.sh data/input_cloud.laz data \
  --utm-frame 32N --voxel-size 0.1 --color-by auto
```

Run lanelet generation:

```bash
docker/lanelet2_generator.sh \
  data/mission.yaml \
  data \
  --map-projector-info data/map_projector_info.yaml --width 2.5 --speed-limit 5 \
  --output-file lanelet2_map.osm
```

Notes:

- Both scripts auto-build the Docker image if needed.
- For **point cloud**: use `--input` / `--output` or positional paths; `--output` may be a directory or a full `.pcd` path. If output is omitted, the input folder is used.
- For **lanelet**: input and output can be different folders; if output is omitted, the input folder is used.
- You can pass options directly after required args; using `--` as a separator is optional (see `docker/pointcloud_converter.sh` usage).

### CLI (plain Python, not ROS)

**Syntax:**

```bash
python3 -m lanelet2_generator.cli <input> <output_dir> [options]
# or
python3 -m lanelet2_generator.cli --input <path> <output_dir> [options]
# or
python3 -m lanelet2_generator.cli --input <path> --output <output_dir> [options]
# or, after pip install:
lanelet2_generator <input> <output_dir> [options]
```

**Examples:**

```bash
# From CSV
python3 -m lanelet2_generator.cli data/waypoints.csv data --width 3.0 --mgrs 33TWN

# From PLY
python3 -m lanelet2_generator.cli data/trajectory.ply data --width 2.5 --mgrs 33TWN

# From waypoint YAML (.yaml/.yml)
python3 -m lanelet2_generator.cli data/mission.yaml data --width 2.5 --mgrs 32UQU --speed-limit 5

# Use map_projector_info.yaml to auto-set MGRS (no --mgrs needed)
python3 -m lanelet2_generator.cli data/traj_fusion_gps.ply data \
  --map-projector-info data/map_projector_info.yaml --width 4.0 --speed-limit 20 --split-distance 100

# From MCAP bag (requires ROS env / rosbag2)
source /opt/ros/humble/setup.bash
python3 -m lanelet2_generator.cli data/recorded.mcap data --width 3.0 --mgrs 33TWN

# From rosbag2 directory (sqlite3)
python3 -m lanelet2_generator.cli /path/to/bag data --width 3.0 --mgrs 33TWN
```

**CLI parameters:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `input` | path | — | Input: CSV, PLY, YAML, MCAP file, or rosbag2 directory (positional; omit if using `--input`) |
| `--input` | path | — | Same as `input`; explicit flag for scripts and Docker. |
| `output_lanelet` | path | — | Output directory for .osm file (positional; omit if using `--output`) |
| `--output` | path | — | Same as `output_lanelet`; explicit flag for scripts and Docker. |
| `--width` | float | 2.0 | Lane width [m] |
| `--mgrs` | string | 33TWN | MGRS code |
| `--map-projector-info` | path | — | Optional `map_projector_info.yaml`; uses `mgrs_grid` from file (overrides `--mgrs`) |
| `--speed-limit` | float | 30 | Speed limit [km/h] |
| `--offset` | float float float | 0 0 0 | Offset [m] from centerline (x y z) |
| `--center` | flag | false | Add centerline to lanelet |
| `--min-distance` | float | 1.0 | Min distance [m] between consecutive points |
| `--step` | int | 1 | Downsample: keep every Nth point (CSV/PLY only) |
| `--interval` | float float | 0.1 2.0 | [Bag/MCAP only] Min and max interval [m] between tf poses |
| `--split-distance` | float | 500 | Split lanelet every M meters along path |
| `--split-direction` | float float | — | Split when direction changes more than DEG deg within M m (e.g. `80 30`) |
| `--output-file` | string | — | Exact output `.osm` filename (absolute or relative to output dir) |
| `--smooth-window` | int | 0 | Interpolating smoothing subdivisions per segment; original points stay on the path |
| `--no-bidirectional` | flag | false | Disable lanelets in both directions (enabled by default) |

### ROS 2 service node (only ROS component)

```bash
# Launch with default output path
ros2 launch lanelet2_generator route_to_lanelet.launch.xml output_path:=/tmp/lanelet_maps

# With custom params
ros2 launch lanelet2_generator route_to_lanelet.launch.xml \
  output_path:=/data/maps/lanelet \
  mgrs:=33TWN \
  width:=3.0
```

The node advertises `/api/routing/set_route_points` (`autoware_adapi_v1_msgs/srv/SetRoutePoints`). When called with goal and waypoints, it generates a lanelet2 map and saves it to the configured output path.

**Launch parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_path` | /tmp/lanelet_maps | Directory where .osm files are saved |
| `mgrs` | 33TWN | MGRS code |
| `width` | 2.0 | Lane width [m] |
| `speed_limit` | 30 | Speed limit [km/h] |
| `min_distance` | 1.0 | Min distance [m] between points |
| `step` | 1 | Downsample step |
| `split_distance` | 500 | Split every M meters |
| `split_direction_deg` | — | Split on direction change [deg] |
| `split_direction_window_m` | — | Direction change window [m] |
| `smooth_window` | 0 | Interpolating smoothing subdivisions per segment; original points stay on the path |
| `bidirectional` | true | Generate lanelets in both directions |

### LAS/LAZ to local MGRS PCD

Converts UTM LAS/LAZ point clouds to the same local MGRS frame used by this package:

- Detects (or accepts) UTM CRS
- Detects `mgrs_grid` from cloud centroid (or force with `--mgrs-grid`)
- Shifts points to local MGRS coordinates
- Optional downsampling (`--stride`, `--max-points`, `--voxel-size`)
- Writes PCD and `map_projector_info.yaml`

**Syntax:**

```bash
python3 -m lanelet2_generator.las_mgrs_cli <input.las|input.laz> [output_dir|output.pcd] [options]
# or
python3 -m lanelet2_generator.las_mgrs_cli --input <path> [--output <dir_or.pcd>] [options]
```

If `output` is omitted, files are written to the input file’s directory. If `output` is a path ending in `.pcd`, that path sets both the directory and the PCD filename (`--pcd-name` is ignored in that case).

**Examples:**

```bash
# Explicit EPSG
python3 -m lanelet2_generator.las_mgrs_cli data/input_cloud.laz data \
  --pcd-name pointcloud_map.pcd --epsg 32632 --voxel-size 0.1 --color-by rgb

# UTM frame notation (instead of EPSG)
python3 -m lanelet2_generator.las_mgrs_cli data/input_cloud.laz data \
  --utm-frame 32N --voxel-size 0.1 --color-by auto

# Explicit .pcd path (same as Docker example above)
python3 -m lanelet2_generator.las_mgrs_cli --input data/merged_clean.las --output data/pointcloud_map.pcd \
  --utm-frame 32N --voxel-size 0.1 --color-by auto
```

**Important notes:**

- `.laz` requires a laspy backend (e.g. `lazrs`): `pip install lazrs`
- Use `--utm-frame` (e.g. `32N`) or `--epsg` if LAS header CRS is missing/wrong
- Output is either a **directory** (use `--pcd-name` for the filename) or a **`.pcd` file path** (basename becomes the PCD name)
- If the source cloud has RGB, use `--color-by rgb` (or `--color-by auto`)

**Key options:**

| Option | Default | Description |
|--------|---------|-------------|
| `--epsg` | — | CRS override by EPSG |
| `--utm-frame` | — | CRS override in UTM format, e.g. `32N`, `32S` |
| `--utm-zone` + `--south` | — | Alternative CRS override |
| `--mgrs-grid` | auto | Force 5-char MGRS grid (e.g. `32UQU`) |
| `--color-by` | `auto` | `auto`, `rgb`, `intensity`, `classification`, `none` |
| `--colormap` | `viridis` | Colormap for scalar coloring |
| `--color-percentiles` | `2 98` | Scalar clipping percentiles |
| `--voxel-size` | — | Voxel downsample size [m] |
| `--stride` | — | Keep every k-th point |
| `--max-points` | — | Random subsample limit |
| `--pcd-name` | `pointcloud_map.pcd` | Output PCD filename when output is a directory (ignored if output path ends in `.pcd`) |
| `--yaml-name` | `map_projector_info.yaml` | Output projector YAML filename |

### Python API

```python
from lanelet2_generator import load_path, filter_path, generate

# Load and generate
poses = load_path("waypoints.csv")
poses = filter_path(poses, min_distance=0.5, step=2)
path = generate(poses=poses, output_dir="./output", mgrs="33TWN", width=2.0)
```

## Input formats

| Format | Extension | Format |
|--------|-----------|--------|
| CSV | .csv | x, y, z, yaw, velocity, change_flag |
| PLY | .ply | Vertices: x, y, z, q_w, q_x, q_y, q_z |
| Waypoint YAML | .yaml, .yml | `waypoints[].position.{x,y,z}` + `orientation.heading_degree` |
| MCAP | .mcap | /tf with base_link |
| rosbag2 | directory | /tf with base_link (sqlite3) |

## Output

- `.osm` file (Lanelet2 / OSM format) saved as `YY-MM-DD-HH-MM-SS-lanelet2_map.osm`
- Compatible with Autoware and Vector Map Builder

## Limitations

- MGRS to WGS84 conversion may produce jagged lanes; post-process in Vector Map Builder for refinement.
- Requires `autoware_adapi_v1_msgs` for the route service node.


## License

Apache License 2.0
