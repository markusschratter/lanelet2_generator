# Functional Specification Document (FSD): lanelet2_generator

Version: 0.2.0  
Last updated: 2026-03-08

---

## 1. Purpose

This document defines the functional requirements for `lanelet2_generator` and serves as a reference for future changes. It captures the intended behavior and constraints of the package.

**Origin:** This package is based on [bag2lanelet](https://github.com/autowarefoundation/autoware_tools/tree/main/bag2lanelet) by the Autoware Foundation.

---

## 2. High-Level Requirements

### 2.1 Library

- **R1:** The package shall expose a Python library usable via `import lanelet2_generator`.
- **R1a:** The library shall be importable and usable for CSV/PLY inputs without ROS installed. ROS-dependent readers (bag) shall be lazily imported.
- **R2:** The library shall provide separate modules for: readers, filtering, geometry, and lanelet export.
- **R2a:** The library shall use only `numpy`, `pyproj`, and `plyfile` as core dependencies. No `tf_transformations` or `transforms3d`; quaternion math is implemented via vectorized numpy.
- **R3:** All I/O, filtering, and export shall be composable and independently usable.

### 2.2 Input Formats

- **R4:** Support CSV input with columns: `x, y, z, yaw, velocity, change_flag`.
- **R5:** Support PLY input with vertex properties: `x, y, z, q_w, q_x, q_y, q_z`.
- **R6:** Support MCAP rosbag2 (.mcap) with `/tf` base_link transforms.
- **R7:** Support sqlite3 rosbag2 (directory) with `/tf` base_link transforms.
- **R8:** Provide a unified `load_path(path)` that dispatches by file extension or directory.

### 2.3 Path Filtering

- **R9:** Support `min_distance`: keep only poses at least M meters apart.
- **R9a:** Filtering shall always preserve the first and last point of the path.
- **R10:** Support `step`: downsample by keeping every Nth point.
- **R11:** Provide `filter_path(poses, min_distance=..., step=...)` as the main entry point.
- **R11a:** `generate()` shall apply filtering uniformly to all input formats (CSV, PLY, bag). No format-specific hardcoded decimation.
- **R12:** Filtering shall produce a reduced pose array; all filters shall be optional.

### 2.4 Lanelet Splitting

- **R13:** Support `split_distance`: split lanelet every M meters along the path.
- **R14:** Support `split_direction`: split when direction changes more than DEG degrees within WINDOW_M meters.
- **R15:** Segments shall share boundary nodes for routing connectivity (as in bag2lanelet).
- **R16:** Splitting logic shall match the behavior of the original bag2lanelet package.

### 2.5 Lanelet Export

- **R17:** Output OSM XML compatible with Lanelet2 and Autoware.
- **R18:** Support MGRS coordinate system with configurable code.
- **R18a:** Coordinate conversion shall use sub-meter precision (float UTM offsets), not integer-truncated MGRS strings.
- **R18b:** The CRS transformer shall be created once per map, not per node.
- **R19:** Support lane width, offset, speed limit, and optional centerline.
- **R20:** Output filename format: `YY-MM-DD-HH-MM-SS-lanelet2_map.osm`.

### 2.6 ROS 2 Integration

- **R21:** Provide a node that advertises `/api/routing/set_route_points` (SetRoutePoints).
- **R22:** Service request shall include `goal` (Pose) and `waypoints` (Pose[]).
- **R23:** On service call: convert route points to poses, run pipeline, save to output path.
- **R24:** Output path and other generation parameters shall be configurable via launch file.
- **R25:** Use `autoware_adapi_v1_msgs/srv/SetRoutePoints` for service type.

### 2.7 CLI

- **R26:** Provide a CLI script with the same options as bag2lanelet.
- **R27:** CLI shall accept input path and output directory as positional arguments.
- **R28:** Support all filter and split options via command-line flags.

---

## 3. Requirements for Later Changes

When extending or modifying the package, the following requirements should be considered:

### 3.1 Backward Compatibility

- **C1:** Changes to the public API (`load_path`, `filter_path`, `generate`, `to_lanelet`) shall preserve existing function signatures or provide deprecation periods.
- **C2:** Changes to CLI arguments shall maintain default values where possible.
- **C3:** Service interface (`/api/routing/set_route_points`) shall remain compatible with Autoware clients.

### 3.2 Extensibility

- **E1:** New input formats shall be added via new readers and registered in `load_path`.
- **E2:** New filters shall be added to the `filtering` module and composed in `filter_path`.
- **E3:** Alternative export formats (e.g., GeoJSON) may be added without changing the core pipeline.

### 3.3 Performance

- **P1:** Readers shall stream or batch large files where possible; avoid loading entire datasets into memory unnecessarily.
- **P2:** Filtering and splitting shall operate on numpy arrays; avoid per-point Python loops where vectorization is possible.
- **P3:** Geometry operations (`pose2line`, yaw extraction) shall use vectorized numpy, not per-point loops.
- **P4:** Segment splitting shall use precomputed cumulative distances for O(n) complexity.

### 3.4 Testing

- **T1:** Unit tests shall cover readers, filtering, geometry, and lanelet builder independently.
- **T2:** Integration tests shall verify CLI and service node with sample data.
- **T3:** Regression tests shall ensure lanelet splitting matches bag2lanelet behavior.

### 3.5 Documentation

- **D1:** Public functions shall have docstrings with Args, Returns, and Raises.
- **D2:** README shall include usage examples for CLI, service node, and Python API.
- **D3:** This FSD shall be updated when new features or requirements are introduced.

### 3.6 Dependencies

- **D4:** Python dependencies shall be listed in `requirements.txt` with version pins where stability is critical.
- **D5:** ROS 2 dependencies shall be declared in `package.xml`; optional dependencies (e.g., `autoware_adapi_v1_msgs`) shall be documented.
- **D6:** The core library shall not depend on ROS packages (`tf_transformations`, `rclpy`). ROS-dependent modules shall use deferred imports.

### 3.7 Input Validation

- **V1:** Readers shall validate input format and raise `ValueError` with clear messages when required columns or properties are missing.
- **V2:** `generate()` shall validate required parameters (`output_dir`) before performing any I/O.

---

## 4. Out of Scope (Current Version)

- Multi-lane or bidirectional lanelet generation
- Integration with live Autoware routing (this package generates maps; it does not replace the routing node)
- GUI or visualization tools
- Southern hemisphere MGRS (parser not validated for southern bands)

---

## 5. Revision History

| Version | Date       | Changes                          |
|---------|------------|----------------------------------|
| 0.1.0   | 2026-03-06 | Initial FSD for lanelet2_generator |
| 0.2.0   | 2026-03-08 | Standalone Python support (lazy ROS imports, pure numpy quaternion math). Vectorized geometry and O(n) splitting. Sub-meter coordinate precision with cached transformer. Uniform filter pipeline for all inputs. Input validation. Service topic fix. Removed tf_transformations/transforms3d dependency. Added requirements R1a, R2a, R9a, R11a, R18a, R18b, P3, P4, D6, V1, V2. |
