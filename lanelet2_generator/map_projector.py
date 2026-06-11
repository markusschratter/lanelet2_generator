"""Generate map_projector_info.yaml and map_config.yaml (local or MGRS georef)."""

from pathlib import Path

import yaml

from lanelet2_generator.mgrs_utils import mgrs_to_wgs


def _has_georef_flags(*, mgrs_grid=None, utm_frame=None, epsg=None, utm_zone=None, south=False):
    return any(
        value is not None and value is not False
        for value in (mgrs_grid, utm_frame, epsg, utm_zone, south)
    )


def write_map_projector_yamls(
    output_dir,
    *,
    local=False,
    grid5=None,
    lat0=0.0,
    lon0=0.0,
    ele0=0.0,
    yaml_name="map_projector_info.yaml",
    map_config_name="map_config.yaml",
):
    """Write map_projector_info.yaml and map_config.yaml to output_dir."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    yaml_path = output_dir / yaml_name
    if local:
        yaml_path.write_text("projector_type: Local\n", encoding="utf-8")
    else:
        doc = {
            "projector_type": "MGRS",
            "vertical_datum": "WGS84",
            "mgrs_grid": str(grid5),
        }
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    lat0 = round(float(lat0), 10)
    lon0 = round(float(lon0), 11)
    map_cfg = {
        "/**": {
            "ros__parameters": {
                "map_origin": {
                    "latitude": lat0,
                    "longitude": lon0,
                    "elevation": float(ele0),
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                }
            }
        }
    }
    map_cfg_path = output_dir / map_config_name
    with open(map_cfg_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(map_cfg, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    return yaml_path, map_cfg_path


def generate_map_projector_files(
    output_dir,
    *,
    local_frame=False,
    mgrs_grid=None,
    utm_frame=None,
    epsg=None,
    utm_zone=None,
    south=False,
    elevation=0.0,
    yaml_name="map_projector_info.yaml",
    map_config_name="map_config.yaml",
):
    """
    Generate map YAML sidecars using the same local/georef rules as the pointcloud converter.

    Default is local (``projector_type: Local``). Pass ``--mgrs-grid`` for georef YAML
    (``--utm-frame`` is accepted for parity with the converter but is not stored in YAML).
    """
    output_dir = Path(output_dir)
    georef_requested = _has_georef_flags(
        mgrs_grid=mgrs_grid,
        utm_frame=utm_frame,
        epsg=epsg,
        utm_zone=utm_zone,
        south=south,
    )

    if local_frame or not georef_requested:
        yaml_path, map_cfg_path = write_map_projector_yamls(
            output_dir,
            local=True,
            yaml_name=yaml_name,
            map_config_name=map_config_name,
        )
        return yaml_path, map_cfg_path, {
            "mode": "local",
            "reason": "default (pass --mgrs-grid for georef)",
        }

    if not mgrs_grid:
        raise ValueError(
            "Georef YAML requires --mgrs-grid (e.g. 32TNT). "
            "The pointcloud converter can auto-detect the grid from LAS centroids; "
            "this tool only writes YAML and has no point cloud input."
        )

    grid5 = str(mgrs_grid).strip()[:5]
    lat0, lon0 = mgrs_to_wgs(grid5)
    ele0 = float(elevation)

    yaml_path, map_cfg_path = write_map_projector_yamls(
        output_dir,
        local=False,
        grid5=grid5,
        lat0=lat0,
        lon0=lon0,
        ele0=ele0,
        yaml_name=yaml_name,
        map_config_name=map_config_name,
    )
    return yaml_path, map_cfg_path, {
        "mode": "mgrs",
        "mgrs_grid": grid5,
        "map_origin": (lat0, lon0, ele0),
        "utm_frame": utm_frame,
    }
