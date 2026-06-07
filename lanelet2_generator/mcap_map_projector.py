"""Generate map YAML sidecars from GNSS NavSatFix messages in an MCAP recording."""

import json
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional, Union

import numpy as np

from lanelet2_generator.map_projector import write_map_projector_yamls
from lanelet2_generator.mgrs_utils import mgrs_to_wgs, utm_frame_from_mgrs_grid

DEFAULT_GNSS_TOPIC = "/sensing/gnss/nav_sat_fix"


@dataclass
class GnssSampleResult:
    """Collected NavSatFix samples from an MCAP file."""

    topic: str
    topic_present: Optional[bool]
    valid_fixes: int
    messages_seen: int
    latitudes: np.ndarray
    longitudes: np.ndarray
    altitudes: np.ndarray
    decoder: str = ""


def resolve_mcap_cli(mcap_cmd=None):
    """Return the mcap CLI executable path."""
    if mcap_cmd:
        path = shutil.which(str(mcap_cmd))
        if path is None:
            raise FileNotFoundError(f"MCAP CLI not found: {mcap_cmd}")
        return path
    path = shutil.which("mcap")
    if path is None:
        raise FileNotFoundError(
            "MCAP CLI not found on PATH. Install from https://mcap.dev/guides/cli "
            "or pass --mcap-cmd /path/to/mcap"
        )
    return path


def _nav_fix_status(msg: Union[dict, object]) -> int:
    if isinstance(msg, dict):
        status = msg.get("status", {})
        if isinstance(status, dict):
            return int(status.get("status", -1))
        if status is None:
            return -1
        return int(status)

    status = getattr(msg, "status", None)
    if status is None:
        return -1
    if isinstance(status, (int, float)):
        return int(status)
    return int(getattr(status, "status", -1))


def _field(msg: Union[dict, object], name: str, default=float("nan")):
    if isinstance(msg, dict):
        return msg.get(name, default)
    return getattr(msg, name, default)


def is_valid_nav_sat_fix(msg: Union[dict, object]) -> bool:
    """Return True for NavSatFix messages with a non-no-fix GNSS status."""
    if _nav_fix_status(msg) < 0:
        return False

    lat = float(_field(msg, "latitude"))
    lon = float(_field(msg, "longitude"))
    if not np.isfinite(lat) or not np.isfinite(lon):
        return False
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return False
    if lat == 0.0 and lon == 0.0:
        return False
    return True


def _topic_present_in_info(info_text: str, topic: str) -> bool:
    pattern = re.compile(r"^\s*\(\d+\)\s+" + re.escape(topic) + r"\s+", re.MULTILINE)
    return bool(pattern.search(info_text))


def _topic_schema_encoding(info_text: str, topic: str) -> Optional[str]:
    """Return schema encoding from mcap info, e.g. ros2msg or ros1msg."""
    pattern = re.compile(
        r"^\s*\(\d+\)\s+"
        + re.escape(topic)
        + r".*\[(ros1msg|ros2msg|protobuf|json|jsonschema)\]",
        re.MULTILINE,
    )
    match = pattern.search(info_text)
    return match.group(1) if match else None


def _run_mcap_info(mcap_path, mcap_cli):
    result = subprocess.run(
        [mcap_cli, "info", str(mcap_path)],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        stderr = (result.stderr or "").strip()
        raise RuntimeError(f"mcap info failed for {mcap_path}: {stderr or result.stdout}")
    return result.stdout


def _collect_gnss_samples(
    topic,
    messages: Iterator[Union[dict, object]],
    *,
    min_valid_fixes,
    max_valid_fixes,
    max_messages,
    decoder,
):
    lats = []
    lons = []
    alts = []
    messages_seen = 0

    for msg in messages:
        messages_seen += 1
        if not is_valid_nav_sat_fix(msg):
            if messages_seen >= max_messages:
                break
            continue

        lats.append(float(_field(msg, "latitude")))
        lons.append(float(_field(msg, "longitude")))
        alt = float(_field(msg, "altitude", 0.0))
        alts.append(alt if np.isfinite(alt) else 0.0)

        if len(lats) >= max(max_valid_fixes, min_valid_fixes):
            break
        if messages_seen >= max_messages:
            break

    return GnssSampleResult(
        topic=topic,
        topic_present=True,
        valid_fixes=len(lats),
        messages_seen=messages_seen,
        latitudes=np.asarray(lats, dtype=np.float64),
        longitudes=np.asarray(lons, dtype=np.float64),
        altitudes=np.asarray(alts, dtype=np.float64),
        decoder=decoder,
    )


def _sample_gnss_via_mcap_cli_json(
    mcap_path,
    mcap_cli,
    topic,
    *,
    min_valid_fixes,
    max_valid_fixes,
    max_messages,
):
    """Decode NavSatFix via ``mcap cat --json`` (ROS1 / protobuf MCAP only)."""
    proc = subprocess.Popen(
        [mcap_cli, "cat", str(mcap_path), "--topics", topic, "--json"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert proc.stdout is not None
    stderr_text = ""

    def iter_decoded():
        nonlocal stderr_text
        for line in proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            yield record.get("data", record)

    try:
        result = _collect_gnss_samples(
            topic,
            iter_decoded(),
            min_valid_fixes=min_valid_fixes,
            max_valid_fixes=max_valid_fixes,
            max_messages=max_messages,
            decoder="mcap-cli-json",
        )
    finally:
        if proc.poll() is None:
            proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        if proc.stderr is not None:
            stderr_text = proc.stderr.read()

    if proc.returncode not in (0, -15, -9) and result.valid_fixes == 0:
        err = (stderr_text or "").strip()
        if "ros2msg" in err:
            return None
        raise RuntimeError(f"mcap cat failed for {mcap_path}: {err or 'unknown error'}")
    return result


def _sample_gnss_via_ros2_reader(
    mcap_path,
    topic,
    *,
    min_valid_fixes,
    max_valid_fixes,
    max_messages,
):
    """Decode ROS2 NavSatFix using embedded MCAP schemas (no ROS install)."""
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 MCAP decoding requires mcap-ros2-support. "
            "Install with: pip install 'lanelet2_generator[mcap]'"
        ) from exc

    def iter_decoded():
        for msg in read_ros2_messages(str(mcap_path), topics=[topic]):
            yield msg.ros_msg

    return _collect_gnss_samples(
        topic,
        iter_decoded(),
        min_valid_fixes=min_valid_fixes,
        max_valid_fixes=max_valid_fixes,
        max_messages=max_messages,
        decoder="mcap-ros2-support",
    )


def sample_gnss_from_mcap(
    mcap_path,
    *,
    topic=DEFAULT_GNSS_TOPIC,
    min_valid_fixes=1,
    max_valid_fixes=500,
    max_messages=50_000,
    mcap_cmd=None,
):
    """
    Read NavSatFix messages from ``topic`` in an MCAP file.

    Uses ``mcap info`` for a fast topic check. ROS2 recordings (``ros2msg``) are
    decoded with ``mcap-ros2-support``; ROS1/protobuf use ``mcap cat --json``.
    """
    mcap_path = Path(mcap_path)
    if not mcap_path.is_file():
        raise FileNotFoundError(f"MCAP file not found: {mcap_path}")

    mcap_cli = resolve_mcap_cli(mcap_cmd)
    info_text = _run_mcap_info(mcap_path, mcap_cli)
    if not _topic_present_in_info(info_text, topic):
        return GnssSampleResult(
            topic=topic,
            topic_present=False,
            valid_fixes=0,
            messages_seen=0,
            latitudes=np.empty(0),
            longitudes=np.empty(0),
            altitudes=np.empty(0),
            decoder="none",
        )

    encoding = _topic_schema_encoding(info_text, topic)
    if encoding == "ros2msg":
        print("Using ROS2 MCAP decoder (mcap-ros2-support)")
        return _sample_gnss_via_ros2_reader(
            mcap_path,
            topic,
            min_valid_fixes=min_valid_fixes,
            max_valid_fixes=max_valid_fixes,
            max_messages=max_messages,
        )

    result = _sample_gnss_via_mcap_cli_json(
        mcap_path,
        mcap_cli,
        topic,
        min_valid_fixes=min_valid_fixes,
        max_valid_fixes=max_valid_fixes,
        max_messages=max_messages,
    )
    if result is not None:
        return result

    print("mcap cat --json unsupported; falling back to ROS2 MCAP decoder")
    return _sample_gnss_via_ros2_reader(
        mcap_path,
        topic,
        min_valid_fixes=min_valid_fixes,
        max_valid_fixes=max_valid_fixes,
        max_messages=max_messages,
    )


def georef_from_gnss_samples(samples: GnssSampleResult):
    """Derive MGRS grid, UTM frame, and map origin from GNSS samples."""
    import mgrs

    lat = float(np.median(samples.latitudes))
    lon = float(np.median(samples.longitudes))
    ele = float(np.median(samples.altitudes)) if samples.altitudes.size else 0.0

    m_impl = mgrs.MGRS()
    grid5 = m_impl.toMGRS(lat, lon)[:5]
    utm_frame = utm_frame_from_mgrs_grid(grid5)
    origin_lat, origin_lon = mgrs_to_wgs(grid5)
    return {
        "mgrs_grid": grid5,
        "utm_frame": utm_frame,
        "map_origin": (origin_lat, origin_lon, ele),
        "gnss_median": (lat, lon, ele),
    }


def generate_map_projector_from_mcap(
    output_dir,
    mcap_path,
    *,
    topic=DEFAULT_GNSS_TOPIC,
    min_valid_fixes=1,
    max_valid_fixes=500,
    max_messages=50_000,
    mcap_cmd=None,
    yaml_name="map_projector_info.yaml",
    map_config_name="map_config.yaml",
):
    """
    Write map YAML sidecars from GNSS in an MCAP, or local defaults if unavailable.

    Returns:
        (yaml_path, map_config_path, info_dict)
    """
    output_dir = Path(output_dir)
    samples = sample_gnss_from_mcap(
        mcap_path,
        topic=topic,
        min_valid_fixes=min_valid_fixes,
        max_valid_fixes=max_valid_fixes,
        max_messages=max_messages,
        mcap_cmd=mcap_cmd,
    )

    if samples.valid_fixes < min_valid_fixes:
        reason = "no valid GNSS fixes in MCAP"
        if samples.topic_present is False:
            reason = f"topic not found: {topic}"
        elif samples.messages_seen == 0:
            reason = f"no messages on topic: {topic}"
        yaml_path, map_cfg_path = write_map_projector_yamls(
            output_dir,
            local=True,
            yaml_name=yaml_name,
            map_config_name=map_config_name,
        )
        return yaml_path, map_cfg_path, {
            "mode": "local",
            "reason": reason,
            "topic": topic,
            "messages_seen": samples.messages_seen,
            "valid_fixes": samples.valid_fixes,
            "decoder": samples.decoder,
        }

    georef = georef_from_gnss_samples(samples)
    origin_lat, origin_lon, ele = georef["map_origin"]
    yaml_path, map_cfg_path = write_map_projector_yamls(
        output_dir,
        local=False,
        grid5=georef["mgrs_grid"],
        lat0=origin_lat,
        lon0=origin_lon,
        ele0=ele,
        yaml_name=yaml_name,
        map_config_name=map_config_name,
    )
    gnss_lat, gnss_lon, gnss_ele = georef["gnss_median"]
    return yaml_path, map_cfg_path, {
        "mode": "mgrs",
        "reason": "valid GNSS in MCAP",
        "topic": topic,
        "messages_seen": samples.messages_seen,
        "valid_fixes": samples.valid_fixes,
        "decoder": samples.decoder,
        "mgrs_grid": georef["mgrs_grid"],
        "utm_frame": georef["utm_frame"],
        "map_origin": georef["map_origin"],
        "gnss_median": (gnss_lat, gnss_lon, gnss_ele),
    }
