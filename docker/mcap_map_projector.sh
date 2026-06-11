#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-lanelet2-generator:latest}"
GNSS_TOPIC="${GNSS_TOPIC:-/sensing/gnss/nav_sat_fix}"

usage() {
  cat <<'EOF'
Usage:
  docker/mcap_map_projector.sh --input PATH --output PATH [options] [-- extra flags...]

  Positional:
    docker/mcap_map_projector.sh <recording.mcap> <output_dir> [-- ...]

  Flags:
    --input PATH        Input MCAP recording
    --output PATH       Output directory for map_projector_info.yaml and map_config.yaml
    --gnss-topic TOPIC  NavSatFix topic (default: /sensing/gnss/nav_sat_fix)
                        Override via env GNSS_TOPIC

  Uses mcap info for topic discovery. ROS2 bags (ros2msg) are decoded with
  mcap-ros2-support (no ROS install). ROS1 bags use mcap cat --json.

Examples:
  docker/mcap_map_projector.sh --input data/recording.mcap --output data/map
  docker/mcap_map_projector.sh --input data/rec.mcap --output data/map \
    --gnss-topic /gps/fix
  GNSS_TOPIC=/gps/fix docker/mcap_map_projector.sh data/rec.mcap data/map
  docker/mcap_map_projector.sh data/rec.mcap data/map -- --min-fixes 10
EOF
}

INPUT_PATH=""
OUTPUT_PATH=""
REMAINING=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --input)
      if [[ $# -lt 2 ]]; then echo "--input requires a path" >&2; exit 1; fi
      INPUT_PATH="$2"
      shift 2
      ;;
    --output)
      if [[ $# -lt 2 ]]; then echo "--output requires a path" >&2; exit 1; fi
      OUTPUT_PATH="$2"
      shift 2
      ;;
    --gnss-topic|--topic)
      if [[ $# -lt 2 ]]; then echo "$1 requires a topic name" >&2; exit 1; fi
      GNSS_TOPIC="$2"
      shift 2
      ;;
    --)
      shift
      REMAINING+=("$@")
      break
      ;;
    *)
      REMAINING+=("$1")
      shift
      ;;
  esac
done

if [[ -z "${INPUT_PATH}" ]]; then
  if [[ ${#REMAINING[@]} -eq 0 ]]; then
    usage
    exit 1
  fi
  INPUT_PATH="${REMAINING[0]}"
  REMAINING=("${REMAINING[@]:1}")
fi

if [[ -z "${OUTPUT_PATH}" ]]; then
  if [[ ${#REMAINING[@]} -eq 0 || "${REMAINING[0]}" == -* ]]; then
    usage
    exit 1
  fi
  OUTPUT_PATH="${REMAINING[0]}"
  REMAINING=("${REMAINING[@]:1}")
fi

if [[ "${INPUT_PATH}" != /* ]]; then
  INPUT_PATH="$(realpath "${INPUT_PATH}")"
fi
if [[ ! -f "${INPUT_PATH}" ]]; then
  echo "MCAP file not found: ${INPUT_PATH}" >&2
  exit 1
fi

if [[ "${OUTPUT_PATH}" != /* ]]; then
  OUTPUT_PATH="$(realpath -m "${OUTPUT_PATH}")"
fi
mkdir -p "${OUTPUT_PATH}"

INPUT_DIR="$(dirname "${INPUT_PATH}")"
INPUT_BASE="$(basename "${INPUT_PATH}")"

docker build -f "${REPO_ROOT}/docker/Dockerfile" -t "${IMAGE_NAME}" "${REPO_ROOT}"

docker run --rm \
  -v "${INPUT_DIR}:/input:ro" \
  -v "${OUTPUT_PATH}:/output" \
  -w /app \
  "${IMAGE_NAME}" \
  python -m lanelet2_generator.mcap_map_projector_cli \
  "/input/${INPUT_BASE}" \
  --output "/output" \
  --gnss-topic "${GNSS_TOPIC}" \
  "${REMAINING[@]}"
