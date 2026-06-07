#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-lanelet2-generator:latest}"

usage() {
  cat <<'EOF'
Usage:
  docker/map_projector.sh [options] [-- extra flags...]

  Positional:
    docker/map_projector.sh <output_dir> [-- ...]

  Flags:
    --output PATH   Output directory for map_projector_info.yaml and map_config.yaml

  Default: local map YAML (same as pointcloud converter without --utm-frame).

Examples:
  docker/map_projector.sh --output data/map
  docker/map_projector.sh --output data/map -- --utm-frame 32N --mgrs-grid 32TNT
  docker/map_projector.sh data/map -- --mgrs-grid 33TWN --elevation 520
EOF
}

OUTPUT_PATH=""
REMAINING=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      if [[ $# -lt 2 ]]; then echo "--output requires a path" >&2; exit 1; fi
      OUTPUT_PATH="$2"
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

if [[ -z "${OUTPUT_PATH}" ]]; then
  if [[ ${#REMAINING[@]} -eq 0 ]]; then
    usage
    exit 1
  fi
  OUTPUT_PATH="${REMAINING[0]}"
  REMAINING=("${REMAINING[@]:1}")
fi

if [[ "${OUTPUT_PATH}" != /* ]]; then
  OUTPUT_PATH="$(realpath -m "${OUTPUT_PATH}")"
fi
mkdir -p "${OUTPUT_PATH}"

docker build -f "${REPO_ROOT}/docker/Dockerfile" -t "${IMAGE_NAME}" "${REPO_ROOT}"

docker run --rm \
  -v "${OUTPUT_PATH}:/output" \
  -w /app \
  "${IMAGE_NAME}" \
  python -m lanelet2_generator.map_projector_cli \
  --output "/output" \
  "${REMAINING[@]}"
