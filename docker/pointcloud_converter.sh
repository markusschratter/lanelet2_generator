#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-lanelet2-generator:latest}"

usage() {
  cat <<'EOF'
Usage:
  docker/pointcloud_converter.sh <input.las|input.laz> [output_dir] [-- converter flags...]

Examples:
  docker/pointcloud_converter.sh data_hyfar/sampled.laz data_hyfar -- --utm-frame 32N --voxel-size 0.1 --color-by auto
  docker/pointcloud_converter.sh /abs/path/sampled.laz -- --epsg 32632 --pcd-name pointcloud_map.pcd

Notes:
  - If output_dir is omitted, the input file directory is used.
  - Everything after '--' is passed directly to lanelet2_generator.las_mgrs_cli.
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

INPUT_PATH="$1"
shift

if [[ "${INPUT_PATH}" != /* ]]; then
  INPUT_PATH="$(realpath "${INPUT_PATH}")"
fi
if [[ ! -f "${INPUT_PATH}" ]]; then
  echo "Input file not found: ${INPUT_PATH}" >&2
  exit 1
fi

OUTPUT_DIR=""
if [[ $# -gt 0 && "$1" != "--" ]]; then
  OUTPUT_DIR="$1"
  shift
  if [[ "${OUTPUT_DIR}" != /* ]]; then
    OUTPUT_DIR="$(realpath -m "${OUTPUT_DIR}")"
  fi
else
  OUTPUT_DIR="$(dirname "${INPUT_PATH}")"
fi
mkdir -p "${OUTPUT_DIR}"

if [[ $# -gt 0 && "$1" == "--" ]]; then
  shift
fi

INPUT_DIR="$(dirname "${INPUT_PATH}")"
OUTPUT_DIR_ABS="${OUTPUT_DIR}"
INPUT_BASENAME="$(basename "${INPUT_PATH}")"

docker build -f "${REPO_ROOT}/docker/Dockerfile" -t "${IMAGE_NAME}" "${REPO_ROOT}"

DOCKER_ARGS=(
  --rm
  -w /app
  "${IMAGE_NAME}"
  python -m lanelet2_generator.las_mgrs_cli
)

if [[ "${INPUT_DIR}" == "${OUTPUT_DIR_ABS}" ]]; then
  DOCKER_ARGS=(
    --rm
    -v "${INPUT_DIR}:/input"
    -w /app
    "${IMAGE_NAME}"
    python -m lanelet2_generator.las_mgrs_cli
    "/input/${INPUT_BASENAME}"
    "/input"
  )
else
  DOCKER_ARGS=(
    --rm
    -v "${INPUT_DIR}:/input:ro"
    -v "${OUTPUT_DIR_ABS}:/output"
    -w /app
    "${IMAGE_NAME}"
    python -m lanelet2_generator.las_mgrs_cli
    "/input/${INPUT_BASENAME}"
    "/output"
  )
fi

docker run "${DOCKER_ARGS[@]}" "$@"
