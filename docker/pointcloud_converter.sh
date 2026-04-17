#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-lanelet2-generator:latest}"

usage() {
  cat <<'EOF'
Usage:
  docker/pointcloud_converter.sh [options] [-- converter flags...]

  Positional (legacy):
    docker/pointcloud_converter.sh <input.las|input.laz> [output_dir|output.pcd] [-- ...]

  Flags:
    --input PATH    Input .las / .laz
    --output PATH   Output directory or full path to .pcd (e.g. data/pointcloud_map.pcd)

Examples:
  docker/pointcloud_converter.sh --input data/merged_clean.las --output data/pointcloud_map.pcd \
    --utm-frame 33N --voxel-size 0.1 --color-by auto
  docker/pointcloud_converter.sh data/sampled.laz data -- --utm-frame 33N --voxel-size 0.1
  docker/pointcloud_converter.sh /abs/path/sampled.laz -- --epsg 32632 --pcd-name pointcloud_map.pcd

Notes:
  - If output is omitted, the input file directory is used (same as before).
  - Everything after '--' is passed directly to lanelet2_generator.las_mgrs_cli.
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

if [[ -z "${OUTPUT_PATH}" && ${#REMAINING[@]} -gt 0 && "${REMAINING[0]}" != -* ]]; then
  OUTPUT_PATH="${REMAINING[0]}"
  REMAINING=("${REMAINING[@]:1}")
fi

if [[ "${INPUT_PATH}" != /* ]]; then
  INPUT_PATH="$(realpath "${INPUT_PATH}")"
fi
if [[ ! -f "${INPUT_PATH}" ]]; then
  echo "Input file not found: ${INPUT_PATH}" >&2
  exit 1
fi

if [[ -z "${OUTPUT_PATH}" ]]; then
  OUTPUT_DIR="$(dirname "${INPUT_PATH}")"
else
  if [[ "${OUTPUT_PATH}" != /* ]]; then
    OUTPUT_PATH="$(realpath -m "${OUTPUT_PATH}")"
  fi
  if [[ "${OUTPUT_PATH}" == *.pcd || "${OUTPUT_PATH}" == *.PCD ]]; then
    OUTPUT_DIR="$(dirname "${OUTPUT_PATH}")"
  else
    OUTPUT_DIR="${OUTPUT_PATH}"
  fi
fi
mkdir -p "${OUTPUT_DIR}"

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
    --input "/input/${INPUT_BASENAME}"
  )
  if [[ -n "${OUTPUT_PATH}" && ("${OUTPUT_PATH}" == *.pcd || "${OUTPUT_PATH}" == *.PCD) ]]; then
    DOCKER_ARGS+=(--output "/input/$(basename "${OUTPUT_PATH}")")
  else
    DOCKER_ARGS+=(--output "/input")
  fi
else
  DOCKER_ARGS=(
    --rm
    -v "${INPUT_DIR}:/input:ro"
    -v "${OUTPUT_DIR_ABS}:/output"
    -w /app
    "${IMAGE_NAME}"
    python -m lanelet2_generator.las_mgrs_cli
    --input "/input/${INPUT_BASENAME}"
  )
  if [[ -n "${OUTPUT_PATH}" && ("${OUTPUT_PATH}" == *.pcd || "${OUTPUT_PATH}" == *.PCD) ]]; then
    DOCKER_ARGS+=(--output "/output/$(basename "${OUTPUT_PATH}")")
  else
    DOCKER_ARGS+=(--output "/output")
  fi
fi

docker run "${DOCKER_ARGS[@]}" "${REMAINING[@]}"
