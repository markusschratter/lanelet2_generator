#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-lanelet2-generator:latest}"

usage() {
  cat <<'EOF'
Usage:
  docker/lanelet2_generator.sh <input> [output_dir] [-- lanelet flags...]

Examples:
  docker/lanelet2_generator.sh data_apex/erding_2.yaml output -- --mgrs 32UQU -l 2.5 -s 5
  docker/lanelet2_generator.sh /abs/path/trajectory.ply /abs/path/output -- --center --split-distance 200

Notes:
  - If output_dir is omitted, the input file directory is used.
  - Everything after '--' is passed directly to lanelet2_generator.cli.
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
if [[ ! -e "${INPUT_PATH}" ]]; then
  echo "Input not found: ${INPUT_PATH}" >&2
  exit 1
fi

OUTPUT_DIR=""
if [[ $# -gt 0 && "$1" != "--" && "$1" != -* ]]; then
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

EXTRA_ARGS=("$@")
EXTRA_VOLUMES=()
map_mount_id=0

# If --map-projector-info points outside mounted input/output dirs, mount it.
for ((i = 0; i < ${#EXTRA_ARGS[@]}; i++)); do
  arg="${EXTRA_ARGS[$i]}"
  map_path=""
  inline=0

  if [[ "${arg}" == "--map-projector-info" ]]; then
    if (( i + 1 < ${#EXTRA_ARGS[@]} )); then
      map_path="${EXTRA_ARGS[$((i + 1))]}"
    fi
  elif [[ "${arg}" == --map-projector-info=* ]]; then
    map_path="${arg#--map-projector-info=}"
    inline=1
  fi

  if [[ -z "${map_path}" || "${map_path}" != /* ]]; then
    continue
  fi
  if [[ ! -f "${map_path}" ]]; then
    continue
  fi

  map_dir="$(dirname "${map_path}")"
  map_base="$(basename "${map_path}")"
  container_map_path=""

  if [[ "${map_dir}" == "${INPUT_DIR}" ]]; then
    container_map_path="/input/${map_base}"
  elif [[ "${map_dir}" == "${OUTPUT_DIR_ABS}" ]]; then
    container_map_path="/output/${map_base}"
  else
    mount_point="/map_projector_${map_mount_id}"
    map_mount_id=$((map_mount_id + 1))
    EXTRA_VOLUMES+=("-v" "${map_dir}:${mount_point}:ro")
    container_map_path="${mount_point}/${map_base}"
  fi

  if (( inline == 1 )); then
    EXTRA_ARGS[$i]="--map-projector-info=${container_map_path}"
  else
    EXTRA_ARGS[$((i + 1))]="${container_map_path}"
  fi
done

DOCKER_ARGS=(
  --rm
  -w /app
  "${IMAGE_NAME}"
  python -m lanelet2_generator.cli
)

if [[ "${INPUT_DIR}" == "${OUTPUT_DIR_ABS}" ]]; then
  DOCKER_ARGS=(
    --rm
    -v "${INPUT_DIR}:/input"
    -w /app
    "${IMAGE_NAME}"
    python -m lanelet2_generator.cli
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
    python -m lanelet2_generator.cli
    "/input/${INPUT_BASENAME}"
    "/output"
  )
fi

docker run "${EXTRA_VOLUMES[@]}" "${DOCKER_ARGS[@]}" "${EXTRA_ARGS[@]}"
