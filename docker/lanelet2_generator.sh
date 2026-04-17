#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-lanelet2-generator:latest}"

usage() {
  cat <<'EOF'
Usage:
  docker/lanelet2_generator.sh [options] [-- lanelet flags...]

  Positional:
    docker/lanelet2_generator.sh <input> [output_dir|output.osm] [-- ...]

  Flags:
    --input PATH    Input: CSV, PLY, YAML, MCAP, or rosbag2 directory
    --output PATH   Output directory, or full path to .osm (sets directory + --output-file)

Examples:
  docker/lanelet2_generator.sh --input data/mission.yaml --output data/out -- --mgrs 32UQU --width 2.5
  docker/lanelet2_generator.sh --input traj.ply --output lanelet2_map.osm --map-projector-info map_projector_info.yaml

Notes:
  - If output is omitted, the input file directory is used.
  - Everything after '--' is passed directly to lanelet2_generator.cli.
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
if [[ ! -e "${INPUT_PATH}" ]]; then
  echo "Input not found: ${INPUT_PATH}" >&2
  exit 1
fi

if [[ -z "${OUTPUT_PATH}" ]]; then
  OUTPUT_DIR="$(dirname "${INPUT_PATH}")"
else
  if [[ "${OUTPUT_PATH}" != /* ]]; then
    OUTPUT_PATH="$(realpath -m "${OUTPUT_PATH}")"
  fi
  if [[ "${OUTPUT_PATH}" == *.osm || "${OUTPUT_PATH}" == *.OSM ]]; then
    OUTPUT_DIR="$(dirname "${OUTPUT_PATH}")"
  else
    OUTPUT_DIR="${OUTPUT_PATH}"
  fi
fi
mkdir -p "${OUTPUT_DIR}"

INPUT_DIR="$(dirname "${INPUT_PATH}")"
OUTPUT_DIR_ABS="${OUTPUT_DIR}"
INPUT_BASENAME="$(basename "${INPUT_PATH}")"

EXTRA_ARGS=("${REMAINING[@]}")

# If --output was a .osm path, pass it as --output-file for the CLI (host path; rewritten below).
has_output_file=0
for ((i = 0; i < ${#EXTRA_ARGS[@]}; i++)); do
  if [[ "${EXTRA_ARGS[$i]}" == "--output-file" || "${EXTRA_ARGS[$i]}" == --output-file=* ]]; then
    has_output_file=1
    break
  fi
done
if [[ -n "${OUTPUT_PATH}" && ("${OUTPUT_PATH}" == *.osm || "${OUTPUT_PATH}" == *.OSM) && "${has_output_file}" -eq 0 ]]; then
  EXTRA_ARGS=("--output-file" "${OUTPUT_PATH}" "${EXTRA_ARGS[@]}")
fi

docker build -f "${REPO_ROOT}/docker/Dockerfile" -t "${IMAGE_NAME}" "${REPO_ROOT}"

EXTRA_VOLUMES=()
map_mount_id=0
output_mount_id=0

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

# If --output-file is an absolute host path, map it into container.
for ((i = 0; i < ${#EXTRA_ARGS[@]}; i++)); do
  arg="${EXTRA_ARGS[$i]}"
  out_path=""
  inline=0

  if [[ "${arg}" == "--output-file" ]]; then
    if (( i + 1 < ${#EXTRA_ARGS[@]} )); then
      out_path="${EXTRA_ARGS[$((i + 1))]}"
    fi
  elif [[ "${arg}" == --output-file=* ]]; then
    out_path="${arg#--output-file=}"
    inline=1
  fi

  if [[ -z "${out_path}" || "${out_path}" != /* ]]; then
    continue
  fi

  out_base="$(basename "${out_path}")"
  container_out_path=""

  # Same host dir for input and output: only /input exists in the container.
  if [[ "${INPUT_DIR}" == "${OUTPUT_DIR_ABS}" && ( "${out_path}" == "${INPUT_DIR}"/* || "${out_path}" == "${INPUT_DIR}" ) ]]; then
    rel="${out_path#${INPUT_DIR}/}"
    rel="${rel#./}"
    if [[ -z "${rel}" || "${rel}" == "${INPUT_DIR}" ]]; then
      rel="${out_base}"
    fi
    container_out_path="/input/${rel}"
  elif [[ "${out_path}" == "${OUTPUT_DIR_ABS}"/* || "${out_path}" == "${OUTPUT_DIR_ABS}" ]]; then
    rel="${out_path#${OUTPUT_DIR_ABS}/}"
    rel="${rel#./}"
    if [[ -z "${rel}" || "${rel}" == "${OUTPUT_DIR_ABS}" ]]; then
      rel="${out_base}"
    fi
    container_out_path="/output/${rel}"
  else
    out_dir="$(dirname "${out_path}")"
    mount_point="/explicit_output_${output_mount_id}"
    output_mount_id=$((output_mount_id + 1))
    EXTRA_VOLUMES+=("-v" "${out_dir}:${mount_point}")
    container_out_path="${mount_point}/${out_base}"
  fi

  if (( inline == 1 )); then
    EXTRA_ARGS[$i]="--output-file=${container_out_path}"
  else
    EXTRA_ARGS[$((i + 1))]="${container_out_path}"
  fi
done

CONTAINER_INPUT="/input/${INPUT_BASENAME}"
if [[ "${INPUT_DIR}" == "${OUTPUT_DIR_ABS}" ]]; then
  CONTAINER_OUTPUT="/input"
else
  CONTAINER_OUTPUT="/output"
fi

if [[ "${INPUT_DIR}" == "${OUTPUT_DIR_ABS}" ]]; then
  DOCKER_ARGS=(
    --rm
    -v "${INPUT_DIR}:/input"
    -w /app
    "${IMAGE_NAME}"
    python -m lanelet2_generator.cli
    --input "${CONTAINER_INPUT}"
    --output "${CONTAINER_OUTPUT}"
  )
else
  DOCKER_ARGS=(
    --rm
    -v "${INPUT_DIR}:/input:ro"
    -v "${OUTPUT_DIR_ABS}:/output"
    -w /app
    "${IMAGE_NAME}"
    python -m lanelet2_generator.cli
    --input "${CONTAINER_INPUT}"
    --output "${CONTAINER_OUTPUT}"
  )
fi

docker run "${EXTRA_VOLUMES[@]}" "${DOCKER_ARGS[@]}" "${EXTRA_ARGS[@]}"
