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
    docker/pointcloud_converter.sh <input.las|input.laz|input.pcd> [output_dir|output.pcd] [-- ...]

  Flags:
    --input PATH                 Input .las / .laz / .pcd, or directory of .las/.laz files
    --output PATH                Output directory or full path to .pcd
    --map-projector-info PATH    map_projector_info.yaml from step 1 (mcap_map_projector)

Examples:
  docker/pointcloud_converter.sh --input data/3_merge/ --output data/map/pointcloud_map.pcd \
    --map-projector-info data/map/map_projector_info.yaml --voxel-size 0.2 --color-by auto

Notes:
  - Map YAML is read from step 1; this tool only writes the PCD.
  - Export PCD_PATH to auto-set --output and --map-projector-info when flags are omitted.
  - Everything after '--' is passed to lanelet2_generator.las_mgrs_cli.
EOF
}

_has_flag() {
  local name="$1"
  shift
  local arg
  for arg in "$@"; do
    if [[ "$arg" == "$name" ]]; then
      return 0
    fi
  done
  return 1
}

_strip_path() {
  local p="${1//$'\r'/}"
  p="${p#"${p%%[![:space:]]*}"}"
  p="${p%"${p##*[![:space:]]}"}"
  while [[ "$p" == *\\ ]]; do
    p="${p%\\}"
    p="${p%"${p##*[![:space:]]}"}"
  done
  printf '%s' "$p"
}

_path_exists() {
  local p="${1:-}"
  [[ -n "$p" ]] || return 1
  [[ -e "$p" || -e "${p%/}" ]]
}

_count_las() {
  local dir="$1"
  find "$dir" -maxdepth 1 \( -iname '*.las' -o -iname '*.laz' \) 2>/dev/null | wc -l | tr -d ' '
}

_resolve_las_input() {
  local path="$(_strip_path "$1")"
  path="${path%/}"

  if [[ -d "$path" ]]; then
    if [[ "$(_count_las "$path")" -eq 0 && -d "$path/3_merge" ]]; then
      if [[ "$(_count_las "$path/3_merge")" -gt 0 ]]; then
        echo "Note: using ${path}/3_merge for LAS input (no .las/.laz in ${path})" >&2
        path="${path}/3_merge"
      fi
    fi
  fi
  printf '%s' "$path"
}

INPUT_PATH=""
OUTPUT_PATH=""
MAP_PROJECTOR_INFO=""
CONVERTER_ARGS=()
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
    --map-projector-info)
      if [[ $# -lt 2 ]]; then echo "--map-projector-info requires a path" >&2; exit 1; fi
      MAP_PROJECTOR_INFO="$2"
      shift 2
      ;;
    --voxel-size|--color-by|--tile-pcd-dir|--stride|--max-points|--pcd-name)
      if [[ $# -lt 2 ]]; then echo "$1 requires a value" >&2; exit 1; fi
      val="$(_strip_path "$2")"
      if [[ "$1" == "--tile-pcd-dir" && -n "$val" && "$val" == /* ]]; then
        val="$(basename "$val")"
        echo "Note: --tile-pcd-dir uses subfolder name '${val}' under --output" >&2
      fi
      CONVERTER_ARGS+=("$1" "$val")
      shift 2
      ;;
    --ascii-pcd|--local-frame|--subtract-xy-from-mgrs|--swap-xy|--south)
      CONVERTER_ARGS+=("$1")
      shift
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

_resolve_output_path() {
  if [[ -n "${OUTPUT_PATH}" ]]; then
    printf '%s' "${OUTPUT_PATH}"
    return 0
  fi
  if [[ -n "${PCD_OUTPUT:-}" ]]; then
    echo "Note: using PCD_OUTPUT=${PCD_OUTPUT}" >&2
    printf '%s' "$(_strip_path "${PCD_OUTPUT}")"
    return 0
  fi
  if [[ -n "${PCD_PATH:-}" ]]; then
    echo "Note: using PCD_PATH -> ${PCD_PATH%/}/pointcloud_map.pcd" >&2
    printf '%s' "${PCD_PATH%/}/pointcloud_map.pcd"
    return 0
  fi
  local map="${MAP_PROJECTOR_INFO}"
  if [[ -n "${map}" && -f "${map}" ]]; then
    echo "Note: --output inferred from map_projector_info -> $(dirname "${map}")/pointcloud_map.pcd" >&2
    printf '%s' "$(dirname "${map}")/pointcloud_map.pcd"
    return 0
  fi
  return 1
}

INPUT_PATH="$(_strip_path "${INPUT_PATH}")"
OUTPUT_PATH="$(_strip_path "${OUTPUT_PATH}")"
MAP_PROJECTOR_INFO="$(_strip_path "${MAP_PROJECTOR_INFO}")"

if [[ -z "${MAP_PROJECTOR_INFO}" && -n "${PCD_PATH:-}" && -f "${PCD_PATH%/}/map_projector_info.yaml" ]]; then
  MAP_PROJECTOR_INFO="${PCD_PATH%/}/map_projector_info.yaml"
  echo "Note: using ${MAP_PROJECTOR_INFO}" >&2
fi
if [[ -n "${MAP_PROJECTOR_INFO}" && "${MAP_PROJECTOR_INFO}" != /* ]] && _path_exists "${MAP_PROJECTOR_INFO}"; then
  MAP_PROJECTOR_INFO="$(realpath "${MAP_PROJECTOR_INFO}")"
fi
if resolved="$(_resolve_output_path)"; then
  OUTPUT_PATH="${resolved}"
fi
if [[ -z "${INPUT_PATH}" && -n "${LAS_INPUT:-}" ]]; then
  INPUT_PATH="$(_strip_path "${LAS_INPUT}")"
  echo "Note: using LAS_INPUT=${INPUT_PATH}" >&2
elif [[ -z "${INPUT_PATH}" ]] && _path_exists "$(pwd)/3_merge"; then
  INPUT_PATH="$(cd "$(pwd)/3_merge" && pwd)"
  echo "Note: using ${INPUT_PATH} for --input" >&2
fi

# Never forward host --output/--tile-pcd-dir into the container; docker owns those paths.
FILTERED_REMAINING=()
skip_next=false
for arg in "${REMAINING[@]}"; do
  if $skip_next; then
    skip_next=false
    continue
  fi
  case "$arg" in
    --output|--tile-pcd-dir)
      skip_next=true
      ;;
    *)
      FILTERED_REMAINING+=("$arg")
      ;;
  esac
done
REMAINING=("${FILTERED_REMAINING[@]}")

if [[ "${INPUT_PATH}" != /* ]]; then
  if _path_exists "${INPUT_PATH}"; then
    INPUT_PATH="$(realpath "${INPUT_PATH}")"
  fi
fi
INPUT_PATH="$(_resolve_las_input "${INPUT_PATH}")"

if ! _path_exists "${INPUT_PATH}"; then
  echo "Input not found: ${INPUT_PATH}" >&2
  echo "  cwd: $(pwd)" >&2
  if [[ -d "${INPUT_PATH}/3_merge" ]]; then
    echo "  hint: pass --input \"${INPUT_PATH}/3_merge/\"" >&2
  fi
  exit 1
fi
if [[ -d "${INPUT_PATH}" ]]; then
  INPUT_PATH="$(cd "${INPUT_PATH}" && pwd)"
elif [[ -f "${INPUT_PATH}" ]]; then
  INPUT_PATH="$(realpath "${INPUT_PATH}")"
fi

if [[ -d "${INPUT_PATH}" ]]; then
  INPUT_DIR="${INPUT_PATH}"
  CONTAINER_INPUT="/input"
elif [[ -f "${INPUT_PATH}" ]]; then
  INPUT_DIR="$(dirname "${INPUT_PATH}")"
  CONTAINER_INPUT="/input/$(basename "${INPUT_PATH}")"
else
  echo "Input must be a file or directory: ${INPUT_PATH}" >&2
  exit 1
fi

DEFAULT_VOXEL_SIZE="${VOXEL_SIZE:-0.2}"

if [[ -d "${INPUT_PATH}" ]]; then
  if [[ -z "${OUTPUT_PATH}" ]]; then
    echo "Error: --output is required for folder input (e.g. \"\$PCD_PATH/pointcloud_map.pcd\")" >&2
    echo "  Or pass --map-projector-info (output dir is inferred from the YAML path)." >&2
    echo "  Or export PCD_PATH=/path/to/map before calling this script." >&2
    echo "  Shell tip: '\\' must be the last character on a continued line — no trailing spaces." >&2
    exit 1
  fi
  if ! _has_flag --voxel-size "${CONVERTER_ARGS[@]}" "${REMAINING[@]}"; then
    if [[ "${ALLOW_NO_VOXEL:-}" == "1" ]]; then
      echo "WARNING: no --voxel-size; large LAS folders may run out of memory" >&2
    else
      CONVERTER_ARGS+=(--voxel-size "${DEFAULT_VOXEL_SIZE}")
      echo "Note: using default --voxel-size ${DEFAULT_VOXEL_SIZE} for folder input (set VOXEL_SIZE or pass --voxel-size to override)" >&2
    fi
  fi
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

OUTPUT_DIR_ABS="${OUTPUT_DIR}"

MAP_MOUNT=()
MAP_ARGS=()
if [[ -n "${MAP_PROJECTOR_INFO}" ]]; then
  if [[ "${MAP_PROJECTOR_INFO}" != /* ]]; then
    MAP_PROJECTOR_INFO="$(realpath "${MAP_PROJECTOR_INFO}")"
  fi
  if [[ ! -f "${MAP_PROJECTOR_INFO}" ]]; then
    echo "map_projector_info not found: ${MAP_PROJECTOR_INFO}" >&2
    exit 1
  fi
  MAP_DIR="$(dirname "${MAP_PROJECTOR_INFO}")"
  MAP_BASE="$(basename "${MAP_PROJECTOR_INFO}")"
  MAP_MOUNT=(-v "${MAP_DIR}:/mapinfo:ro")
  MAP_ARGS=(--map-projector-info "/mapinfo/${MAP_BASE}")
fi

docker build -f "${REPO_ROOT}/docker/Dockerfile" -t "${IMAGE_NAME}" "${REPO_ROOT}"

# Always write PCD + pcd_tiles under /output (never /input), even when both
# map to the same host directory.
VOLUME_ARGS=(-v "${INPUT_DIR}:/input:ro" -v "${OUTPUT_DIR_ABS}:/output")
if [[ -n "${OUTPUT_PATH}" && ("${OUTPUT_PATH}" == *.pcd || "${OUTPUT_PATH}" == *.PCD) ]]; then
  OUT_ARG=(--output "/output/$(basename "${OUTPUT_PATH}")")
else
  OUT_ARG=(--output "/output")
fi

TILE_ARG=(--tile-pcd-dir pcd_tiles)
if _has_flag --tile-pcd-dir "${CONVERTER_ARGS[@]}"; then
  TILE_ARG=()
fi

echo "Input (read-only):  ${INPUT_DIR} -> /input"
echo "Output (read-write): ${OUTPUT_DIR_ABS} -> /output"
if [[ -n "${OUTPUT_PATH}" ]]; then
  echo "Output PCD file:    ${OUTPUT_PATH}"
  echo "Tile PCD folder:    ${OUTPUT_DIR_ABS}/pcd_tiles"
fi
if [[ ${#CONVERTER_ARGS[@]} -gt 0 ]]; then
  echo "Converter flags:    ${CONVERTER_ARGS[*]}"
fi

docker run --rm \
  "${VOLUME_ARGS[@]}" \
  "${MAP_MOUNT[@]}" \
  -w /app \
  "${IMAGE_NAME}" \
  python -m lanelet2_generator.las_mgrs_cli \
  --input "${CONTAINER_INPUT}" \
  "${OUT_ARG[@]}" \
  "${TILE_ARG[@]}" \
  "${MAP_ARGS[@]}" \
  "${CONVERTER_ARGS[@]}" \
  "${REMAINING[@]}"

TILE_HOST_DIR="${OUTPUT_DIR_ABS}/pcd_tiles"
if [[ -d "${TILE_HOST_DIR}" ]]; then
  n_tiles="$(find "${TILE_HOST_DIR}" -maxdepth 1 -name '*.pcd' 2>/dev/null | wc -l | tr -d ' ')"
  echo "Wrote ${n_tiles} tile PCD(s) under ${TILE_HOST_DIR}"
else
  echo "WARNING: tile folder not found on host: ${TILE_HOST_DIR}" >&2
fi
