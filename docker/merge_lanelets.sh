#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-lanelet2-generator:latest}"

usage() {
  cat <<'EOF'
Usage:
  docker/merge_lanelets.sh <input.osm> [more.osm ...] -o <merged.osm> [-- merge flags...]

Merge any number of Lanelet2 .osm files inside Docker (same image as other CLI tools).
All input paths and the output path must share a common parent directory (e.g. keep them
in one folder or under the same project tree).

Examples:
  docker/merge_lanelets.sh maps/a.osm maps/b.osm maps/c.osm -o maps/merged.osm
  docker/merge_lanelets.sh ./a.osm ./b.osm -o ./out.osm -- --step-offset 2000000

Notes:
  - Everything after '--' is passed to: python -m lanelet2_generator.merge_lanelets_cli
  - IMAGE_NAME env overrides the image tag (default: lanelet2-generator:latest)
EOF
}

INPUTS=()
OUTPUT=""
EXTRA=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -o|--output)
      if [[ -z "${2:-}" ]]; then
        echo "Missing value for $1" >&2
        exit 1
      fi
      OUTPUT="$2"
      shift 2
      ;;
    --)
      shift
      EXTRA=("$@")
      break
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
    *)
      INPUTS+=("$1")
      shift
      ;;
  esac
done

if [[ ${#INPUTS[@]} -lt 2 ]]; then
  echo "Need at least two input .osm files." >&2
  usage
  exit 1
fi
if [[ -z "${OUTPUT}" ]]; then
  echo "Missing -o / --output for merged .osm path." >&2
  usage
  exit 1
fi

ABS_INPUTS=()
for p in "${INPUTS[@]}"; do
  if [[ "${p}" != /* ]]; then
    rp="$(realpath "${p}")"
  else
    rp="${p}"
  fi
  if [[ ! -f "${rp}" ]]; then
    echo "Input not found: ${p}" >&2
    exit 1
  fi
  ABS_INPUTS+=("${rp}")
done

ABS_OUT="$(realpath -m "${OUTPUT}")"

readarray -t _DOCKER_PATHS < <(
  python3 - "${ABS_INPUTS[@]}" "${ABS_OUT}" <<'PY'
import os
import sys

paths = [os.path.realpath(p) for p in sys.argv[1:]]
if len(paths) < 3:
    print("internal: need >=2 inputs + output", file=sys.stderr)
    sys.exit(1)
*inputs, out = paths
dirs = [os.path.dirname(p) for p in paths]
try:
    common = os.path.commonpath(dirs)
except ValueError:
    print(
        "ERROR: input/output paths do not share a single directory prefix. "
        "Put all .osm files and the output under one tree (e.g. one folder).",
        file=sys.stderr,
    )
    sys.exit(1)
if common == "/":
    print("ERROR: refusing to mount filesystem root as /work.", file=sys.stderr)
    sys.exit(1)

def cpath(p):
    rel = os.path.relpath(p, common)
    return "/work/" + rel.replace(os.sep, "/")

for p in inputs:
    print(cpath(p))
print(cpath(out))
print(common)
PY
)

CONTAINER_INPUTS=("${_DOCKER_PATHS[@]:0:${#_DOCKER_PATHS[@]}-2}")
COMMON="${_DOCKER_PATHS[-1]}"
CONTAINER_OUT="${_DOCKER_PATHS[-2]}"

docker build -f "${REPO_ROOT}/docker/Dockerfile" -t "${IMAGE_NAME}" "${REPO_ROOT}"

mkdir -p "$(dirname "${ABS_OUT}")"

docker run --rm \
  -v "${COMMON}:/work" \
  -w /app \
  "${IMAGE_NAME}" \
  python -m lanelet2_generator.merge_lanelets_cli \
  "${CONTAINER_INPUTS[@]}" \
  -o "${CONTAINER_OUT}" \
  "${EXTRA[@]}"

echo "Wrote: ${ABS_OUT}"
