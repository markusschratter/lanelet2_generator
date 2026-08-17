#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-lanelet2-generator:latest}"

# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

echo "Building ${IMAGE_NAME} ..."
DOCKER_REBUILD=1 ensure_docker_image "${REPO_ROOT}" "${IMAGE_NAME}"
echo "Done."
