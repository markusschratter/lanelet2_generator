#!/usr/bin/env bash
# Shared helpers for lanelet2_generator Docker wrapper scripts.

ensure_docker_image() {
  local repo_root="${1:?repo root required}"
  local image_name="${2:-lanelet2-generator:latest}"
  local dockerfile="${repo_root}/docker/Dockerfile"

  if [[ ! -f "${dockerfile}" ]]; then
    echo "Dockerfile not found: ${dockerfile}" >&2
    exit 1
  fi

  if [[ "${DOCKER_REBUILD:-}" == "1" ]]; then
    echo "Building Docker image ${image_name} (DOCKER_REBUILD=1) ..."
    docker build -f "${dockerfile}" -t "${image_name}" "${repo_root}"
    return 0
  fi

  if docker image inspect "${image_name}" >/dev/null 2>&1; then
    echo "Using Docker image: ${image_name}"
    return 0
  fi

  echo "Building Docker image ${image_name} (not found locally) ..."
  docker build -f "${dockerfile}" -t "${image_name}" "${repo_root}"
}
