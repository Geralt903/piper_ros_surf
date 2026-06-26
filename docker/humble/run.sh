#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

docker run --rm -it \
  --name piper_ros_humble \
  --network host \
  --privileged \
  --ipc host \
  -e DISPLAY="${DISPLAY:-}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${REPO_ROOT}:/ws/piper_ros" \
  -w /ws/piper_ros \
  piper-ros:humble \
  bash
