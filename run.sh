#!/usr/bin/env bash
# Run the OMPL tutorial container on Windows via WSL 2.
# Usage: ./run.sh [image_name]   (default: ompl-tutorial)

set -e

IMAGE="${1:-ompl-tutorial}"

xhost +local:docker 2>/dev/null || true

docker run -it --rm \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd):/workspace" \
  "$IMAGE"
