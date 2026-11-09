#!/bin/bash
# Run a command in the development image with this directory mounted at /ws.
# Host networking lets a browser on this machine reach a viewer started in the container.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TTY=(); [ -t 0 ] && TTY=(-it) || true
# PODMAN_ARGS adds options, such as the seccomp change tools/test_sanitizers.sh needs for setarch
exec podman run --rm "${TTY[@]}" ${PODMAN_ARGS:-} --network host --memory="${MEM:-12g}" \
  -v "$ROOT:/ws" -w /ws \
  -e TESSERACT_RESOURCE_PATH=/ws/src \
  tesseract-visualization-threepp-dev:noble "$@"
