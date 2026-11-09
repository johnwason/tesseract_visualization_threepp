#!/bin/bash
# Run the threepp tests on a virtual display, so that the GLFW window test runs as well as the headless ones.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
# --init: as PID 1, xvfb-run never starts its command
export PODMAN_ARGS="--init ${PODMAN_ARGS:-}"
exec "$ROOT/tools/run.sh" bash -euo pipefail -c '
  export LD_LIBRARY_PATH=/ws/build/install/lib LIBGL_ALWAYS_SOFTWARE=1
  timeout -k 5 300 xvfb-run -a ctest --test-dir build/colcon/tesseract_visualization_threepp --output-on-failure -V
'
