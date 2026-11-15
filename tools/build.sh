#!/bin/bash
# Build tesseract, threepp, tesseract_threepp and tesseract_visualization_threepp with colcon, in the container, then run the
# tests. All output is under build/. usage: tools/build.sh [--no-test]
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
exec "$ROOT/tools/run.sh" bash -euo pipefail -c '
  mkdir -p build/ccache
  export CMAKE_BUILD_PARALLEL_LEVEL='"${JOBS:-3}"'
  colcon --log-base build/log build \
    --build-base build/colcon --install-base build/install --merge-install \
    --base-paths src tesseract_threepp tesseract_visualization_threepp \
    --packages-up-to tesseract_visualization_threepp --parallel-workers 1 --event-handlers console_direct- console_cohesion+ \
    --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
                 -DTESSERACT_THREEPP_ENABLE_TESTING=ON -DTESSERACT_VISUALIZATION_THREEPP_ENABLE_TESTING=ON -DBUILD_SHARED_LIBS=ON \
                 -DTHREEPP_BUILD_EXAMPLES=OFF -DTHREEPP_BUILD_TESTS=OFF -DTHREEPP_BUILD_EDITOR=OFF -DTHREEPP_WITH_AUDIO=OFF
  if [ "'"${1:-}"'" != "--no-test" ]; then
    export LD_LIBRARY_PATH=/ws/build/install/lib
    ctest --test-dir build/colcon/tesseract_threepp --output-on-failure
    ctest --test-dir build/colcon/tesseract_visualization_threepp --output-on-failure
  fi
'
