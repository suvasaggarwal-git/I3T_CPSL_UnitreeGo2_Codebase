#!/bin/bash
# Build the full workspace from the workspace root.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "Workspace: ${WORKSPACE_ROOT}"

cd "${WORKSPACE_ROOT}"
rm -rf build/ install/ log/

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
