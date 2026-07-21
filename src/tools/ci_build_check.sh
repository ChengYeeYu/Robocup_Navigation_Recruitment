#!/usr/bin/env bash
# Local pre-flight build check. Run this before handing the repo out (or
# after pulling changes) to catch compile/packaging errors early.
#
# Usage: bash src/tools/ci_build_check.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: /opt/ros/humble/setup.bash not found. Run src/tools/install_dependencies.sh first." >&2
  exit 1
fi

echo "==> colcon build (bringup, planner_cpp, planner_py)"
colcon build --symlink-install --packages-select bringup planner_cpp planner_py

echo "==> colcon test (lint)"
colcon test --packages-select bringup planner_cpp planner_py
colcon test-result --verbose

echo "==> All good."
