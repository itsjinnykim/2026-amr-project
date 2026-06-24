#!/usr/bin/env bash
set -euo pipefail

workspace="${1:-$HOME/Desktop/storagy_ws}"

if [[ ! -d "$workspace" ]]; then
  echo "Workspace not found: $workspace" >&2
  echo "Usage: $0 [/path/to/storagy_ws]" >&2
  exit 1
fi

echo "Searching Nav2-related params and launch references in: $workspace"

grep -RIn \
  "planner_server\|controller_server\|local_costmap\|global_costmap\|params_file\|navigation.launch.py\|bringup.launch.py" \
  "$workspace/src" "$workspace/install" \
  --include "*.yaml" \
  --include "*.yml" \
  --include "*.py" \
  2>/dev/null || true
