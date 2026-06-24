#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /path/to/storagy-practice-ws-docker" >&2
  exit 1
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
storagy_root="$1"

if [[ ! -d "$storagy_root/src/storagy" ]]; then
  echo "Storagy package not found under: $storagy_root/src/storagy" >&2
  exit 1
fi

mkdir -p \
  "$storagy_root/src/storagy/param" \
  "$storagy_root/src/storagy/scripts" \
  "$storagy_root/src/storagy/behavior_trees"

cp "$repo_root/config/nav2_params_dwb_safe.yaml" \
  "$storagy_root/src/storagy/param/nav2_params_dwb_safe.yaml"
cp "$repo_root/scripts/nav2_score_monitor.py" \
  "$storagy_root/src/storagy/scripts/nav2_score_monitor.py"
cp "$repo_root/scripts/compare_nav2_score_runs.py" \
  "$storagy_root/src/storagy/scripts/compare_nav2_score_runs.py"
cp "$repo_root/behavior_trees/score_replanning_recovery.xml" \
  "$storagy_root/src/storagy/behavior_trees/score_replanning_recovery.xml"

chmod +x \
  "$storagy_root/src/storagy/scripts/nav2_score_monitor.py" \
  "$storagy_root/src/storagy/scripts/compare_nav2_score_runs.py"

echo "Installed score-based Nav2 files into: $storagy_root/src/storagy"
echo "Tuned params: src/storagy/param/nav2_params_dwb_safe.yaml"
echo "BT XML:       src/storagy/behavior_trees/score_replanning_recovery.xml"
echo "Tools:        src/storagy/scripts/nav2_score_monitor.py"
