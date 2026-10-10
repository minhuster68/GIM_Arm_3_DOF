#!/usr/bin/env bash
set -e
smc_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$smc_workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
exec python3 tools/tune_smc_joint.py "$@"
