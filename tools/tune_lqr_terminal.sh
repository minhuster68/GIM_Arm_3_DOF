#!/usr/bin/env bash
# Tự nạp ROS/workspace; giữ cùng ROS domain với phần cứng đang dùng.
set -e
lqr_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$lqr_workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
exec python3 tools/tune_lqr_joint.py "$@"
