#!/usr/bin/env bash
# Mở bằng gedit để sửa; chạy bằng bash để tự nạp ROS và workspace.
set -e

pid_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$pid_workspace"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "Không tìm thấy /opt/ros/humble/setup.bash" >&2
    exit 1
fi
if [[ ! -f install/setup.bash ]]; then
    echo "Chưa có install/setup.bash; hãy build workspace trước." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source install/setup.bash

# Mặc định bắt đầu khâu 3 (elbow), rồi dùng next để sang khâu 2 và khâu 1.
# Có thể truyền --joint shoulder, --joint base, --no-show hoặc --dry-run.
exec python3 src/gim_arm_hardware/scripts/tune_motor_gains.py \
    --params-file src/gim_arm_controller_pid/config/pid_hardware_soft.yaml \
    "$@"
