#!/usr/bin/env bash
# ab_feedforward.sh — chạy A/B feedforward tự động TRÊN BUS ẢO và in bảng so sánh.
#
# Vì sao cần script: một vòng A/B gồm 6 bước (sửa URDF -> build -> khởi động lại
# driver -> khởi động lại ros2_control -> đợi encoder có số thật -> chạy quỹ đạo),
# làm tay rất dễ sót. Sót nguy hiểm nhất là bước "đợi encoder": nếu chạy lúc
# /joint_states còn NaN thì bảng sai số vẫn in ra bình thường nhưng toàn số NaN
# hoặc số rác -- trông như kết quả thật. Script này kiểm trước mỗi lần đo.
#
# CHỈ CHẠY TRÊN VCAN. Script tự từ chối nếu can0 là bus thật: nó khởi động lại
# controller và tự động cho tay chạy hết 1 vòng quỹ đạo, không phải thứ được
# phép tự ý làm với thiết bị đang đeo trên tay người. Trên tay thật thì làm tay
# theo RUNBOOK.md để lúc nào cũng có người cầm nút dừng.
#
#   ./tools/ab_feedforward.sh [can_interface]

# KHÔNG dùng "set -u": /opt/ros/humble/setup.bash tham chiếu biến chưa đặt
# (AMENT_TRACE_SETUP_FILES) và sẽ chết ngay ở dòng source đầu tiên.
set -o pipefail
IFACE="${1:-can0}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
URDF="$ROOT/src/gim_arm_description/urdf/gim_arm.urdf"
LOG="$(mktemp -d)"
cd "$ROOT"

if ! ip -d link show "$IFACE" 2>/dev/null | grep -q vcan; then
  echo "DỪNG: '$IFACE' không phải vcan (hoặc không tồn tại)."
  echo "Dựng bus ảo:  sudo modprobe vcan && sudo ip link add dev $IFACE type vcan"
  echo "              sudo ip link set up $IFACE"
  echo "Nếu đây là bus THẬT có động cơ: làm tay theo RUNBOOK.md, đừng dùng script này."
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash" 2>/dev/null || { echo "Chưa colcon build."; exit 1; }

cleanup() {
  pkill -f gim6010_mujoco_sim 2>/dev/null
  pkill -f ros2_control_node 2>/dev/null
  pkill -f robot_state_publisher 2>/dev/null
  sleep 1
}
trap cleanup EXIT

set_ff() {   # $1 = true|false
  sed -i "s|<param name=\"velocity_feedforward\">[a-z]*|<param name=\"velocity_feedforward\">$1|; \
          s|<param name=\"gravity_feedforward\">[a-z]*|<param name=\"gravity_feedforward\">$1|" "$URDF"
  colcon build --packages-select gim_arm_description > "$LOG/build.log" 2>&1 \
    || { echo "build gim_arm_description LỖI, xem $LOG/build.log"; exit 1; }
  source "$ROOT/install/setup.bash"
}

wait_for_valid_encoder() {   # trả về 1 nếu sau 30s vẫn chưa có số thật
  for _ in $(seq 30); do
    local pos
    pos=$(timeout 3 ros2 topic echo /joint_states --once 2>/dev/null \
          | sed -n '/^position/,/^velocity/p' | grep -c 'nan')
    [ "$pos" = "0" ] && return 0
    sleep 1
  done
  return 1
}

run_case() {   # $1 = true|false, $2 = nhãn
  echo "== $2: đang dựng lại (sửa URDF + build + khởi động lại) ..."
  cleanup
  set_ff "$1"
  ( setsid nohup python3 "$ROOT/src/gim_arm_control/gim_control/gim6010_mujoco_sim.py" \
      --can "$IFACE" > "$LOG/sim_$1.log" 2>&1 < /dev/null & )
  sleep 4
  ( setsid nohup ros2 launch gim_control origin_gim_arm_control.launch.py \
      > "$LOG/launch_$1.log" 2>&1 < /dev/null & )
  sleep 12

  grep -o "Feedforward:.*" "$LOG/launch_$1.log" | head -1 | sed 's/^/   plugin báo: /'
  if ! wait_for_valid_encoder; then
    echo "   DỪNG: /joint_states vẫn NaN sau 30s -- driver chưa cấp encoder."
    exit 1
  fi
  echo "   encoder OK, đang chạy 1 vòng quỹ đạo (~40s) ..."
  MPLBACKEND=Agg ros2 run gim_control origin_draw_trajectory > "$LOG/run_$1.log" 2>&1
  awk '/SAI SỐ BÁM/{f=1} f' "$LOG/run_$1.log" | sed -n '3,6p'
  echo
}

echo "Kết quả ghi tại: $LOG"
echo
run_case false "FEEDFORWARD TẮT (chuẩn)"
run_case true  "FEEDFORWARD BẬT"

echo "========================= SO SÁNH ========================="
paste <(awk '/SAI SỐ BÁM/{f=1} f' "$LOG/run_false.log" | sed -n '4,6p') \
      <(awk '/SAI SỐ BÁM/{f=1} f' "$LOG/run_true.log"  | sed -n '4,6p' | awk '{$1="";print}') \
  | awk 'BEGIN{printf "%-16s %10s %10s | %10s %10s   %s\n","khớp","RMS tắt","max tắt","RMS bật","max bật","cải thiện"}
         {printf "%-16s %10.4f %10.4f | %10.4f %10.4f   %6.1fx\n",$1,$2,$3,$5,$6,($5>0?$2/$5:0)}'
echo "==========================================================="
echo "URDF đang ở trạng thái BẬT. Log đầy đủ: $LOG"
