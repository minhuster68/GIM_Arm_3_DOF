#!/usr/bin/env python3
"""
calibrate_gravity.py — hiệu chỉnh mô hình trọng lực từ NHIỀU tư thế.

    # T1: ros2 launch gim_control origin_gim_arm_control.launch.py  (giữ chạy)
    # Đưa tay tới 1 tư thế, GIỮ YÊN, rồi:
    python3 tools/calibrate_gravity.py --add
    # Đổi tư thế, lặp lại >= 4 lần, rồi:
    python3 tools/calibrate_gravity.py --fit

===========================================================================
VÌ SAO PHẢI NHIỀU TƯ THẾ
===========================================================================
`check_gravity_model` đo MỘT tư thế, cho 3 con số. Có 3 ẩn (offset của 3 khớp).
3 phương trình 3 ẩn -> LUÔN giải được khớp hoàn hảo, kể cả khi mô hình sai
hoàn toàn. Sai dư bằng 0 ở đó không chứng minh gì cả.

Với N tư thế, ta có 3N phương trình cho 3 ẩn. N=4 -> 12 phương trình, dư 9.
Lúc đó SAI DƯ mới có nghĩa, và nó phân biệt được ba nguyên nhân rất khác nhau:

  sai dư NHỎ (< ~10% |G|)     -> chỉ lệch điểm 0. Script in ra 3 số
                                  zero_offset_rad để dán vào URDF. Xong.
  sai dư LỚN nhưng TỈ LỆ với  -> khối lượng URDF sai. Script in ra hệ số
    |G| (cùng dấu, cùng hệ số)   nhân tốt nhất -- gần 1.3 nghĩa là tay thật
                                  nặng hơn URDF 30% (tải chưa khai?).
  sai dư LỚN và LOẠN XẠ        -> hình học sai (<origin rpy> / <axis>), hoặc
                                  có tư thế nào đó đang tựa vào cữ cơ khí.

===========================================================================
CHỌN TƯ THẾ ĐO
===========================================================================
Điều kiện: |G(q)| phải TRỘI hơn ma sát tĩnh ở cả 3 khớp. Ma sát khô ước tính
0.032 / 0.256 / 0.032 Nm (base/shoulder/elbow). Muốn sai số dưới 10% thì cần
|G| > 10x ma sát, tức > 0.32 / 2.56 / 0.32 Nm.

Tư thế gần q = 0 là TỆ NHẤT: ở đó |G| nhỏ nhất. Tư thế tốt là VAI HẠ XUỐNG
(shoulder ~ -1.1 rad) -- lúc đó cả cánh tay nằm ngang, moment arm lớn nhất.

Dùng --suggest để in danh sách tư thế tốt kèm |G| dự kiến.

===========================================================================
KHỬ MA SÁT TĨNH -- làm nếu muốn số đẹp
===========================================================================
Mô-men đo lúc đứng yên = G(q) + ma sát tĩnh, và ma sát tĩnh nằm đâu đó trong
một dải, dấu tuỳ chiều bạn vừa đi tới. Cách khử: tới CÙNG tư thế hai lần, một
lần từ trên xuống, một lần từ dưới lên, và --add cả hai. Trung bình của chúng
là G(q) thật; nửa hiệu là độ lớn ma sát. Script tự làm việc này khi thấy hai
bản ghi có q gần nhau (< 0.02 rad).
"""

import argparse
import json
import math
import os
import socket
import struct
import sys
import time

import numpy as np

# (tên, node_id, gear, direction) -- khớp <ros2_control> trong gim_arm.urdf
AXES = [("base_joint", 0, 8.0, -1.0),
        ("shoulder_joint", 1, 64.0, +1.0),
        ("elbow_joint", 2, 8.0, -1.0)]

CMD_GET_ENCODER, CMD_GET_TORQUES = 0x009, 0x01C
CAN_RTR_FLAG = 0x40000000
DEFAULT_STORE = os.path.expanduser("~/.gim_gravity_cal.json")


def load_dyn(urdf):
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, "..", "kinematics_test", "arm_dynamics.py"),
                 os.path.join(here, "..", "src", "gim_arm_control",
                              "gim_control", "arm_dynamics.py")):
        cand = os.path.normpath(cand)
        if os.path.isfile(cand):
            spec = importlib.util.spec_from_file_location("_ad", cand)
            m = importlib.util.module_from_spec(spec)
            sys.modules["_ad"] = m
            spec.loader.exec_module(m)
            return m.ArmDynamics(urdf), m
    sys.exit("Không tìm thấy arm_dynamics.py")


def measure(ifname, n_samples=30):
    """Trả về (q_raw, tau_joint) -- q chưa trừ offset, tau đã nhân gear·dir."""
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        s.bind((ifname,))
    except OSError as e:
        sys.exit(f"Không mở được '{ifname}': {e}")
    s.settimeout(0.2)

    pos = {n: [] for _, n, _, _ in AXES}
    trq = {n: [] for _, n, _, _ in AXES}
    t_end = time.time() + 3.0
    while time.time() < t_end and min(len(v) for v in trq.values()) < n_samples:
        for _, node, _, _ in AXES:                      # RTR xin Get_Torques
            cid = ((node << 5) | CMD_GET_TORQUES) | CAN_RTR_FLAG
            s.send(struct.pack("<IBBBB", cid, 8, 0, 0, 0) + b"\x00" * 8)
        deadline = time.time() + 0.05
        while time.time() < deadline:
            try:
                f = s.recv(16)
            except socket.timeout:
                break
            cid, dlc = struct.unpack_from("<IB", f, 0)
            cid &= 0x7FF
            node, cmd = cid >> 5, cid & 0x1F
            if node not in pos or dlc < 8:
                continue
            if cmd == CMD_GET_ENCODER:
                pos[node].append(struct.unpack_from("<f", f[8:], 0)[0])
            elif cmd == CMD_GET_TORQUES:
                trq[node].append(struct.unpack_from("<f", f[8:], 4)[0])

    missing = [nm for nm, node, _, _ in AXES if not trq[node]]
    if missing:
        sys.exit(f"Không nhận được Get_Torques từ: {', '.join(missing)}\n"
                 "Driver có trả lời RTR cho 0x01C không? Thử odrivetool.")

    q = np.array([np.mean(pos[node]) / gear * 2 * math.pi * d if pos[node] else 0.0
                  for _, node, gear, d in AXES])
    tau = np.array([np.mean(trq[node]) * gear * d for _, node, gear, d in AXES])
    return q, tau


def suggest(dyn, m):
    tau_f = np.array([m.DRY_FRICTION_PER_N * m.GEAR_RATIOS[n]
                      for n in dyn.joint_names])
    rng = np.random.default_rng(1)
    span = dyn.q_max - dyn.q_min
    pool = []
    for _ in range(120000):
        q = dyn.q_min + 0.12 * span + rng.random(3) * span * 0.76   # chừa mép
        pool.append((np.min(np.abs(dyn.gravity(q)) / tau_f), q))
    pool.sort(key=lambda x: -x[0])
    picked = []
    for r, q in pool:
        if all(np.linalg.norm(q - p) > 0.6 for p in picked):
            picked.append(q)
            G = dyn.gravity(q)
            print(f"  q = [{q[0]:6.3f},{q[1]:6.3f},{q[2]:6.3f}]  "
                  f"|G| = [{abs(G[0]):.3f}, {abs(G[1]):.3f}, {abs(G[2]):.3f}] Nm  "
                  f"tỉ số tệ nhất {r:5.1f}x")
        if len(picked) >= 6:
            break
    print("\n  Chọn 4-5 tư thế trong danh sách, càng khác nhau càng tốt.")
    print("  Đi tới bằng forward_position_controller, nhích từng 0.1 rad.")


def fit(dyn, recs, m):
    from scipy.optimize import least_squares
    Q = np.array([r["q"] for r in recs])
    T = np.array([r["tau"] for r in recs])

    # gộp các bản ghi ở cùng tư thế (khử ma sát tĩnh bằng trung bình 2 chiều)
    groups, used = [], set()
    for i in range(len(Q)):
        if i in used:
            continue
        same = [j for j in range(len(Q)) if np.linalg.norm(Q[j] - Q[i]) < 0.02]
        used.update(same)
        groups.append((Q[same].mean(axis=0), T[same].mean(axis=0), len(same)))
    print(f"{len(recs)} bản ghi -> {len(groups)} tư thế phân biệt "
          f"({sum(1 for g in groups if g[2] > 1)} tư thế đo 2 chiều, đã khử ma sát)\n")
    if len(groups) < 3:
        print("!! Cần ít nhất 3 tư thế phân biệt (nên 4-5). Đo thêm rồi --fit lại.")
        return

    def resid(p):
        dq, scale = p[:3], p[3]
        return np.concatenate([scale * dyn.gravity(q + dq) - t for q, t, _ in groups])

    # Bước 1: chỉ fit offset (scale ép = 1)
    r1 = least_squares(lambda d: resid(np.r_[d, 1.0]), np.zeros(3), bounds=(-2, 2))
    # Bước 2: fit offset + hệ số khối lượng
    r2 = least_squares(resid, np.r_[r1.x, 1.0],
                       bounds=([-2, -2, -2, 0.3], [2, 2, 2, 3.0]))

    tau_f = np.array([m.DRY_FRICTION_PER_N * m.GEAR_RATIOS[n]
                      for n in dyn.joint_names])
    # |G| điển hình của bộ tư thế -- dùng để đánh giá sai dư theo TỈ LỆ.
    # Ngưỡng tuyệt đối theo riêng ma sát là quá chặt ở base: ma sát base chỉ
    # 0.032 Nm, nhỏ hơn cả sai số đo dòng điện, nên một phép fit ĐÚNG vẫn bị
    # báo là thất bại. Lấy ngưỡng = max(2·ma_sát, 12%·|G|).
    G_typ = np.mean([np.abs(dyn.gravity(q)) for q, _, _ in groups], axis=0)
    thresh = np.maximum(2.0 * tau_f, 0.12 * G_typ)

    def report(lbl, r, scale):
        res = r.fun.reshape(len(groups), 3)
        rms = np.sqrt((res ** 2).mean(axis=0))
        print(f"--- Giả thuyết: {lbl} ---")
        print(f"  offset (rad)     : {np.round(r.x[:3], 4)}   "
              f"({np.round(np.degrees(r.x[:3]), 1)} độ)")
        if scale != 1.0:
            print(f"  hệ số khối lượng : {scale:.3f}  "
                  f"(1.0 = URDF đúng; 1.3 = tay thật nặng hơn 30%)")
        print(f"  sai dư RMS (Nm)  : {np.round(rms, 4)}")
        print(f"  = % của |G| điển hình: "
              f"{np.round(rms / G_typ * 100, 1)}%   (dưới 12% là đạt)")
        print(f"  ngưỡng đạt (Nm)  : {np.round(thresh, 4)}  -> "
              f"{'ĐẠT' if np.all(rms < thresh) else 'KHÔNG ĐẠT'}")
        print()
        return rms

    report("CHỈ lệch điểm 0", r1, 1.0)
    rms2 = report("điểm 0 + hệ số khối lượng", r2, r2.x[3])

    print("=" * 70)
    if np.all(rms2 < thresh):
        off = -r2.x[:3]
        print("KẾT LUẬN: mô hình giải thích được số đo. Dán vào <ros2_control>:")
        for i, (nm, _, _, _) in enumerate(AXES):
            print(f'  <joint name="{nm}"> ... '
                  f'<param name="zero_offset_rad">{off[i]:.4f}</param>')
        if abs(r2.x[3] - 1.0) > 0.08:
            print(f"\nVÀ: hệ số khối lượng {r2.x[3]:.3f} -- nhân <mass> trong URDF "
                  f"cho số này,\nhoặc thêm hẳn khối lượng tải vào link cuối. "
                  f"Nếu bạn mới lắp tải mà chưa khai,\nđây chính là nó.")
    else:
        print("KẾT LUẬN: KHÔNG có bộ offset + hệ số nào giải thích được số đo.")
        print("Sai dư vượt ngưỡng ma sát -> vấn đề nằm ở HÌNH HỌC, không phải")
        print("điểm 0 hay khối lượng. Nghi ngờ theo thứ tự:")
        print("  1. <origin rpy> hoặc <axis> của base_joint. Comment trong URDF")
        print("     của bạn đã đánh dấu: axis = '0.99932 -0.031099 -0.019999',")
        print("     'lệch trục sạch, cần xác nhận với CAD'.")
        print("  2. Có tư thế nào đó đang TỰA VÀO CỮ CƠ KHÍ -- kết cấu gánh tải")
        print("     thay động cơ, nên mô-men đo được nhỏ bất thường. Xem cột")
        print("     sai dư từng tư thế bên dưới.")
        print()
        print("Sai dư từng tư thế (Nm):")
        res = r2.fun.reshape(len(groups), 3)
        for i, (q, t, n) in enumerate(groups):
            print(f"  q={np.round(q,3)}  sai dư={np.round(res[i],3)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can", default="can0")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--store", default=DEFAULT_STORE)
    ap.add_argument("--add", action="store_true", help="đo tư thế hiện tại và lưu")
    ap.add_argument("--fit", action="store_true", help="fit trên toàn bộ bản ghi")
    ap.add_argument("--suggest", action="store_true", help="in tư thế đo tốt")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--clear", action="store_true")
    args = ap.parse_args()

    urdf = args.urdf
    if urdf is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            urdf = os.path.join(get_package_share_directory("gim_arm_description"),
                                "urdf", "gim_arm.urdf")
        except Exception:
            sys.exit("Truyền --urdf")
    dyn, m = load_dyn(urdf)

    recs = []
    if os.path.exists(args.store):
        recs = json.load(open(args.store))

    if args.clear:
        os.path.exists(args.store) and os.remove(args.store)
        print("Đã xoá bản ghi.")
        return
    if args.suggest:
        print("TƯ THẾ ĐO TỐT (|G| trội hơn ma sát ở cả 3 khớp):\n")
        suggest(dyn, m)
        return
    if args.list or (not args.add and not args.fit):
        print(f"{len(recs)} bản ghi trong {args.store}")
        for i, r in enumerate(recs):
            print(f"  [{i}] q={np.round(r['q'],4)}  tau={np.round(r['tau'],4)}")
        if not recs:
            print("  (chưa có. Dùng --suggest rồi --add)")
        return

    if args.add:
        q, tau = measure(args.can)
        G = dyn.gravity(q)
        tau_f = np.array([m.DRY_FRICTION_PER_N * m.GEAR_RATIOS[n]
                          for n in dyn.joint_names])
        ratio = np.abs(G) / tau_f
        print(f"q   = {np.round(q, 4)}")
        print(f"tau = {np.round(tau, 4)} Nm (khớp, đã nhân gear·dir)")
        print(f"G   = {np.round(G, 4)} Nm (mô hình, tại q chưa hiệu chỉnh)")
        print(f"|G|/ma sát = {np.round(ratio, 1)}  (cần > 5 ở CẢ BA khớp)")
        if np.any(ratio < 5):
            bad = [dyn.joint_names[i] for i in range(3) if ratio[i] < 5]
            print(f"\n!! TƯ THẾ KÉM: {', '.join(bad)} có trọng lực quá nhỏ so với")
            print("   ma sát. Bản ghi vẫn được lưu nhưng sẽ làm nhiễu phép fit.")
            print("   Dùng --suggest để tìm tư thế tốt hơn.")
            if input("   Vẫn lưu? [y/N] ").strip().lower() != "y":
                return
        recs.append(dict(q=q.tolist(), tau=tau.tolist(), t=time.time()))
        json.dump(recs, open(args.store, "w"), indent=1)
        print(f"\nĐã lưu. Tổng {len(recs)} bản ghi. Cần >= 4 tư thế phân biệt.")

    if args.fit:
        if not recs:
            sys.exit("Chưa có bản ghi nào.")
        fit(dyn, recs, m)


if __name__ == "__main__":
    main()