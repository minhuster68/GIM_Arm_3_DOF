#!/usr/bin/env python3
"""
replay_trajectory.py -- xem TRUOC quy dao that trong MuJoCo, khong can hardware.

Dung DUNG cung duong tinh toan voi draw_trajectory.py (gim_control.shapes ->
GimArmKinematics.solve_trajectory), nen quy dao hien o day chinh la quy dao se
gui xuong /gim_arm_group_controller/follow_joint_trajectory.

Khac test_mujoco.py: file do chi goi mj_step khong dieu khien gi, nen chi cho
xem tay may ROI TU DO. File nay phat lai quy dao that.

CHAY:
  cd ~/gim60108_ws && source install/setup.bash
  python3 src/gim_arm_mujoco/replay_trajectory.py            # bao cao + viewer
  python3 src/gim_arm_mujoco/replay_trajectory.py --no-viewer # chi bao cao

GIOI HAN QUAN TRONG -- doc truoc khi tin ket qua:
MuJoCo nap CUNG file URDF ma Pinocchio dung de tinh g(q). Nen neu mo phong
lai vong dieu khien MIT (torque_ff = g(q) tu Pinocchio), bu trong luc se
CHINH XAC TUYET DOI vi hai ben dung y het mot mo hinh -- tay may se "lo lung"
hoan hao bat ke khoi luong/tam khoi trong URDF co dung robot that hay khong.
=> Phep thu troi tu do KHONG THE kiem chung bang mo phong. Chi hardware moi
   tra loi duoc "URDF co khop robot that khong". Xem khoi comment trong
   gim_arm.urdf.
Nhung gi mo phong O DAY TRA LOI DUOC (va deu la thu can biet truoc):
  1. Duong di hinh hoc: dau but co ve dung chu O khong, IK co hoi tu khong
  2. Co vuot gioi han khop <limit lower/upper> khong
  3. Mo-men CAN THIET doc quy dao co vuot effort="5" Nm khong
  4. Van toc khop can thiet co vuot velocity="12.5" rad/s khong
"""

import argparse
import os
import re
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS_SRC = os.path.abspath(os.path.join(HERE, ".."))

# gim_control co the chua duoc source -- fallback vao thang thu muc nguon.
try:
    from gim_control.gim_arm_kinematics import GimArmKinematics
    from gim_control.shapes import letter_o, discretize
except ImportError:
    sys.path.insert(0, os.path.join(WS_SRC, "gim_arm_control"))
    from gim_control.gim_arm_kinematics import GimArmKinematics
    from gim_control.shapes import letter_o, discretize

import mujoco

# ---- Cau hinh: GIU DONG BO voi main() cua draw_trajectory.py ----------------
# Neu doi hinh ve / dt / tool offset o draw_trajectory.py thi phai doi o day,
# khong thi mo phong khong con phan anh quy dao that nua.
URDF_SRC = os.path.join(WS_SRC, "gim_arm_description", "urdf", "gim_arm.urdf")
TOOL_OFFSET = (0.4031, 0.049, -0.029)
SHAPE_KW = dict(center=(-0.3021, 0.1447), radius=0.025, plane="x", plane_value=0.1)
N_POINTS = 60
DT = 0.3                # giay giua 2 diem, nhu draw_trajectory.py
TRANSITION_TIME = 3.0   # doan di chuyen em ve diem dau
EFFORT_LIMIT = 5.0      # <limit effort> trong URDF
VELOCITY_LIMIT = 12.5   # <limit velocity> trong URDF


def build_mujoco_model():
    """Nap URDF vao MuJoCo. Doi package:// thanh duong dan TUYET DOI (khong
    dung duong dan tuong doi nhu test_mujoco.py -- de chay duoc tu bat ky cwd).
    """
    with open(URDF_SRC, "r", encoding="utf-8") as f:
        content = f.read()
    mesh_dir = os.path.join(HERE, "meshes")
    fixed = re.sub(r"package://[^/]+/meshes/", mesh_dir + os.sep, content)
    out_path = os.path.join(HERE, "gim_arm_mujoco.urdf")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(fixed)
    model = mujoco.MjModel.from_xml_path(out_path)

    # TAT TIEP XUC -- bat buoc, khong phai toi uu.
    # <collision> trong URDF dung DUNG mesh day cua tung link, ma cac link lien
    # ke ve theo CAD thi long vao nhau san. Ngay o tu the nghi MuJoCo tim ra 4
    # tiep xuc voi do lun 70-80mm, sinh ra qfrc_constraint ~1080 Nm hoan toan
    # ao. mj_inverse tru luc ao do vao ket qua nen qfrc_inverse bao 3417 Nm
    # trong khi mo-men trong luc that chi ~2.24 Nm (da doi chieu voi
    # pin.rnea cua Pinocchio: khop nhau tuyet doi sau khi tat tiep xuc).
    # Phat lai o day la KINEMATIC (ghi qpos + mj_forward, khong step vat ly)
    # nen tat tiep xuc khong anh huong gi den duong di hien thi.
    # => Doi lai: file nay KHONG kiem tra duoc tu dung do. Muon kiem tra thi
    #    phai thay <collision> bang hinh bao don gian (capsule/box) truoc.
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    return model, out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-viewer", action="store_true", help="chi in bao cao")
    ap.add_argument("--speed", type=float, default=1.0, help="he so toc do phat lai")
    args = ap.parse_args()

    # ---- 1. Tinh quy dao y het draw_trajectory.py ---------------------------
    kin = GimArmKinematics(URDF_SRC, tool_offset_xyz=TOOL_OFFSET)
    targets = discretize(letter_o(**SHAPE_KW), n_points=N_POINTS, close_loop=True)
    results = kin.solve_trajectory(targets)
    q_draw = np.array([r.q for r in results])

    print("=" * 74)
    print("1. IK")
    print("=" * 74)
    n_bad = sum(not r.converged for r in results)
    errs = np.array([r.position_error_m for r in results])
    print(f"So diem: {len(results)}, khong hoi tu: {n_bad}")
    print(f"Sai so vi tri: max {errs.max() * 1000:.4f} mm, trung binh "
          f"{errs.mean() * 1000:.4f} mm")
    if n_bad:
        print("=> draw_trajectory.py se DUNG va khong gui gi (co check nay san).")
        return

    # Sai so hinh hoc thuc su: FK lai roi so voi diem dich.
    fk_pts = np.array([kin.fk_position(q) for q in q_draw])
    dev = np.linalg.norm(fk_pts - np.array(targets), axis=1)
    print(f"Kiem tra lai bang FK: lech max {dev.max() * 1000:.4f} mm")
    r_fit = np.linalg.norm(fk_pts[:, 1:] - np.array(SHAPE_KW["center"]), axis=1)
    print(f"Ban kinh chu O do lai tu FK: {r_fit.mean():.5f} m "
          f"(dat {SHAPE_KW['radius']}), do tron +-{(r_fit.max()-r_fit.min())*1000:.4f} mm")
    print(f"Do lech mat phang ve (x): {fk_pts[:, 0].std() * 1000:.4f} mm")

    # ---- 2. Gioi han khop ---------------------------------------------------
    print()
    print("=" * 74)
    print("2. GIOI HAN KHOP")
    print("=" * 74)
    lo, hi = kin.model.lowerPositionLimit, kin.model.upperPositionLimit
    for i, name in enumerate(kin.joint_names):
        col = q_draw[:, i]
        margin_lo, margin_hi = col.min() - lo[i], hi[i] - col.max()
        warn = ""
        if margin_lo < 0 or margin_hi < 0:
            warn = "  <-- VUOT GIOI HAN"
        elif min(margin_lo, margin_hi) < 0.05:
            warn = "  <-- SAT GIOI HAN (<0.05 rad)"
        print(f"  {name:16s} chay {col.min():+.4f} .. {col.max():+.4f}  "
              f"(limit {lo[i]:+.4f} .. {hi[i]:+.4f}, du {margin_lo:.4f}/{margin_hi:.4f}){warn}")

    # ---- 3. Quy dao day du: doan chuyen tiep + vong ve ----------------------
    # draw_trajectory.py chen point[0] = vi tri hien tai, point[1] = diem dau
    # quy dao sau TRANSITION_TIME. Mo phong dung diem dau lam "vi tri hien tai"
    # gia dinh la tay may da o do -- doan chuyen tiep that phu thuoc vi tri
    # thuc luc launch nen khong the biet truoc.
    times = np.concatenate([[0.0], TRANSITION_TIME + DT * np.arange(len(q_draw))])
    q_full = np.vstack([q_draw[0], q_draw])

    # ---- 4. Mo-men + van toc CAN THIET (inverse dynamics trong MuJoCo) -----
    model, urdf_out = build_mujoco_model()
    data = mujoco.MjData(model)
    print()
    print("=" * 74)
    print("3. MO-MEN / VAN TOC CAN THIET (MuJoCo inverse dynamics)")
    print("=" * 74)

    # Thu tu khop cua MuJoCo KHONG chac trung Pinocchio -- tra theo TEN.
    mj_idx = []
    for name in kin.joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            print(f"  Khong tim thay khop '{name}' trong model MuJoCo.")
            return
        mj_idx.append(model.jnt_dofadr[jid])
    print(f"  Anh xa khop -> dof MuJoCo: "
          f"{dict(zip(kin.joint_names, mj_idx))}")
    if mj_idx != list(range(len(mj_idx))):
        print("  (thu tu KHAC Pinocchio -- da tra theo ten nen van dung)")

    # Sai phan huu han tren luoi thoi gian that de ra qvel/qacc.
    qd = np.gradient(q_full, times, axis=0)
    qdd = np.gradient(qd, times, axis=0)

    tau = np.zeros_like(q_full)
    for k in range(len(q_full)):
        for i, dof in enumerate(mj_idx):
            data.qpos[dof] = q_full[k, i]
            data.qvel[dof] = qd[k, i]
            data.qacc[dof] = qdd[k, i]
        mujoco.mj_inverse(model, data)
        for i, dof in enumerate(mj_idx):
            tau[k, i] = data.qfrc_inverse[dof]

    print(f"  {'khop':16s} {'|tau| max':>10s} {'du so voi 5Nm':>14s} "
          f"{'|qd| max':>10s} {'du so voi 12.5':>15s}")
    for i, name in enumerate(kin.joint_names):
        t_max, v_max = np.abs(tau[:, i]).max(), np.abs(qd[:, i]).max()
        flag = "  <-- VUOT" if t_max > EFFORT_LIMIT else ""
        print(f"  {name:16s} {t_max:10.4f} {EFFORT_LIMIT - t_max:14.4f} "
              f"{v_max:10.4f} {VELOCITY_LIMIT - v_max:15.4f}{flag}")
    print(f"  (mo-men nay = trong luc + quan tinh, KHONG gom ma sat/hop so thuc)")

    # ---- 5. Viewer ----------------------------------------------------------
    if args.no_viewer:
        print(f"\nDa ghi model MuJoCo: {urdf_out}")
        return

    # Nap muon (chi khi that su mo viewer). Dat ten khac `mujoco` -- neu viet
    # `import mujoco.viewer` o day thi Python coi `mujoco` la bien LOCAL cua
    # main() nen moi tham chieu mujoco.* PHIA TREN se UnboundLocalError.
    from mujoco import viewer as mj_viewer

    print()
    print("=" * 74)
    print("4. VIEWER -- phat lai quy dao (SPACE de tam dung, ESC de thoat)")
    print("=" * 74)
    print(f"  {TRANSITION_TIME}s dau: khong co gi (mo phong bat dau ngay tai diem dau)")
    print(f"  {DT * len(q_draw):.1f}s sau: ve chu O, {len(q_draw)} diem")
    print("  Cham do = vet dau but (pen_tip), ve dan theo quy dao")

    # Phat lai KINEMATIC: ghi truc tiep qpos roi mj_forward. Khong step vat ly,
    # nen thay dung duong di hinh hoc ma controller se yeu cau -- tach biet han
    # voi cau hoi "driver co bam duoc khong" (do la viec cua tune MIT tren that).
    with mj_viewer.launch_passive(model, data) as viewer:
        viewer.user_scn.ngeom = 0
        t_start = time.time()
        while viewer.is_running():
            t = ((time.time() - t_start) * args.speed) % times[-1]
            for i, dof in enumerate(mj_idx):
                data.qpos[dof] = np.interp(t, times, q_full[:, i])
                data.qvel[dof] = 0.0
            mujoco.mj_forward(model, data)

            # Vet but: them 1 cham moi cho tung diem quy dao da di qua.
            n_done = int(np.searchsorted(times, t))
            n_done = min(n_done, viewer.user_scn.maxgeom)
            if n_done != viewer.user_scn.ngeom:
                viewer.user_scn.ngeom = n_done
                for k in range(n_done):
                    g = viewer.user_scn.geoms[k]
                    mujoco.mjv_initGeom(
                        g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.004, 0, 0]),
                        fk_pts[min(k, len(fk_pts) - 1)], np.eye(3).flatten(),
                        np.array([1.0, 0.15, 0.15, 1.0], dtype=np.float32))

            viewer.sync()
            time.sleep(0.002)


if __name__ == "__main__":
    main()
