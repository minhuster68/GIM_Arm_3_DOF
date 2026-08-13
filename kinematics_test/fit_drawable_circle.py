#!/usr/bin/env python3
"""
fit_drawable_circle.py -- tim vong tron LON NHAT thuc su ve duoc trong mot mat
phang ve, va kiem chung bang bo giai IK DOC LAP.

BOI CANH -- hai cai bay da mac phai truoc do, ghi lai de dung mac lai:

 (1) scan_workspace.py in ra BOUNDING BOX cua vung tot, va chinh no da canh bao
     "khong dam bao dac kin 100% ben trong". Vung voi toi duoc cua tay 3 DOF cat
     boi mot mat phang la mot DAI CONG HEP -- o x=0.2 no chi chiem ~13% dien
     tich bounding box. Chon tam chu O theo bounding box cho ra
     center=(-0.045, 0.70) r=0.07 trong draw_trajectory.py, thuc te chi 16/60
     diem la voi toi duoc. (DA SUA: draw_trajectory.py gio dung ung vien
     x=0.1 center=(-0.3021, 0.1447) r=0.025 do chinh file nay tim ra, dat
     60/60. Moc "HIEN TAI" trong main() da doi theo.)

 (2) Ban dau file nay danh dau o vuong tu luoi mau roi dung binh_closing de lap
     lo. Do la SAI: closing BIA RA vung dac tu cac diem thua. Ket qua bia ra la
     mat phang x=0.1 tam (-0.9883, 0.8001) r=0.06 -- hoan toan khong ve duoc.
     Va phan kiem chung lai dung GimArmKinematics.ik_position(), tuc chinh bo
     giai bi loi (chi hoi tu 158/200 tren diem chac chan kha thi, ket o goc hop
     gioi han khop 24/200 lan), nen no bao "60/60 khong hoi tu" cho CA hai
     phuong an va che mat viec de xuat cung sai.

CACH LAM DUNG (file nay):
 - Phu vung that, khong bia: voi moi cap (q_base, q_shoulder) tren luoi day,
   giai q_elbow sao cho x(q) = plane_value bang do dau + chia doi. Tap nghiem
   chinh la anh cua mat 2D {x = plane_value} trong khong gian khop, tuc dung
   vung voi toi duoc, phu day tu nhien.
 - Kiem chung DOC LAP: cham diem cuoi cung bang least_squares co bound =
   gioi han khop, nhieu diem khoi dau. Bo giai nay da kiem dinh 60/60 voi sai
   so 0.000 mm tren diem sinh tu FK cua tu the ngau nhien -- KHONG dung
   ik_position() cua repo cho viec kiem chung.

CHAY:  cd ~/gim60108_ws && source install/setup.bash
       python3 kinematics_test/fit_drawable_circle.py
"""

import os
import sys

import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, ".."))

try:
    from gim_control.gim_arm_kinematics import GimArmKinematics
    from gim_control.shapes import letter_o, discretize
except ImportError:
    sys.path.insert(0, os.path.join(WS, "src", "gim_arm_control"))
    from gim_control.gim_arm_kinematics import GimArmKinematics
    from gim_control.shapes import letter_o, discretize

URDF = os.path.join(WS, "src", "gim_arm_description", "urdf", "gim_arm.urdf")
TOOL_OFFSET = (0.4031, 0.049, -0.029)   # giu dong bo voi draw_trajectory.py
PLANES = [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40]
N_AB = 140          # luoi tren (q_base, q_shoulder)
N_ELBOW = 60        # so mau do dau tren q_elbow
CELL = 0.005        # o vuong 5mm cho distance transform
COND_MAX = 15.0     # nguong condition number, giong scan_workspace.py
N_POINTS = 60       # so diem roi rac tren chu O, nhu draw_trajectory.py


class Solver:
    def __init__(self):
        self.kin = GimArmKinematics(URDF, tool_offset_xyz=TOOL_OFFSET)
        self.lo = self.kin.model.lowerPositionLimit
        self.hi = self.kin.model.upperPositionLimit
        rng = np.random.default_rng(0)
        self.starts = [self.kin.mid_q] + [
            self.lo + (self.hi - self.lo) * rng.random(3) for _ in range(2)]

    def fk(self, q):
        return self.kin.fk_position(q)

    def ik(self, target, tol=1e-4):
        """IK THAM CHIEU doc lap voi ik_position() cua repo. Tra (dat?, q)."""
        best_q, best_e = None, np.inf
        for q0 in self.starts:
            r = least_squares(
                lambda q: self.kin.fk_position(q) - target,
                np.clip(q0, self.lo, self.hi), bounds=(self.lo, self.hi),
                xtol=1e-12, ftol=1e-12, gtol=1e-12)
            e = float(np.linalg.norm(r.fun))
            if e < best_e:
                best_q, best_e = r.x, e
            if best_e < tol:
                break
        return best_e < tol, best_q, best_e

    def self_test(self, n=40):
        """Kiem dinh bo giai tham chieu tren diem CHAC CHAN kha thi (sinh tu FK
        cua tu the ngau nhien hop le). Neu buoc nay khong 100% thi moi ket luan
        ben duoi deu vo nghia."""
        rng = np.random.default_rng(1)
        worst = 0.0
        n_ok = 0
        for _ in range(n):
            q = self.lo + (self.hi - self.lo) * rng.random(3)
            ok, _, e = self.ik(self.fk(q))
            n_ok += ok
            worst = max(worst, e)
        return n_ok, n, worst

    def plane_cloud(self, pv):
        """Diem (y,z) voi toi duoc trong mat phang x = pv, phu day KHONG bia.
        Voi moi (q_base, q_shoulder), do dau x(q) - pv theo q_elbow roi chia doi."""
        qa = np.linspace(self.lo[0], self.hi[0], N_AB)
        qb = np.linspace(self.lo[1], self.hi[1], N_AB)
        qe = np.linspace(self.lo[2], self.hi[2], N_ELBOW)
        pts, conds = [], []
        for a in qa:
            for b in qb:
                xs = np.array([self.fk([a, b, e])[0] for e in qe]) - pv
                sign_change = np.where(np.sign(xs[:-1]) != np.sign(xs[1:]))[0]
                for i in sign_change:
                    e0, e1 = qe[i], qe[i + 1]
                    f0 = xs[i]
                    for _ in range(30):            # chia doi
                        em = 0.5 * (e0 + e1)
                        fm = self.fk([a, b, em])[0] - pv
                        if np.sign(fm) == np.sign(f0):
                            e0, f0 = em, fm
                        else:
                            e1 = em
                    q = np.array([a, b, 0.5 * (e0 + e1)])
                    p = self.fk(q)
                    c = np.linalg.cond(self.kin.jacobian(q)[:3, :])
                    pts.append(p[1:])
                    conds.append(c)
        return np.array(pts).reshape(-1, 2), np.array(conds)


def inscribed_circle(pts):
    """Vong tron noi tiep lon nhat trong tap diem, KHONG lap lo (moi o chi dac
    khi that su co diem roi vao -- phu day la do trace o tren, khong do closing)."""
    if len(pts) < 50:
        return None
    p0 = pts.min(0)
    idx = ((pts - p0) / CELL).astype(int)
    grid = np.zeros(idx.max(0) + 3, bool)
    grid[idx[:, 0] + 1, idx[:, 1] + 1] = True
    dist = ndimage.distance_transform_edt(grid) * CELL
    i, j = np.unravel_index(dist.argmax(), dist.shape)
    return p0[0] + (i - 1) * CELL, p0[1] + (j - 1) * CELL, float(dist[i, j])


def main():
    s = Solver()
    n_ok, n, worst = s.self_test()
    print(f"Kiem dinh bo giai THAM CHIEU: {n_ok}/{n} dat, sai so xau nhat "
          f"{worst * 1000:.6f} mm")
    if n_ok != n:
        print("Bo giai tham chieu chua dat 100% -- dung lai, moi ket luan sau se sai.")
        return
    print()

    print(f"{'mat phang x':>12s} {'so diem':>9s} {'tam noi tiep (y, z)':>24s} {'r_max':>8s}")
    print("-" * 58)
    cands = []
    for pv in PLANES:
        pts, conds = s.plane_cloud(pv)
        good = pts[conds < COND_MAX] if len(pts) else pts
        res = inscribed_circle(good)
        if res is None:
            print(f"{pv:12.3f} {len(good):9d}   qua it diem tot")
            continue
        cy, cz, r = res
        print(f"{pv:12.3f} {len(good):9d}   ({cy:+.4f}, {cz:+.4f})".ljust(48)
              + f"{r:8.4f}")
        cands.append((r, pv, cy, cz))

    if not cands:
        print("Khong mat phang nao dung duoc.")
        return

    # Kiem chung tung ung vien bang bo giai THAM CHIEU tren dung 60 diem that.
    print()
    print("=" * 74)
    print(f"KIEM CHUNG bang bo giai tham chieu ({N_POINTS} diem that moi phuong an)")
    print("=" * 74)
    print("So sanh voi cau hinh HIEN TAI trong draw_trajectory.py truoc:")
    # Moc "HIEN TAI" phai GIU DONG BO voi draw_trajectory.py va replay_trajectory.py
    # (SHAPE_KW). Neu de so cu o day thi dong nay luon bao "khong dung duoc" bat ke
    # limit moi dung hay sai -- tuc mat luon tac dung tra loi cau hoi "quy dao dang
    # dung co con voi toi duoc sau khi doi <limit> khong".
    for label, kw in [("HIEN TAI", dict(center=(-0.3021, 0.1447), radius=0.025,
                                        plane="x", plane_value=0.1))] + [
        (f"x={pv} r={np.floor(r * f / 0.005) * 0.005:.3f}",
         dict(center=(round(cy, 4), round(cz, 4)),
              radius=float(np.floor(r * f / 0.005) * 0.005),
              plane="x", plane_value=pv))
        for (r, pv, cy, cz) in sorted(cands, reverse=True)[:3]
        for f in (0.85, 0.7)
        if np.floor(r * f / 0.005) * 0.005 > 0
    ]:
        tg = discretize(letter_o(**kw), n_points=N_POINTS, close_loop=True)
        qs, n_reach, worst_e = [], 0, 0.0
        for t in tg:
            ok, q, e = s.ik(t)
            n_reach += ok
            worst_e = max(worst_e, e)
            if ok:
                qs.append(q)
        tag = "" if n_reach == N_POINTS else "   <-- KHONG DUNG DUOC"
        print(f"\n  {label}: voi toi duoc {n_reach}/{N_POINTS}, "
              f"sai so xau nhat {worst_e * 1000:.3f} mm{tag}")
        print(f"    {kw}")
        if n_reach == N_POINTS:
            Q = np.array(qs)
            cs = [np.linalg.cond(s.kin.jacobian(q)[:3, :]) for q in Q]
            print(f"    cond(J) doc quy dao: max {max(cs):.1f} "
                  f"({'OK' if max(cs) < COND_MAX else 'GAN SINGULAR'})")
            for i, nm in enumerate(s.kin.joint_names):
                print(f"    {nm:16s} {Q[:, i].min():+.4f}..{Q[:, i].max():+.4f}  "
                      f"(du bien {Q[:, i].min() - s.lo[i]:.3f} / "
                      f"{s.hi[i] - Q[:, i].max():.3f} rad)")


if __name__ == "__main__":
    main()
