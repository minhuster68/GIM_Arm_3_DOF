"""
shapes.py — tham số hoá quỹ đạo hình học (task #5). Mỗi hình là 1 hàm
path(t) -> np.array([x, y, z]), t chạy trong [0, 1].

Mặt phẳng vẽ: vuông góc trục `plane` (một trong 'x','y','z'), cố định tại
`plane_value`. Khớp đúng cách scan_workspace.py đã dùng để chọn vùng an toàn.
"""

import numpy as np


def letter_o(center, radius, plane="x", plane_value=0.2):
    """
    Chữ O: 1 vòng tròn đầy đủ, đi ngược chiều kim đồng hồ từ góc 0.
    center: (c0, c1) toạ độ tâm trong 2 trục CÒN LẠI (không phải trục `plane`)
    radius: bán kính (m)
    """

    def path(t):
        theta = 2 * np.pi * t
        c0 = center[0] + radius * np.cos(theta)
        c1 = center[1] + radius * np.sin(theta)
        return _assemble(plane, plane_value, c0, c1)

    return path


def ellipse(center, radius_a, radius_b, plane="x", plane_value=0.2, rotation=0.0):
    """
    Hình elip đầy đủ, đi ngược chiều kim đồng hồ từ góc 0.
    center: (c0, c1) tâm elip trong 2 trục còn lại
    radius_a: bán trục theo hướng "0" (trước khi xoay)
    radius_b: bán trục theo hướng "1" (trước khi xoay)
    rotation: góc xoay elip (rad) quanh tâm, cho phép nghiêng elip tuỳ ý
    """

    def path(t):
        theta = 2 * np.pi * t
        x_local = radius_a * np.cos(theta)
        y_local = radius_b * np.sin(theta)
        c0 = center[0] + x_local * np.cos(rotation) - y_local * np.sin(rotation)
        c1 = center[1] + x_local * np.sin(rotation) + y_local * np.cos(rotation)
        return _assemble(plane, plane_value, c0, c1)

    return path


def letter_s(center, width, height, plane="x", plane_value=0.2):
    """
    Chữ S: 2 cung tròn bán kính bằng nhau (r = width/2), cong ngược chiều
    nhau, nối liền mạch tại đúng 1 điểm chung (đã kiểm chứng bằng đại số,
    không chỉ gần đúng). Chiều cao co giãn độc lập với width bằng cách scale
    trục dọc sau khi dựng hình cơ bản (chiều cao tự nhiên = 4r).
    center: (c0, c1) tâm hình chữ nhật bao quanh chữ S
    width: khoảng cách bụng cong rộng nhất (m)
    height: chiều cao tổng thể mong muốn (m)
    """
    r = width / 2.0
    top_center = (center[0], center[1] + r)
    bot_center = (center[0], center[1] - r)
    natural_height = 4 * r
    scale = height / natural_height if natural_height > 0 else 1.0

    def path(t):
        if t <= 0.5:
            tau = t / 0.5
            theta = np.pi / 2 - tau * np.pi
            c0 = top_center[0] + r * np.cos(theta)
            c1 = top_center[1] + r * np.sin(theta)
        else:
            tau = (t - 0.5) / 0.5
            theta = np.pi / 2 - tau * np.pi
            c0 = bot_center[0] - r * np.cos(theta)
            c1 = bot_center[1] + r * np.sin(theta)
        c1 = center[1] + (c1 - center[1]) * scale  # co giãn chiều cao độc lập
        return _assemble(plane, plane_value, c0, c1)

    return path


def ellipse_3d(center, axis_u, axis_v, radius_u, radius_v, bulge=0.0,
               bulge_harmonic=2, phase=0.0):
    """
    Ellipse nằm trong 1 mặt phẳng BẤT KỲ của không gian 3D (khác letter_o/
    ellipse ở trên: 2 hàm đó bắt buộc mặt phẳng song song mặt phẳng toạ độ).

    center: (x,y,z) tâm ellipse trong hệ world (m)
    axis_u, axis_v: 2 vector chỉ hướng bán trục. axis_v được trực giao hoá
        (Gram-Schmidt) so với axis_u, nên chỉ cần đưa hướng gần đúng; cả hai
        đều được chuẩn hoá độ dài -> radius_u/radius_v mới là kích thước thật.
    radius_u, radius_v: bán trục (m) theo û và v̂
    bulge: biên độ "phình" ra khỏi mặt phẳng, theo pháp tuyến n̂ = û × v̂ (m).
        =0 -> ellipse phẳng. Khác 0 -> quỹ đạo 3D khép kín.
    bulge_harmonic: số chu kỳ phình trên 1 vòng. =2 -> phình cực đại tại 2 đầu
        trục û và lõm tại 2 đầu trục v̂ (dùng bulge âm để đảo lại).
    phase: lệch pha điểm xuất phát (rad)
    """
    center = np.asarray(center, dtype=float)
    u = np.asarray(axis_u, dtype=float)
    u = u / np.linalg.norm(u)
    v = np.asarray(axis_v, dtype=float)
    v = v - np.dot(v, u) * u  # trực giao hoá: bỏ thành phần trùng û
    norm_v = np.linalg.norm(v)
    if norm_v < 1e-9:
        raise ValueError("axis_v song song với axis_u -> không dựng được mặt phẳng")
    v = v / norm_v
    n = np.cross(u, v)

    def path(t):
        theta = 2 * np.pi * t + phase
        return (center
                + radius_u * np.cos(theta) * u
                + radius_v * np.sin(theta) * v
                + bulge * np.cos(bulge_harmonic * theta) * n)

    return path


def shoulder_sweep(pivot, radius, az_center_deg, el_center_deg, az_amp_deg,
                   el_amp_deg, radius_amp=0.0, radius_harmonic=2,
                   forward=(0.0, 1.0, 0.0), up=(0.0, 0.0, 1.0), phase=0.0):
    """
    Quỹ đạo ellipse QUÉT THEO GÓC quanh tâm vai, bám theo vỏ cầu tầm với.

    Vì sao cần hàm này (đã đo bằng scan workspace, không phải suy đoán): tay
    3 DOF này với chốt khuỷu có giới hạn nên vùng với được là 1 LỚP VỎ CẦU
    quanh tâm vai (r ~ 0.39..0.60 m), KHÔNG phải khối đặc. Ellipse phẳng biên
    độ lớn (letter_o/ellipse/ellipse_3d) luôn chọc thủng vỏ đó ở 2 đầu trục
    dài -> IK không giải nổi. Quét theo GÓC giữ nguyên độ vươn nên biên độ
    ngang có thể lớn gấp nhiều lần mà vẫn nằm gọn trong vùng không singular.

    pivot: (x,y,z) tâm quay vai trong hệ world (m)
    radius: độ vươn tay (m) -- khoảng cách từ vai tới đầu tay
    az_center_deg / az_amp_deg: tâm và biên độ góc quét NGANG (độ).
        az=0 là hướng `forward`, az dương lệch về phía forward × up.
    el_center_deg / el_amp_deg: tâm và biên độ góc quét DỌC (độ),
        el>0 cao hơn vai, el<0 thấp hơn vai.
    radius_amp: biên độ "thở" của độ vươn (m) -- cho khớp khuỷu cũng có biên
        độ thay vì đứng yên. =0 -> tay giữ nguyên độ vươn suốt vòng.
    radius_harmonic: số chu kỳ thở trên 1 vòng (=2: vươn xa nhất ở 2 đầu biên
        ngang, co lại ở 2 đầu biên dọc).
    forward, up: hệ quy chiếu người đeo. Mặc định người ngồi nhìn theo +Y,
        đỉnh đầu theo +Z -> right = forward × up = +X.
    """
    pivot = np.asarray(pivot, dtype=float)
    f = np.asarray(forward, dtype=float)
    f = f / np.linalg.norm(f)
    u = np.asarray(up, dtype=float)
    u = u - np.dot(u, f) * f
    u = u / np.linalg.norm(u)
    right = np.cross(f, u)

    def path(t):
        theta = 2 * np.pi * t + phase
        az = np.radians(az_center_deg + az_amp_deg * np.cos(theta))
        el = np.radians(el_center_deg + el_amp_deg * np.sin(theta))
        r = radius + radius_amp * np.cos(radius_harmonic * theta)
        direction = (np.cos(el) * (np.cos(az) * f + np.sin(az) * right)
                     + np.sin(el) * u)
        return pivot + r * direction

    return path


def discretize(path_fn, n_points=50, close_loop=False):
    """Rời rạc hoá path(t) thành list các điểm (x,y,z) numpy array."""
    if close_loop:
        ts = np.linspace(0, 1, n_points, endpoint=False)
    else:
        ts = np.linspace(0, 1, n_points)
    return [path_fn(t) for t in ts]


def _assemble(plane, plane_value, c0, c1):
    if plane == "x":
        return np.array([plane_value, c0, c1])
    elif plane == "y":
        return np.array([c0, plane_value, c1])
    elif plane == "z":
        return np.array([c0, c1, plane_value])
    raise ValueError(f"plane phải là 'x', 'y', hoặc 'z', nhận được '{plane}'")