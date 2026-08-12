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