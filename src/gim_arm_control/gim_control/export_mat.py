import os
import tempfile
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.io import savemat

# 1. Đọc file cache quỹ đạo mà lqi_node vừa tạo ra
cache_file = os.path.join(tempfile.gettempdir(), "gim_lqi_way.npz")
data = np.load(cache_file)
q_way = data["q"]
dt = float(data["dt"])
n = len(q_way)

# 2. Nội suy Spline tuần hoàn (Giống hệt class PeriodicSpline của bạn)
ts = np.arange(n + 1) * dt
q_way_closed = np.vstack([q_way, q_way[:1]])
sp = CubicSpline(ts, q_way_closed, axis=0, bc_type="periodic")

# 3. Trích xuất với tần số 100Hz (0.01s) để khớp với control_hz
period = n * dt
t_fine = np.arange(0, period, 0.01)
q = sp(t_fine)
qd = sp(t_fine, 1)
qdd = sp(t_fine, 2)

# 4. Xuất thẳng ra định dạng của MATLAB
savemat('trajectory.mat', {'t': t_fine, 'q': q, 'qd': qd, 'qdd': qdd})
print("Thành công! Đã lưu file trajectory.mat tại thư mục hiện tại.")