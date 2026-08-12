import mujoco
import mujoco.viewer
import time
import re

# Đường dẫn file
urdf_path = "gim_arm.urdf"
urdf_mujoco_path = "gim_arm_mujoco.urdf"  # bản đã sửa đường dẫn mesh, MuJoCo đọc được
xml_path = "gim_arm.xml"

# 0. Sửa package://<tên_package>/meshes/... thành meshes/... -- MuJoCo không
# hiểu quy ước package:// của ROS, chỉ hiểu đường dẫn tương đối/tuyệt đối
# thường. Bước này BẮT BUỘC làm lại mỗi lần đồng bộ gim_arm.urdf mới từ ROS,
# không phải việc làm 1 lần.
with open(urdf_path, "r", encoding="utf-8") as f:
    content = f.read()
fixed_content = re.sub(r"package://[^/]+/meshes/", "meshes/", content)
with open(urdf_mujoco_path, "w", encoding="utf-8") as f:
    f.write(fixed_content)

# 1. Đọc file URDF (bản đã sửa đường dẫn) và biên dịch ngầm
print("Đang biên dịch URDF...")
try:
    model = mujoco.MjModel.from_xml_path(urdf_mujoco_path)
except Exception as e:
    print(f"Lỗi khi đọc URDF: {e}")
    exit()

# 2. Lưu lại thành file MJCF (.xml) để dùng cho sau này
# Định dạng XML của MuJoCo giúp bạn dễ dàng thêm Motor, Cảm biến, Ma sát...
mujoco.mj_saveLastXML(xml_path, model)
print(f"Đã tạo file cấu hình vật lý: {xml_path}")

# 3. Tạo dữ liệu mô phỏng (Data) từ Model
data = mujoco.MjData(model)

# 4. Mở cửa sổ Viewer tương tác
print("Đang mở MuJoCo Viewer... (Nhấn phím SPACE để Tạm dừng/Tiếp tục)")

# Dùng launch_passive để chạy cửa sổ ở một luồng riêng, code chính vẫn chạy
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Vòng lặp mô phỏng
    while viewer.is_running():
        step_start = time.time()

        # Tính toán vật lý tiến lên 1 bước (thường là 2ms)
        mujoco.mj_step(model, data)

        # Cập nhật hình ảnh lên màn hình
        viewer.sync()

        # Đảm bảo mô phỏng chạy đúng với thời gian thực (Real-time)
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)