cmake_minimum_required(VERSION 3.8)
project(gim_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Tìm các package ROS 2 phụ thuộc
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 1b. Pinocchio -- dùng để tính g(q) bù trọng lực cho MIT mode. Cài qua
# 'sudo apt install ros-humble-pinocchio' nếu chưa có.
#
# Pinocchio CÓ export target hiện đại 'pinocchio::pinocchio' (xem
# /opt/ros/humble/lib/x86_64-linux-gnu/cmake/pinocchio/pinocchioTargets.cmake).
# BẮT BUỘC link qua target này, KHÔNG dùng biến pinocchio_INCLUDE_DIRS/
# pinocchio_LIBRARIES kiểu cũ: target mang theo cả bộ macro tiền xử lý mà
# Pinocchio cần và ta không thể tự đoán đúng -- xem mục 2b.
find_package(pinocchio REQUIRED)

# 2. Biên dịch file C++ thành thư viện động (Shared Library)
add_library(gim_arm_system_hardware SHARED
  src/gim_arm_system.cpp
)

# 2b. KHÔNG tự định nghĩa BOOST_MPL_LIMIT_* ở đây.
#
# pinocchio::JointModel là 1 boost::variant gộp ~24 kiểu khớp, vượt giới hạn
# mặc định 20 của Boost::MPL. Nhưng chỉ nới BOOST_MPL_LIMIT_LIST_SIZE là VÔ
# TÁC DỤNG: Boost mặc định dùng header đã tiền xử lý sẵn (list10/20/30/40.hpp),
# nên giới hạn thật không đổi và vẫn lỗi
# "using invalid field boost::variant<...>::storage_".
# Phải kèm BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS để Boost sinh macro động,
# cộng BOOST_FUSION_INVOKE_MAX_ARITY=12 cho các joint visitor của Pinocchio.
#
# Target pinocchio::pinocchio đã mang đủ cả 4 macro đó (cùng
# PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION -- phải khớp với lúc build thư viện
# nếu không sẽ lỗi link/ABI). Cứ link target là xong, đừng chép tay.

# 3. Trỏ đường dẫn include (pinocchio + Eigen tự đến từ target ở mục 4)
target_include_directories(gim_arm_system_hardware PRIVATE
  include
)

# 4. Liên kết thư viện
ament_target_dependencies(gim_arm_system_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)
# Dùng chữ ký "plain" (không PUBLIC/PRIVATE) vì ament_target_dependencies ở
# trên cũng gọi target_link_libraries kiểu plain -- trộn 2 chữ ký trên cùng
# 1 target là lỗi CMake.
target_link_libraries(gim_arm_system_hardware
  pinocchio::pinocchio
)

# 5. Khai báo file Plugin XML cho ROS 2
pluginlib_export_plugin_description_file(hardware_interface gim_arm_hardware.xml)

# 6. Quy tắc Install (Đưa các file vào thư mục install sau khi build)
install(
  TARGETS gim_arm_system_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES gim_arm_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# 7. Xuất các thông tin cho các package khác dùng (nếu cần)
ament_export_include_directories(include)
ament_export_libraries(gim_arm_system_hardware)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

ament_package()