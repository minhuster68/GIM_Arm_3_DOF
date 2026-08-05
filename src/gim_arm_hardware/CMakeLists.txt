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

# 2. Biên dịch file C++ thành thư viện động (Shared Library)
add_library(gim_arm_system_hardware SHARED
  src/gim_arm_system.cpp
)

# 3. Trỏ đường dẫn include
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