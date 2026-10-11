cmake_minimum_required(VERSION 3.8)
project(robot_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(glfw3 REQUIRED)

# 本机的 MuJoCo 安装在 Python 虚拟环境中，直接使用它的头文件和动态库。
set(MUJOCO_DIR "/home/wujuncheng/black/.venv/lib/python3.10/site-packages/mujoco")

add_executable(robot_stand_pub robot_stand_pub.cpp)
ament_target_dependencies(robot_stand_pub rclcpp std_msgs robot_interface sensor_msgs)

add_executable(robot_mujoco_sim robot_mujoco_sim.cpp)
ament_target_dependencies(robot_mujoco_sim rclcpp std_msgs robot_interface sensor_msgs)
target_include_directories(robot_mujoco_sim PRIVATE "${MUJOCO_DIR}/include")
# ament_target_dependencies 使用不带 PRIVATE 的链接形式，这里保持一致。
target_link_libraries(robot_mujoco_sim
  "${MUJOCO_DIR}/libmujoco.so.3.14.0"
  glfw
)
# 安装后仍能找到虚拟环境中的 MuJoCo 动态库。
set_target_properties(robot_mujoco_sim PROPERTIES INSTALL_RPATH "${MUJOCO_DIR}")

# 使用现有完整场景；它引用的模型和网格保持在原目录中。
target_compile_definitions(robot_mujoco_sim PRIVATE
  MUJOCO_MODEL_PATH="/home/wujuncheng/black/src/black_stand/description/main.xml"
)

install(TARGETS robot_stand_pub robot_mujoco_sim
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
