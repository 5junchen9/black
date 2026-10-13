cmake_minimum_required(VERSION 3.8)
project(robot_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 根据消息定义生成 C++ 和 Python 可使用的类型。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/RobotCommand.msg"
  "msg/MotorState.msg"
  "msg/RobotState.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
