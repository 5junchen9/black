# ROS 2 / MuJoCo C++ 学习工程

包含 `robot_interface` 消息包和 `robot_test` 节点包，使用 ROS 2 Humble、C++17、MuJoCo 和 GLFW。

## 当前功能

- `robot_mujoco_sim` 加载模型、计算关节 PD 力矩、运行仿真并发布 12 个电机状态。
- `robot_stand_pub` 收到反馈后每 10ms 发布 12 个电机的 `{kp,kd,q,w,tau}`。
- 默认速度阻尼；PS5 × 从当前角度用 0.5 秒平滑过渡到站姿并保持；○ 返回速度阻尼。
- 左摇杆输入仅保存，尚未实现行走。

阻尼不指定趴下姿态。当前起身仍是单段插值，未加入准备姿态、积分补偿或分阶段力矩限幅。IMU 和实际力矩反馈也尚未完成。

## 本机依赖

`src/robot_test/CMakeLists.txt` 当前使用以下绝对路径，换电脑时需要修改：

- MuJoCo：`/home/wujuncheng/black/.venv/lib/python3.10/site-packages/mujoco`，动态库为 `libmujoco.so.3.14.0`。
- 模型：`/home/wujuncheng/black/src/black_stand/description/main.xml`，引用旧工程中的模型和网格。

## 编译和运行

在本目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_test
source install/setup.bash
```

分别在三个终端运行，每个终端先加载 ROS 和工作空间环境：

```bash
ros2 run robot_test robot_mujoco_sim
```

```bash
ros2 run robot_test robot_stand_pub
```

```bash
ros2 run joy game_controller_node --ros-args -p device_id:=0
```

PS5 按键使用 `game_controller_node` 的标准映射：`buttons[0]` 为 ×，`buttons[1]` 为 ○；以实际 `/joy` 消息为准。

## 验证边界

已完成编译和模拟消息通信检查。仿真中的重复起身稳定性仍在调试，不能将编译、消息检查当作站稳或真实硬件验收。
