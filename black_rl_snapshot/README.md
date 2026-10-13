# Black 强化学习控制版本

这里保存 2026-10-13 当前实际使用的源码、模型和场地，以及自定义 ROS 消息。它是对上游 rl_sar 的修改文件集合，不是另一个完整 rl_sar 仓库。原机器人 XML 和网格已经保存在本仓库的 `src/black_stand/description/`。

## 已保存的内容

- `rl_sar/src/rl_sar/src/black_mujoco_sim.cpp`：物理仿真、关节状态和 IMU 发布、PD 力矩计算。
- `rl_sar/src/rl_sar/src/rl_black_controller.cpp`：观测、六帧历史、模型推理、关节映射和手柄控制。
- `rl_sar/src/rl_sar/CMakeLists.txt`、`package.ros2.xml`：当前构建配置。
- `rl_sar/policy/black/robot_lab/best.pt`：实际加载的 TorchScript 模型。
- `scene_field.xml`：障碍练习场地。
- `robot_interface/`：当前消息包的源码。

## 当前行为

控制命令计划每 5 ms 发布一次，模型每四个控制周期推理一次。模型输入为六帧、每帧 45 项，共 270 项；输出为 12 个关节动作。策略顺序为 FL、FR、RL、RR，仿真消息顺序为 FL、FR、RR、RL。目标角度为默认角度加 0.25 倍模型动作。

buttons[0] 起立，buttons[1] 退出策略进入阻尼，buttons[2] 进入策略。第四键未分配。左摇杆控制前进和横移，幅度 0.5 m/s；右摇杆 axes[3] 控制转向，幅度 0.8 rad/s。具体物理按钮和轴排列以当前 DualSense 与 joy_node 的映射为准。

控制参数目前直接写在 C++ 中。草稿 base.yaml 未读取，未包含在这份快照里。

## 恢复到上游工程

上游基线：`fan-ziqi/rl_sar` 的 `376d42c9b128f963ab08579762d5a216a976ce39`。当前修改后的本地提交是 `6865448`。恢复时先准备这个基线的完整工程及其依赖，然后用本目录 `rl_sar/` 内的文件覆盖对应路径。保留上游许可证。

以下路径按当前机器说明，不要在现有工作目录重复克隆或覆盖自己的未提交修改：

1. 将完整上游工程放在 `/home/wujuncheng/black/rl_sar`，初始化它的 Git 子模块。
2. 将本快照 `rl_sar/` 中的文件复制到对应位置。
3. 把本仓库 `src/black_stand/description/` 的内容复制到 `rl_sar/src/rl_sar_zoo/black_description/mjcf/`，再把本目录的 `scene_field.xml` 放进去。
4. 把 `robot_interface/` 放到 ROS 工作空间中，先用 colcon 编译，再 source 该工作空间的 install/setup.bash。当前使用的是 `/home/wujuncheng/ros2_task_ws`。
5. 准备 ROS 2 Humble、GLFW、MuJoCo 和上游所需的推理依赖，运行 `./build.sh rl_sar`。

MuJoCo 当前位于 `/home/wujuncheng/black/.venv/lib/python3.10/site-packages/mujoco`，链接版本为 3.14.0。best.pt 的加载路径也是本机绝对路径。换机器后需要修改 CMake 和控制器里的对应路径。

## 运行

每个运行终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/wujuncheng/ros2_task_ws/install/setup.bash
source /home/wujuncheng/black/rl_sar/install/setup.bash
```

分别在三个终端运行：

```bash
ros2 run rl_sar black_mujoco_sim
ros2 run joy joy_node
ros2 run rl_sar rl_black_controller
```

只运行一个电机命令控制器，停止原来的 robot_stand_pub 后再启动新控制器。

## 验证范围

`./build.sh rl_sar` 完整编译通过；模型加载已检查。新场景经过 MuJoCo 加载、100 步无控制推进和离屏画面检查。持续行走和越障效果仍需运行演示验证；编译成功不代表策略能够跨越所有障碍。
