# black_stand

ROS 2 Humble 与 MuJoCo 四足机器人站立仿真。

## 构建

```bash
cd ~/black
source .venv/bin/activate
source /opt/ros/humble/setup.bash
python /usr/bin/colcon build --packages-select black_stand --symlink-install
```

## 运行

```bash
cd ~/black
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch black_stand stand.launch.py
```

仿真启动后，机器人先在零力矩状态下自由落地。检测到连续稳定 2 秒后，控制节点开始执行站立控制。

> 当前控制参数仅针对仓库中的 MuJoCo 模型和场景验证，不应直接用于实体机器人。
