#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS2 节点：运行 MuJoCo，并发布关节状态。"""

import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]


class MujocoSimNode(Node):
    """订阅关节力矩，运行仿真，发布关节角度和速度。"""

    def __init__(self, name):
        super().__init__(name)

        package_path = get_package_share_directory("black_stand")
        model_path = package_path + "/description/main.xml"
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 根节点位置、姿态 + 12 个关节的初始趴下角度
        # 初始悬空，让机器人先自然落地。
        self.data.qpos[:] = [
            0, 0, 0.45, 1, 0, 0, 0,
            0, 0.8, -1.6,
            0, -0.8, 1.6,
            0, -0.8, 1.6,
            0, 0.8, -1.6,
        ]
        mujoco.mj_forward(self.model, self.data)

        self.sub = self.create_subscription(
            Float64MultiArray, "joint_torque_cmd", self.command_callback, 1
        )
        self.pub = self.create_publisher(JointState, "joint_states", 1)
        self.odom_pub = self.create_publisher(Odometry, "base_odom", 1)

    def command_callback(self, msg):
        # 力矩顺序：FL、FR、RR、RL，每条腿依次 hip、thigh、calf。
        self.data.ctrl[:] = msg.data

    def publish_joint_state(self):
        stamp = self.get_clock().now().to_msg()

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = JOINT_NAMES
        msg.position = self.data.qpos[7:].tolist()
        msg.velocity = self.data.qvel[6:].tolist()
        self.pub.publish(msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base"
        odom.pose.pose.position.x = self.data.qpos[0]
        odom.pose.pose.position.y = self.data.qpos[1]
        odom.pose.pose.position.z = self.data.qpos[2]
        odom.pose.pose.orientation.w = self.data.qpos[3]
        odom.pose.pose.orientation.x = self.data.qpos[4]
        odom.pose.pose.orientation.y = self.data.qpos[5]
        odom.pose.pose.orientation.z = self.data.qpos[6]
        odom.twist.twist.linear.x = self.data.qvel[0]
        odom.twist.twist.linear.y = self.data.qvel[1]
        odom.twist.twist.linear.z = self.data.qvel[2]
        odom.twist.twist.angular.x = self.data.qvel[3]
        odom.twist.twist.angular.y = self.data.qvel[4]
        odom.twist.twist.angular.z = self.data.qvel[5]
        self.odom_pub.publish(odom)

    def step(self):
        mujoco.mj_step(self.model, self.data)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode("stand_mujoco_sim")

    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            step_period = node.model.opt.timestep
            publish_period = step_period
            render_period = 1.0 / 60.0
            next_step = time.monotonic()
            next_publish = node.data.time
            next_render = next_step

            while rclpy.ok() and viewer.is_running():
                # 控制器随 500 Hz 物理状态更新力矩；仿真不再逐步等待 ROS 往返。
                rclpy.spin_once(node, timeout_sec=0.0)
                now = time.monotonic()

                # 按真实时间补齐物理步，渲染变慢时也不会拖慢仿真时间。
                steps = 0
                while now >= next_step and steps < 10:
                    node.step()
                    next_step += step_period
                    steps += 1
                    if node.data.time >= next_publish:
                        node.publish_joint_state()
                        next_publish += publish_period

                if now >= next_render:
                    viewer.sync()
                    next_render = now + render_period

                sleep_time = min(next_step, next_render) - time.monotonic()
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
