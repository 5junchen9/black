#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS2 节点：订阅关节状态，发布站立所需的关节力矩。"""

import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class StandControllerNode(Node):
    """使用 PD 控制，让 12 个关节保持目标站立姿态。"""

    def __init__(self, name):
        super().__init__(name)

        # 顺序：FL、FR、RR、RL，每条腿依次 hip、thigh、calf。
        self.stand_pos = np.array([
            0.1, 0.7, -1.2,    # FL
            -0.1, -0.7, 1.2,   # FR
            0.1, -0.7, 1.2,    # RR
            -0.1, 0.7, -1.2,   # RL
        ])

        self.standing = False
        self.stand_start_time = None
        self.getup_start_pos = None
        self.stable_since = None
        self.hold_stable_since = None
        self.integral_enabled = False
        self.base_linear_speed = float("inf")
        self.base_angular_speed = float("inf")
        self.kp = np.array([35, 45, 45] * 4)
        self.ki = np.array([
            1, 3, 3,
            1, 3, 3,
            3, 6, 6,
            3, 6, 6,
        ])
        self.integral_torque_limit = np.array([
            6, 10, 10,
            6, 10, 10,
            10, 15, 15,
            10, 15, 15,
        ])
        self.kd = np.array([2.5, 3, 3] * 4)
        self.integral_error = np.zeros(12)
        self.last_control_time = None

        self.pub = self.create_publisher(Float64MultiArray, "joint_torque_cmd", 1)
        self.sub = self.create_subscription(
            JointState, "joint_states", self.state_callback, 1
        )
        self.odom_sub = self.create_subscription(
            Odometry, "base_odom", self.odom_callback, 1
        )

    def odom_callback(self, msg):
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.base_linear_speed = np.linalg.norm([linear.x, linear.y, linear.z])
        self.base_angular_speed = np.linalg.norm([angular.x, angular.y, angular.z])

    def state_callback(self, msg):
        joint_pos = np.array(msg.position)
        joint_vel = np.array(msg.velocity)
        now = time.monotonic()

        if self.last_control_time is None:
            dt = 0.0
        else:
            dt = np.clip(now - self.last_control_time, 0.0, 0.02)
        self.last_control_time = now

        if not self.standing:
            stable = (
                np.max(np.abs(joint_vel)) < 0.1
                and self.base_linear_speed < 0.03
                and self.base_angular_speed < 0.1
            )

            if stable:
                if self.stable_since is None:
                    self.stable_since = now
                elif now - self.stable_since >= 2.0:
                    self.standing = True
                    self.stand_start_time = now
                    self.getup_start_pos = joint_pos.copy()
                    self.get_logger().info(
                        "自由落地后已连续稳定 2 秒，开始站立；当前关节角："
                        f"{np.round(joint_pos, 3).tolist()}"
                    )
            else:
                self.stable_since = None

        if not self.standing:
            self.integral_error[:] = 0.0
            command = Float64MultiArray()
            command.data = [0.0] * 12
            self.pub.publish(command)
            return

        # 0.5 秒内快速到达站姿，使用平滑曲线避免目标角瞬间跳变。
        elapsed = now - self.stand_start_time
        phase = np.clip(elapsed / 0.5, 0.0, 1.0)
        blend = phase * phase * (3.0 - 2.0 * phase)
        target_pos = (
            (1.0 - blend) * self.getup_start_pos + blend * self.stand_pos
        )

        error = target_pos - joint_pos

        # 先只用 PD 完成起身。整机连续稳定后再启用积分，避免机器人
        # 仍在运动时积分继续蓄力，造成突然抖动或弹跳。
        hold_stable = (
            elapsed >= 1.5
            and np.max(np.abs(joint_vel)) < 0.2
            and self.base_linear_speed < 0.05
            and self.base_angular_speed < 0.2
        )
        if not self.integral_enabled:
            if hold_stable:
                if self.hold_stable_since is None:
                    self.hold_stable_since = now
                elif now - self.hold_stable_since >= 0.5:
                    self.integral_enabled = True
                    self.get_logger().info("站姿已稳定，启用小幅积分补偿")
            else:
                self.hold_stable_since = None

        if self.integral_enabled:
            self.integral_error += error * dt
        else:
            self.integral_error[:] = 0.0
        integral_error_limit = self.integral_torque_limit / self.ki
        self.integral_error = np.clip(
            self.integral_error, -integral_error_limit, integral_error_limit
        )
        integral_torque = self.ki * self.integral_error

        torque = self.kp * error + integral_torque - self.kd * joint_vel

        # 先恢复已经验证能站稳的前后腿限幅。画面慢的问题在仿真循环，
        # 不能用继续增大力矩来补偿。
        if elapsed < 0.5:
            torque_limit = np.array([11.0] * 6 + [12.0] * 6)
        else:
            front_limit = min(4.0 + 16.0 * (elapsed - 0.5), 20.0)
            rear_limit = min(8.0 + 12.0 * (elapsed - 0.5), 20.0)
            torque_limit = np.array([front_limit] * 6 + [rear_limit] * 6)
        torque = np.clip(torque, -torque_limit, torque_limit)

        command = Float64MultiArray()
        command.data = torque.tolist()
        self.pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = StandControllerNode("stand_controller")

    try:
        rclpy.spin(node)
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
