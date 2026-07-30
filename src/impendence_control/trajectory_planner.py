#!/usr/bin/env python3
"""
上位机轨迹规划节点 (Trajectory Planner Node)

功能：
  给定某个关节的起始位置、结束位置和运动时间，通过三次样条插值生成平滑轨迹，
  将插值后的控制帧以 /joint_command 话题发送给导纳控制节点 (admittance_calculate.py)。

三次插值（Cubic Spline）保证：
  - 位置连续
  - 速度连续（起始/结束速度为 0）
  - 加速度连续

使用方式（命令行参数）：
  ros2 run <package> trajectory_planner --ros-args \
    -p joint_name:="right_elbow_pitch" \
    -p start_pos:=10.0 \
    -p end_pos:=30.0 \
    -p duration:=3.0 \
    -p frequency:=50.0

也可以在启动后在另一个终端通过 ROS2 service 动态触发轨迹。
"""

import rclpy
import json
import time
import math
import sys
from rclpy.node import Node
from std_msgs.msg import String


class TrajectoryPlanner(Node):
    """轨迹规划节点 - 三次样条插值，发布 /joint_command"""

    def __init__(self):
        super().__init__('trajectory_planner')

        # ========== 可配置参数 ==========
        self.declare_parameter('joint_name', 'right_elbow_pitch')
        self.declare_parameter('start_pos', 0.0)
        self.declare_parameter('end_pos', 10.0)
        self.declare_parameter('duration', 3.0)        # 运动时间 (秒)
        self.declare_parameter('frequency', 50.0)      # 控制频率 (Hz)
        self.declare_parameter('only_legs', False)     # 是否仅下发腿部关节
        self.declare_parameter('auto_start', True)     # 启动后是否自动执行

        self.joint_name = self.get_parameter('joint_name').value
        self.start_pos = self.get_parameter('start_pos').value
        self.end_pos = self.get_parameter('end_pos').value
        self.duration = self.get_parameter('duration').value
        self.frequency = self.get_parameter('frequency').value
        self.only_legs = self.get_parameter('only_legs').value
        self.auto_start = self.get_parameter('auto_start').value

        # ========== 关节名称列表（与 admittance_calculate.py 保持一致，共 28 个） ==========
        self.joint_names = [
            'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw',
            'left_knee_pitch', 'left_ankle_yaw', 'left_ankle_pitch',
            'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw',
            'right_knee_pitch', 'right_ankle_yaw', 'right_ankle_pitch',
            'waist_yaw', 'waist_pitch', 'waist_roll',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow_pitch', 'left_wrist_yaw',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow_pitch', 'right_wrist_yaw',
            'neck_yaw', 'neck_roll', 'neck_pitch'
        ]

        # 默认关节角度（28个关节的零位/当前位）
        self.default_angles = [0.0] * 28

        # ========== 轨迹计算相关 ==========
        self.trajectory_coeffs = None   # 三次多项式系数 (a0, a1, a2, a3)
        self.trajectory_active = False
        self.trajectory_start_time = 0.0
        self.frame_count = 0

        # ========== 发布者 ==========
        self.joint_command_pub = self.create_publisher(
            String,
            '/joint_command',
            10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 轨迹规划节点已启动')
        self.get_logger().info(f'  目标关节: {self.joint_name}')
        self.get_logger().info(f'  起始位置: {self.start_pos:.2f}°')
        self.get_logger().info(f'  结束位置: {self.end_pos:.2f}°')
        self.get_logger().info(f'  运动时间: {self.duration:.2f} s')
        self.get_logger().info(f'  控制频率: {self.frequency:.1f} Hz')
        self.get_logger().info(f'  only_legs: {self.only_legs}')
        self.get_logger().info(f'  自动启动: {self.auto_start}')
        self.get_logger().info('=' * 60)

        # 自动启动
        if self.auto_start:
            self.get_logger().info('⏳ 3 秒后自动开始轨迹...')
            self._auto_start_timer = self.create_timer(3.0, self._delayed_start)

    def _delayed_start(self):
        """延迟启动回调（兼容老版本 ROS2 无 one_shot）"""
        self._auto_start_timer.cancel()
        self.start_trajectory()

    # ==================== 三次样条插值 ====================

    def compute_cubic_coefficients(self, start_pos: float, end_pos: float, duration: float):
        """
        计算三次多项式系数。

        三次多项式: p(t) = a0 + a1*t + a2*t^2 + a3*t^3

        边界条件:
          p(0)       = start_pos     (起始位置)
          p(duration) = end_pos      (结束位置)
          p'(0)       = 0            (起始速度为零)
          p'(duration) = 0           (结束速度为零)

        求解得到:
          a0 = start_pos
          a1 = 0
          a2 = 3 * (end_pos - start_pos) / duration^2
          a3 = -2 * (end_pos - start_pos) / duration^3

        Returns:
            tuple: (a0, a1, a2, a3)
        """
        delta = end_pos - start_pos
        T = duration

        a0 = start_pos
        a1 = 0.0
        a2 = 3.0 * delta / (T * T)
        a3 = -2.0 * delta / (T * T * T)

        return (a0, a1, a2, a3)

    def evaluate_cubic(self, coeffs, t: float) -> float:
        """计算三次多项式在时刻 t 的值。"""
        a0, a1, a2, a3 = coeffs
        return a0 + a1 * t + a2 * t * t + a3 * t * t * t

    def evaluate_cubic_velocity(self, coeffs, t: float) -> float:
        """计算三次多项式在时刻 t 的速度（一阶导数）。"""
        _, a1, a2, a3 = coeffs
        return a1 + 2.0 * a2 * t + 3.0 * a3 * t * t

    # ==================== 轨迹执行 ====================

    def start_trajectory(self):
        """开始执行轨迹。"""
        if self.joint_name not in self.joint_names:
            self.get_logger().error(f'关节 "{self.joint_name}" 不在关节列表中！')
            self.get_logger().error(f'可用关节: {self.joint_names}')
            return

        self.trajectory_coeffs = self.compute_cubic_coefficients(
            self.start_pos, self.end_pos, self.duration
        )
        a0, a1, a2, a3 = self.trajectory_coeffs
        self.get_logger().info(
            f'📐 三次多项式系数: '
            f'a0={a0:.4f}, a1={a1:.4f}, a2={a2:.4f}, a3={a3:.4f}'
        )

        self.trajectory_active = True
        self.trajectory_start_time = time.time()
        self.frame_count = 0

        # 以指定频率发布控制帧
        period = 1.0 / self.frequency
        self.trajectory_timer = self.create_timer(period, self.publish_trajectory_frame)

        self.get_logger().info(
            f'▶️  轨迹开始! {self.joint_name}: '
            f'{self.start_pos:.2f}° → {self.end_pos:.2f}°, '
            f'时长 {self.duration:.2f}s, 频率 {self.frequency:.1f}Hz'
        )

    def publish_trajectory_frame(self):
        """发布一帧轨迹控制命令。"""
        if not self.trajectory_active:
            return

        elapsed = time.time() - self.trajectory_start_time

        # 轨迹结束判定
        if elapsed >= self.duration:
            # 发送最终位置
            self._send_joint_command(self.end_pos, final=True)
            self.get_logger().info(
                f'✅ 轨迹完成! 共发送 {self.frame_count + 1} 帧, '
                f'实际耗时 {elapsed:.3f}s'
            )
            self.trajectory_active = False
            self.trajectory_timer.cancel()
            return

        # 计算当前期望位置
        desired_pos = self.evaluate_cubic(self.trajectory_coeffs, elapsed)
        desired_vel = self.evaluate_cubic_velocity(self.trajectory_coeffs, elapsed)

        self._send_joint_command(desired_pos)
        self.frame_count += 1

        # 每 10 帧打印一次进度
        if self.frame_count % 10 == 0:
            progress = (elapsed / self.duration) * 100.0
            self.get_logger().info(
                f'📤 帧 #{self.frame_count} | '
                f'时间: {elapsed:.3f}s ({progress:.1f}%) | '
                f'位置: {desired_pos:.3f}° | '
                f'速度: {desired_vel:.3f}°/s'
            )

    def _send_joint_command(self, target_pos: float, final: bool = False):
        """构造并发送 /joint_command 消息。"""
        # 复制默认角度，仅修改目标关节
        joint_angles = self.default_angles.copy()

        joint_index = self.joint_names.index(self.joint_name)
        joint_angles[joint_index] = target_pos

        command_data = {
            'joint_angles': joint_angles,
            'only_legs': self.only_legs,
            'source': 'trajectory_planner',
            'target_joint': self.joint_name,
            'target_position': target_pos,
            'final': final,
        }

        msg = String()
        msg.data = json.dumps(command_data)
        self.joint_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('轨迹规划节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
