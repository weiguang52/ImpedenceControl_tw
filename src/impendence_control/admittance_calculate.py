# 导纳控制计算
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Tuple, Optional, Any, List
import math  
from rclpy.node import Node

CURRENT_MA_TO_A = 0.001
RDK_ACTION_ID = 1

# ========== 关节参数配置表（在这里定义所有关节的参数） ==========
JOINT_CONFIGS = {
    '''
    'joint': {
        'current_threshold': 0.03,              # 电流突变阈值 (A)
        'current_recovery_threshold': 0.015,    # 电流恢复阈值 (A)
        'current_to_torque_coeff': 1.2,        # 电流到力矩转换系数
        'expected_torque': 0.1,                 # 期望力矩 (N·m)
        'damping_coeff': 15.0,                  # 阻尼系数 (B)
        'stiffness_coeff': 120.0,               # 刚度系数 (K)
    },
    '''
    'right_shoulder_roll': {
        'current_threshold': 0.04,              # 电流突变阈值 (A)
        'current_recovery_threshold': 0.02,    # 电流恢复阈值 (A)
        'current_to_torque_coeff': 0.8,        # 电流到力矩转换系数
        'expected_torque': 0.0004,                 # 期望力矩 (N·m)
        'damping_coeff': 0.0001,                  # 阻尼系数 (B)
        'stiffness_coeff': 0.07,               # 刚度系数 (K)
    },#
    'right_shoulder_yaw': {
        'current_threshold': 0.02,
        'current_recovery_threshold': 0.01,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'right_elbow_pitch': {
        'current_threshold': 0.2,
        'current_recovery_threshold': 0.1,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.04,
    },#
    'right_wrist_yaw': {
        'current_threshold': 0.035,
        'current_recovery_threshold': 0.0175,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'left_shoulder_roll': {
        'current_threshold': 0.04,
        'current_recovery_threshold': 0.02,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'left_shoulder_yaw': {
        'current_threshold': 0.02,
        'current_recovery_threshold': 0.01,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'left_elbow_pitch': {
        'current_threshold': 0.02,
        'current_recovery_threshold': 0.01,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 20.0,
        'stiffness_coeff': 200.0,
    },#
    'left_wrist_yaw': {
        'current_threshold': 0.035,
        'current_recovery_threshold': 0.0175,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'right_shoulder_pitch': {
        'current_threshold': 0.02,
        'current_recovery_threshold': 0.01,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'left_shoulder_pitch': {
        'current_threshold': 0.02,
        'current_recovery_threshold': 0.01,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'neck_roll': {
        'current_threshold': 0.025,
        'current_recovery_threshold': 0.012,
        'current_to_torque_coeff': 1.0,
        'expected_torque': 0.08,
        'damping_coeff': 12.0,
        'stiffness_coeff': 100.0,
    },
    'neck_yaw': {
        'current_threshold': 0.04,
        'current_recovery_threshold': 0.02,
        'current_to_torque_coeff': 1.5,
        'expected_torque': 0.15,
        'damping_coeff': 20.0,
        'stiffness_coeff': 150.0,
    },
    'waist_pitch': {
        'current_threshold': 0.022,
        'current_recovery_threshold': 0.011,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.05,
        'damping_coeff': 10.0,
        'stiffness_coeff': 90.0,
    },
    'waist_roll': {
        'current_threshold': 0.025,
        'current_recovery_threshold': 0.012,
        'current_to_torque_coeff': 1.0,
        'expected_torque': 0.08,
        'damping_coeff': 12.0,
        'stiffness_coeff': 100.0,
    },
    'right_hip_pitch': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.005,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },
    'left_hip_pitch': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.005,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },
    'waist_yaw': {
        'current_threshold': 0.025,
        'current_recovery_threshold': 0.012,
        'current_to_torque_coeff': 1.0,
        'expected_torque': 0.08,
        'damping_coeff': 12.0,
        'stiffness_coeff': 100.0,
    },
    'right_hip_roll': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.04,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'right_hip_yaw': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.03,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'right_knee_pitch': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.04,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'right_ankle_yaw': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.02,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'right_ankle_pitch': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.025,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'left_hip_roll': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.04,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'left_hip_yaw': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.03,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'left_knee_pitch': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.04,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'left_ankle_yaw': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.02,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
    'left_ankle_pitch': {
        'current_threshold': 2,
        'current_recovery_threshold': 0.025,
        'current_to_torque_coeff': 0.8,
        'expected_torque': 0.0004,
        'damping_coeff': 0.0001,
        'stiffness_coeff': 0.07,
    },#
}

# 默认配置（如果某个关节没有在上面定义）
DEFAULT_CONFIG = {
    'current_threshold': 0.02,
    'current_recovery_threshold': 0.01,
    'current_to_torque_coeff': 1.0,
    'expected_torque': 0.0,
    'damping_coeff': 10.0,
    'stiffness_coeff': 100.0,
}

# 通用参数（所有关节共用）
COMMON_PARAMS = {
    'collision_confirm_threshold': 2,      # 碰撞确认阈值
    'recovery_confirm_threshold': 2,       # 恢复确认阈值
    'baseline_update_interval': 100,       # 基准更新间隔
    'filter_window_size': 20,              # 滤波窗口大小
    'max_position_adjustment': 30.0,       # 最大位置调整量 (度)
}
# ================================================================

# ========== 串口 / 控制板 / 电机位映射占位表 ==========
# 后续只需要在这里手动改映射关系：
#   serial_id: 主控板串口号，范围 0-5
#   board_id:  该串口下的控制板板号，协议内通常为 0x00-0x07
#   motor_index: 该板上的电机序号，从 0 开始
#   motor_count: 该板实际控制的电机数量，仅支持 2 / 3 / 6 / 7
#
# 注意：下面只是占位映射，用于把“原始 28 关节命令”先落到可发送的
#       serial_id + board_id + motor_index 上。真实硬件拓扑确定后，改这里即可。
JOINT_MOTOR_ROUTE: Dict[str, Dict[str, int]] = {
    # serial 1
    'right_hip_roll':      {'serial_id': 1, 'board_id': 1, 'motor_index': 0, 'motor_count': 2},
    'right_hip_yaw':       {'serial_id': 1, 'board_id': 1, 'motor_index': 1, 'motor_count': 2},
    'right_knee_pitch':    {'serial_id': 1, 'board_id': 2, 'motor_index': 0, 'motor_count': 2},
    'right_ankle_yaw':     {'serial_id': 1, 'board_id': 2, 'motor_index': 1, 'motor_count': 2},
    'right_ankle_pitch':   {'serial_id': 1, 'board_id': 3, 'motor_index': 0, 'motor_count': 2},
    # serial 2
    # serial 3
    'left_hip_pitch':      {'serial_id': 3, 'board_id': 0, 'motor_index': 1, 'motor_count': 3},
    'right_hip_pitch':     {'serial_id': 3, 'board_id': 0, 'motor_index': 2, 'motor_count': 3},
    'left_hip_roll':       {'serial_id': 3, 'board_id': 1, 'motor_index': 0, 'motor_count': 2},
    'left_hip_yaw':        {'serial_id': 3, 'board_id': 1, 'motor_index': 1, 'motor_count': 2},
    'left_knee_pitch':     {'serial_id': 3, 'board_id': 2, 'motor_index': 0, 'motor_count': 2},
    'left_ankle_yaw':      {'serial_id': 3, 'board_id': 2, 'motor_index': 1, 'motor_count': 2},
    'left_ankle_pitch':    {'serial_id': 3, 'board_id': 3, 'motor_index': 0, 'motor_count': 2},
    # serial 4
    'right_shoulder_pitch':{'serial_id': 4, 'board_id': 0, 'motor_index': 0, 'motor_count': 6},
    'left_shoulder_pitch': {'serial_id': 4, 'board_id': 0, 'motor_index': 1, 'motor_count': 6},
    'neck_yaw':            {'serial_id': 4, 'board_id': 0, 'motor_index': 2, 'motor_count': 6},
    'neck_roll':           {'serial_id': 4, 'board_id': 0, 'motor_index': 3, 'motor_count': 6},
    'right_shoulder_roll': {'serial_id': 4, 'board_id': 1, 'motor_index': 0, 'motor_count': 2},
    'right_shoulder_yaw':  {'serial_id': 4, 'board_id': 1, 'motor_index': 1, 'motor_count': 2},
    'right_elbow_pitch':   {'serial_id': 4, 'board_id': 2, 'motor_index': 0, 'motor_count': 2},
    'right_wrist_yaw':     {'serial_id': 4, 'board_id': 3, 'motor_index': 0, 'motor_count': 2},
    # serial 5
    # serial 6
    'left_shoulder_roll':  {'serial_id': 6, 'board_id': 1, 'motor_index': 0, 'motor_count': 2},
    'left_shoulder_yaw':   {'serial_id': 6, 'board_id': 1, 'motor_index': 1, 'motor_count': 2},
    'left_elbow_pitch':    {'serial_id': 6, 'board_id': 2, 'motor_index': 0, 'motor_count': 2},
    'left_wrist_yaw':      {'serial_id': 6, 'board_id': 3, 'motor_index': 0, 'motor_count': 2},
    # serial 7
    'neck_pitch':          {'serial_id': 0, 'board_id': 7, 'motor_index': 1, 'motor_count': 2},
    'waist_pitch':         {'serial_id': 4, 'board_id': 0, 'motor_index': 4, 'motor_count': 6},
    'waist_roll':          {'serial_id': 4, 'board_id': 0, 'motor_index': 5, 'motor_count': 6},
    'waist_yaw':           {'serial_id': 1, 'board_id': 0, 'motor_index': 2, 'motor_count': 3},
}

LEG_JOINT_NAMES = {
    'left_hip_pitch', 'right_hip_pitch','right_hip_roll','right_hip_yaw','right_knee_pitch','right_ankle_yaw','right_ankle_pitch',
    'left_hip_roll','left_hip_yaw','left_knee_pitch','left_ankle_yaw','left_ankle_pitch',
    'right_shoulder_pitch','left_shoulder_pitch','right_shoulder_roll','right_shoulder_yaw','right_elbow_pitch','right_wrist_yaw',
    'left_shoulder_roll','left_shoulder_yaw','left_elbow_pitch','left_wrist_yaw',
}


@dataclass
class CurrentFilter:
    """电流滤波器 - 滑动窗口平均"""
    window_size: int = 20
    buffer: deque = field(default_factory=deque)
    sum_value: float = 0.0
    
    def add_sample(self, value: float):
        self.buffer.append(value)
        self.sum_value += value
        if len(self.buffer) > self.window_size:
            self.sum_value -= self.buffer.popleft()
    
    def get_average(self) -> float:
        if not self.buffer:
            return 0.0
        return self.sum_value / len(self.buffer)


@dataclass
class MotorAdmittanceState:
    """单个电机的导纳控制状态"""
    joint_id: str = ""
    current_current: float = 0.0
    current_position: float = 0.0
    last_current: float = 0.0
    last_position: float = 0.0
    target_position: float = 0.0
    planned_position: float = 0.0
    
    current_filter: CurrentFilter = field(default_factory=lambda: CurrentFilter(20))
    baseline_current: float = 0.0
    sample_counter: int = 0
    
    collision_detected: bool = False
    collision_counter: int = 0
    recovery_counter: int = 0
    
    last_velocity: float = 0.0
    last_update_time: float = field(default_factory=time.time)


class JointAdmittanceController(Node):
    """关节导纳控制器"""
    
    def __init__(self):
        super().__init__('joint_admittance_controller')
        self.motor_states: Dict[str, MotorAdmittanceState] = {}
    
    def _get_config(self, joint_id: str) -> dict:
        """获取关节配置（内部方法）"""
        if joint_id in JOINT_CONFIGS:
            return JOINT_CONFIGS[joint_id]
        else:
            print(f"⚠️  关节 {joint_id} 未在配置表中定义，使用默认配置")
            return DEFAULT_CONFIG
    
    def _init_joint(self, joint_id: str):
        """初始化关节（内部方法）"""
        if joint_id not in self.motor_states:
            state = MotorAdmittanceState(joint_id=joint_id)
            state.current_filter = CurrentFilter(COMMON_PARAMS['filter_window_size'])
            self.motor_states[joint_id] = state
    
    def execute_admittance_control(self, joint_id: str, current: float, position: float) -> Tuple[bool, float]:
        """
        执行导纳控制（主函数 - 唯一需要调用的接口）
        
        Args:
            joint_id: 关节标识符
            current: 当前电流 (A)
            position: 当前关节位置 (度)
        
        Returns:
            Tuple[bool, float]: (是否检测到碰撞, 更新后的目标位置)
        """
        # 自动初始化
        self._init_joint(joint_id)
        
        state = self.motor_states[joint_id]
        config = self._get_config(joint_id)
        
        # 更新传感器数据
        state.current_current = current
        state.current_position = position
        
        # 电流滤波
        state.current_filter.add_sample(state.current_current)
        state.sample_counter += 1
        if state.sample_counter >= COMMON_PARAMS['baseline_update_interval']:
            state.baseline_current = state.current_filter.get_average()
            state.sample_counter = 0
        
        # 碰撞检测
        collision_detected = self._detect_collision(state, config)
        
        # 计算力矩差
        measured_torque = state.current_current * config['current_to_torque_coeff']
        torque_difference = measured_torque - config['expected_torque']
        
        # 计算速度差
        current_time = time.time()
        dt = current_time - state.last_update_time
        velocity = 0.0
        if dt > 0:
            velocity = (state.current_position - state.last_position) / dt
        velocity_difference = velocity - state.last_velocity
        
        # 导纳方程求解位置调整量
        position_diff = (torque_difference - config['damping_coeff'] * velocity_difference) / config['stiffness_coeff']
        
        # 限幅
        max_adj = COMMON_PARAMS['max_position_adjustment']
        position_diff = max(min(position_diff, max_adj), -max_adj)
        self.get_logger().info(f'collision_detected:  {collision_detected}  position_diff:  {position_diff}')
        
        # 更新目标位置
        state.target_position = state.planned_position + position_diff
        self.get_logger().info(f'target_position:  {state.target_position}')
        
        # 更新历史数据
        state.last_current = state.current_current
        state.last_position = state.current_position
        state.last_velocity = velocity
        state.last_update_time = current_time
        
        return collision_detected, state.target_position
    
    def _detect_collision(self, state: MotorAdmittanceState, config: dict) -> bool:
        """碰撞检测（内部方法）"""
        current_deviation = abs(state.current_current - state.baseline_current)
        
        if state.collision_detected:
            # 检测恢复
            if current_deviation < config['current_recovery_threshold']:
                state.recovery_counter += 1
                if state.recovery_counter >= COMMON_PARAMS['recovery_confirm_threshold']:
                    state.collision_detected = False
                    state.collision_counter = 0
                    state.recovery_counter = 0
                    print(f"✓ 关节 {state.joint_id}: 碰撞恢复")
                    return False
            else:
                state.recovery_counter = 0
        else:
            # 检测碰撞
            if current_deviation > config['current_threshold']:
                state.collision_counter += 1
                if state.collision_counter >= COMMON_PARAMS['collision_confirm_threshold']:
                    state.collision_detected = True
                    state.recovery_counter = 0
                    self.get_logger().info(f"⚠️  关节 {state.joint_id}: 检测到碰撞 (电流偏差: {current_deviation:.4f}A)")
                    return True
            else:
                state.collision_counter = 0
        
        return state.collision_detected
    
    def set_planned_position(self, joint_id: str, planned_pos: float):
        """设置规划位置"""
        self._init_joint(joint_id)
        self.motor_states[joint_id].planned_position = planned_pos
    
    def get_position_adjustment(self, joint_id: str) -> float:
        """获取位置调整量"""
        if joint_id not in self.motor_states:
            return 0.0
        state = self.motor_states[joint_id]
        return state.target_position - state.planned_position
    
    def get_diagnostic_info(self, joint_id: str) -> str:
        """获取诊断信息"""
        if joint_id not in self.motor_states:
            return f"关节 {joint_id} 未初始化"
        
        state = self.motor_states[joint_id]
        config = self._get_config(joint_id)
        
        return f"""
========== 关节 {joint_id} 诊断信息 ==========
当前电流: {state.current_current:.4f} A
当前位置: {state.current_position:.2f}°
基准电流: {state.baseline_current:.4f} A
电流偏差: {abs(state.current_current - state.baseline_current):.4f} A

目标位置: {state.target_position:.2f}°
规划位置: {state.planned_position:.2f}°
位置调整: {state.target_position - state.planned_position:.2f}°

碰撞状态: {'是' if state.collision_detected else '否'}

控制参数:
  - 阻尼系数: {config['damping_coeff']}
  - 刚度系数: {config['stiffness_coeff']}
  - 电流阈值: {config['current_threshold']}A
===============================================
"""
    
# ========== ROS2导纳控制节点 ==========
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rdk_x5_multi_serial.msg import SerialData
import json
import numpy as np
import sys
import os


class AdmittanceControlNode(Node):
    """导纳控制节点 - 接收关节命令和电机反馈，执行导纳控制，发布电机命令"""

    def __init__(self):
        super().__init__('admittance_control_node')

        # ========== 配置参数 ==========
        self.declare_parameter('enable_admittance', False)
        self.declare_parameter('joint_command_topic', '/joint_command')
        self.declare_parameter('motor_command_topic', '/serial_cmd')
        self.declare_parameter('motor_feedback_topic', '/serial_data')

        # 单关节调试模式：
        #   设为关节名（如 'right_elbow_pitch'）则只处理该关节命令和反馈，其他关节忽略。
        #   设为空字符串 '' 则正常处理全部关节。
        self.declare_parameter('debug_joint', '')

        # 电机角度调试：
        #   enable_motor_angle_debug=True 时，会按 motor_angle_debug_interval 节流打印并发布
        #   每个 port/board_id/motor_index 实际下发的角度。
        self.declare_parameter('enable_motor_angle_debug', True)
        self.declare_parameter('motor_angle_debug_interval', 30)
        self.declare_parameter('motor_angle_debug_topic', '/motor_angle_debug')

        self.ENABLE_ADMITTANCE = self.get_parameter('enable_admittance').value
        self.JOINT_COMMAND_TOPIC = self.get_parameter('joint_command_topic').value
        self.MOTOR_COMMAND_TOPIC = self.get_parameter('motor_command_topic').value
        self.MOTOR_FEEDBACK_TOPIC = self.get_parameter('motor_feedback_topic').value
        self.ENABLE_MOTOR_ANGLE_DEBUG = bool(self.get_parameter('enable_motor_angle_debug').value)
        self.MOTOR_ANGLE_DEBUG_INTERVAL = max(1, int(self.get_parameter('motor_angle_debug_interval').value))
        self.MOTOR_ANGLE_DEBUG_TOPIC = self.get_parameter('motor_angle_debug_topic').value

        # 单关节调试模式
        self.DEBUG_JOINT = self.get_parameter('debug_joint').value

        # ========== 导纳控制器 ==========
        if self.ENABLE_ADMITTANCE:
            self.admittance_controller = JointAdmittanceController()
            self.get_logger().info('✅ 导纳控制器已启用')
        else:
            self.admittance_controller = None
            self.get_logger().info('⚠️  导纳控制器已禁用')

        # ========== 关节名称映射 ==========
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

        # ========== ROS2订阅和发布 ==========
        self.joint_command_sub = self.create_subscription(
            String,
            self.JOINT_COMMAND_TOPIC,
            self.joint_command_callback,
            10
        )

        self.motor_feedback_sub = self.create_subscription(
            SerialData,
            self.MOTOR_FEEDBACK_TOPIC,
            self.motor_feedback_callback,
            10
        )

        self.motor_command_pub = self.create_publisher(
            String,
            self.MOTOR_COMMAND_TOPIC,
            10
        )

        self.motor_angle_debug_pub = self.create_publisher(
            String,
            self.MOTOR_ANGLE_DEBUG_TOPIC,
            10
        )

        # ========== 统计信息 ==========
        self.stats = {
            'total_joint_commands_received': 0,
            'total_motor_feedback_received': 0,
            'total_motor_commands_published': 0,
            'admittance_adjustments': 0,
            'motor_angle_debug_published': 0,
            'start_time': time.time()
        }

        # ========== 缓存最近的反馈数据 ==========
        self.last_feedback = {}
        # 当前控制流程只有一个动作，action_id 固定为 1。
        # frame_index 对每一帧关节命令递增，同一帧内各控制板共用该编号。
        self._next_frame_index = 0

        # 最近一次“关节角 -> 电机角”的详细映射，供调试输出使用。
        self._last_motor_angle_debug = {}
        self._last_motor_angle_debug_seq = 0

        # 定期打印统计信息
        self.create_timer(5.0, self.print_statistics)

        self.get_logger().info('='*70)
        self.get_logger().info('🎯 导纳控制节点已启动')
        self.get_logger().info(f'  导纳控制: {"✅ 启用" if self.ENABLE_ADMITTANCE else "❌ 禁用"}')
        self.get_logger().info(f'  订阅关节命令话题: {self.JOINT_COMMAND_TOPIC}')
        self.get_logger().info(f'  订阅电机反馈话题: {self.MOTOR_FEEDBACK_TOPIC}')
        self.get_logger().info(f'  发布电机命令话题: {self.MOTOR_COMMAND_TOPIC}')
        if self.DEBUG_JOINT:
            self.get_logger().info(f'  🔧 单关节调试模式: {self.DEBUG_JOINT}')
        self.get_logger().info(f'  电机角度调试: {"✅ 启用" if self.ENABLE_MOTOR_ANGLE_DEBUG else "❌ 禁用"}')
        if self.ENABLE_MOTOR_ANGLE_DEBUG:
            self.get_logger().info(
                f'  电机角度调试话题: {self.MOTOR_ANGLE_DEBUG_TOPIC}, '
                f'日志间隔: 每 {self.MOTOR_ANGLE_DEBUG_INTERVAL} 帧'
            )
        self.get_logger().info('='*70)

    def motor_feedback_callback(self, msg: SerialData):
        """接收 RDK 的 /serial_data 反馈，更新缓存（供下次导纳控制使用）。

        RDK 消息类型：rdk_x5_multi_serial/msg/SerialData
            uint8 port
            uint8 board_id
            float32[] angles
            float32[] currents
            int32 action_id
            int32 frame_index

        注意：TW 内部历史命名仍使用 serial_id，含义等同于 RDK 的 port。
        """
        try:
            self.stats['total_motor_feedback_received'] += 1

            serial_id = int(msg.port)
            board_id = int(msg.board_id)
            angles = [float(x) for x in msg.angles]
            currents = [float(x) * CURRENT_MA_TO_A for x in msg.currents]
            action_id = int(msg.action_id)
            frame_index = int(msg.frame_index)

            if len(angles) != len(currents):
                n = min(len(angles), len(currents))
                self.get_logger().warning(
                    f'/serial_data 角度/电流长度不一致: '
                    f'port={serial_id}, board_id={board_id}, '
                    f'angles={len(angles)}, currents={len(currents)}，仅使用前 {n} 个'
                )
                angles = angles[:n]
                currents = currents[:n]

            if not angles or not currents:
                # RDK 对非电机反馈或异常帧也可能发布空数组，这里直接忽略。
                return

            self.update_feedback_cache(
                serial_id,
                board_id,
                angles,
                currents,
                action_id,
                frame_index,
            )

        except Exception as e:
            self.get_logger().error(f'处理 /serial_data 反馈失败: {e}')

    def joint_command_callback(self, msg):
        """接收关节命令，执行导纳控制，发布到motor_command"""
        try:
            # 解析命令
            command_data = json.loads(msg.data)
            joint_angles = np.array(command_data['joint_angles'])
            only_legs = command_data.get('only_legs', False)
            # 对齐 RDK 新版链路字段：动作编号固定为 1。
            # 上游未提供 frame_index 时，由本节点自动生成递增编号。
            upstream_frame_index = command_data.get('frame_index')
            if upstream_frame_index is None or int(upstream_frame_index) < 0:
                frame_index = self._next_frame_index
            else:
                frame_index = int(upstream_frame_index)
            self._next_frame_index = max(self._next_frame_index, frame_index + 1)

            self.stats['total_joint_commands_received'] += 1

            # 执行导纳控制（如果启用）
            if self.ENABLE_ADMITTANCE and self.admittance_controller:
                adjusted_angles = self.apply_admittance_control(joint_angles)
            else:
                adjusted_angles = joint_angles

            # 转换为 motor_command 格式并发布
            motor_command = self.build_motor_command(
                adjusted_angles,
                only_legs,
                frame_index=frame_index,
            )
            self.publish_motor_angle_debug(motor_command, adjusted_angles, only_legs)
            self.publish_motor_command(motor_command)

        except Exception as e:
            self.get_logger().error(f'处理关节命令失败: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def apply_admittance_control(self, joint_angles):
        """
        应用导纳控制

        Args:
            joint_angles: 28个关节角度

        Returns:
            调整后的关节角度
        """
        adjusted_angles = joint_angles.copy()

        # 遍历所有关节，应用导纳控制
        for i, joint_name in enumerate(self.joint_names):
            if joint_name not in self.last_feedback:
                # 没有反馈数据，直接使用原始角度
                continue

            # 获取该关节的电流和实际位置
            feedback = self.last_feedback[joint_name]
            current = feedback.get('current', 0.0)
            actual_position = feedback.get('actual_angle', joint_angles[i])

            # 设置规划位置（命令角度）
            self.admittance_controller.set_planned_position(joint_name, joint_angles[i])

            # 执行导纳控制
            collision_detected, target_position = self.admittance_controller.execute_admittance_control(
                joint_name,
                current,
                actual_position
            )
            if(collision_detected):
                # 更新调整后的角度
                adjustment = target_position - joint_angles[i]
                if abs(adjustment) > 0.01:  # 只有调整量大于阈值时才应用
                    adjusted_angles[i] = target_position
                    self.stats['admittance_adjustments'] += 1

                    if abs(adjustment) > 1.0:  # 较大调整时打印
                        self.get_logger().debug(
                            f'关节 {joint_name}: 导纳调整 {adjustment:.2f}° '
                            f'(碰撞: {"是" if collision_detected else "否"})'
                        )

        return adjusted_angles

    def build_motor_command(
        self,
        joint_angles,
        only_legs=False,
        frame_index=-1,
    ):
        """
        将关节角度转换为 /motor_command 简洁格式。

        发布格式为 std_msgs/String，其中 msg.data 是 JSON 字符串：

        {
            "port": 1,
            "board_id": 1,
            "angles": [160.0, 50.0],
            "action_id": 1,
            "frame_index": 42
        }

        注意：
        - 这里不再使用 commands 批量字段
        - 不再下发 stop_on_error
        - 不再下发 verify_crc
        - serial_id 改名为 port
        """
        board_mapping = self.joint_angles_to_board_mapping(joint_angles, only_legs)

        motor_commands = []

        for (serial_id, board_id), angles in sorted(board_mapping.items()):
            motor_commands.append({
                "port": int(serial_id),
                "board_id": int(board_id),
                "angles": [float(a) for a in angles],
                "action_id": RDK_ACTION_ID,
                "frame_index": frame_index,
            })

        return motor_commands

    def update_feedback_cache(
        self,
        serial_id,
        board_id,
        angles,
        currents,
        action_id,
        frame_index,
    ):
        """更新反馈缓存，将 serial_id/port + board_id + motor_index 映射回关节。

        Args:
            serial_id: RDK SerialData.port，TW 内部沿用 serial_id 命名。
            board_id: RDK SerialData.board_id。
            angles: RDK SerialData.angles，按电机位顺序排列。
            currents: RDK SerialData.currents，按电机位顺序排列。
            action_id: RDK SerialData.action_id。
            frame_index: RDK SerialData.frame_index。
        """
        for joint_name, route in JOINT_MOTOR_ROUTE.items():
            if int(route['serial_id']) != int(serial_id):
                continue
            if int(route['board_id']) != int(board_id):
                continue

            motor_index = int(route['motor_index'])
            if len(angles) <= motor_index or len(currents) <= motor_index:
                continue

            self.last_feedback[joint_name] = {
                'serial_id': int(serial_id),
                'board_id': int(board_id),
                'motor_index': motor_index,
                'actual_angle': angles[motor_index],
                'current': currents[motor_index],
                'action_id': int(action_id),
                'frame_index': int(frame_index),
            }

    def transform_joint_angle_to_motor_angle(self, joint_name: str, joint_angle: float) -> float:
        """关节角到电机协议角的转换。"""
        whole_offset = 0
        right_shoulder_roll_offset = 360
        right_knee_pitch_offset = 360
        angle = float(joint_angle)

        transform_map = {
            # 左腿
            'left_hip_pitch':   lambda x: x + whole_offset,
            'left_hip_roll':    lambda x: x + whole_offset,
            'left_hip_yaw':     lambda x: x + whole_offset,
            'left_knee_pitch':  lambda x: x + whole_offset,
            'left_ankle_yaw':   lambda x: x + whole_offset,
            'left_ankle_pitch': lambda x: x + whole_offset,

            # 右腿
            'right_hip_pitch':      lambda x: x + whole_offset,
            'right_hip_roll':       lambda x: x + whole_offset,
            'right_hip_yaw':        lambda x: x + whole_offset,
            'right_knee_pitch':     lambda x: x,
            'right_ankle_yaw':      lambda x: x,
            'right_ankle_pitch':    lambda x: x + whole_offset,

            # 右臂
            'right_shoulder_roll':  lambda x: x + whole_offset,
            'right_shoulder_yaw':   lambda x: x + whole_offset,
            'right_elbow_pitch':    lambda x: x + whole_offset,
            'right_wrist_yaw':      lambda x: x + whole_offset,
            'right_shoulder_pitch': lambda x: x + whole_offset,

            # 左臂
            'left_shoulder_roll':   lambda x: x + whole_offset,
            'left_shoulder_yaw':    lambda x: x + whole_offset,
            'left_elbow_pitch':     lambda x: x + whole_offset,
            'left_wrist_yaw':       lambda x: x + whole_offset,
            'left_shoulder_pitch':  lambda x: x + whole_offset,

            # 腰和脖子，先按统一零位处理，后续根据机械方向修正
            'waist_yaw':    lambda x: x + whole_offset,
            'waist_pitch':  lambda x: x + whole_offset,
            'waist_roll':   lambda x: x + whole_offset,
            'neck_yaw':     lambda x: x + whole_offset,
            'neck_roll':    lambda x: x + whole_offset,
            'neck_pitch':   lambda x: x + whole_offset,
        }


        motor_angle = transform_map.get(joint_name, lambda x: x + whole_offset)(angle)

        # 保护：限制到协议合法范围 [0, 360)
        if motor_angle < 0:
            motor_angle += 360.0
        elif motor_angle >= 360:
            motor_angle = 359.999

        return motor_angle

    def joint_angles_to_board_mapping(self, joint_angles, only_legs=False):
        """将 28 个关节角度映射到 serial_id + board_id + 电机数组。

        返回值格式：
            {(serial_id, board_id): [motor_angle_0, motor_angle_1, ...]}

        同时填充 self._last_motor_angle_debug，用于输出每个电机位的调试信息。
        """
        board_mapping: Dict[Tuple[int, int], List[float]] = {}
        debug_mapping: Dict[Tuple[int, int], Dict[str, Any]] = {}

        for joint_index, joint_name in enumerate(self.joint_names):
            if joint_index >= len(joint_angles):
                continue
            if only_legs and joint_name not in LEG_JOINT_NAMES:
                continue
            # 单关节调试模式：只处理 debug_joint 指定的关节
            if self.DEBUG_JOINT and joint_name != self.DEBUG_JOINT:
                continue

            route = JOINT_MOTOR_ROUTE.get(joint_name)
            if route is None:
                self.get_logger().warning(f'关节 {joint_name} 未配置串口/板号/电机位映射，已跳过')
                continue

            serial_id = int(route['serial_id'])
            board_id = int(route['board_id'])
            motor_index = int(route['motor_index'])
            motor_count = int(route['motor_count'])

            if serial_id < 0 or serial_id > 8:
                raise ValueError(f'{joint_name} 的 serial_id 必须在 0-7 范围内，当前值: {serial_id}')
            if motor_count not in (2, 3, 6, 7):
                raise ValueError(f'{joint_name} 的 motor_count 仅支持 2/3/6/7，当前值: {motor_count}')
            if motor_index < 0 or motor_index >= motor_count:
                raise ValueError(f'{joint_name} 的 motor_index={motor_index} 超出 motor_count={motor_count}')

            key = (serial_id, board_id)
            if key not in board_mapping:
                board_mapping[key] = [0.0] * motor_count
                debug_mapping[key] = {
                    'port': serial_id,
                    'board_id': board_id,
                    'motor_count': motor_count,
                    'motors': [None] * motor_count,
                }
            elif len(board_mapping[key]) != motor_count:
                raise ValueError(
                    f'serial_id={serial_id}, board_id={board_id} 的 motor_count 配置不一致: '
                    f'{len(board_mapping[key])} vs {motor_count}'
                )

            raw_joint_angle = float(joint_angles[joint_index])
            motor_angle = self.transform_joint_angle_to_motor_angle(joint_name, raw_joint_angle)

            board_mapping[key][motor_index] = motor_angle
            debug_mapping[key]['motors'][motor_index] = {
                'motor_index': motor_index,
                'joint_index': joint_index,
                'joint_name': joint_name,
                'joint_angle_deg': raw_joint_angle,
                'motor_angle_deg': float(motor_angle),
            }

        # 对未填充的电机位也记录下来，避免调试时误以为没有下发。
        # 注意：当前逻辑中未填充位实际会随 angles 数组以 0.0 下发。
        for key, info in debug_mapping.items():
            for motor_index, item in enumerate(info['motors']):
                if item is None:
                    info['motors'][motor_index] = {
                        'motor_index': motor_index,
                        'joint_index': None,
                        'joint_name': None,
                        'joint_angle_deg': None,
                        'motor_angle_deg': float(board_mapping[key][motor_index]),
                        'note': 'unmapped_slot_filled_with_0.0',
                    }

        self._last_motor_angle_debug = debug_mapping
        return board_mapping

    def _build_motor_angle_debug_payload(self, motor_command, joint_angles, only_legs):
        """构造电机角度调试 JSON。"""
        boards = []
        for key in sorted(self._last_motor_angle_debug.keys()):
            info = self._last_motor_angle_debug[key]
            boards.append({
                'port': int(info['port']),
                'board_id': int(info['board_id']),
                'motor_count': int(info['motor_count']),
                'angles': [
                    float(m['motor_angle_deg']) if m.get('motor_angle_deg') is not None else None
                    for m in info['motors']
                ],
                'motors': info['motors'],
            })

        return {
            'timestamp': time.time(),
            'seq': int(self.stats['total_joint_commands_received']),
            'only_legs': bool(only_legs),
            'debug_source': 'admittance_calculate.py',
            'boards': boards,
        }

    def _log_motor_angle_debug(self, payload):
        """按 port/board/motor 打印本帧实际下发角度。"""
        lines = [
            '',
            f'🧪 电机角度调试 seq={payload["seq"]}, only_legs={payload["only_legs"]}',
        ]

        for board in payload['boards']:
            lines.append(
                f'  port={board["port"]}, board_id={board["board_id"]}, '
                f'motor_count={board["motor_count"]}, angles={self._format_float_list(board["angles"])}'
            )
            for motor in board['motors']:
                joint_name = motor.get('joint_name')
                motor_index = motor.get('motor_index')
                motor_angle = motor.get('motor_angle_deg')
                joint_angle = motor.get('joint_angle_deg')

                if joint_name is None:
                    lines.append(
                        f'    motor[{motor_index}]: motor_angle={motor_angle:.3f}° '
                        f'({motor.get("note", "unmapped")})'
                    )
                else:
                    lines.append(
                        f'    motor[{motor_index}]: {joint_name} '
                        f'joint_angle={joint_angle:.3f}° -> motor_angle={motor_angle:.3f}°'
                    )

        self.get_logger().info('\n'.join(lines))

    def _format_float_list(self, values):
        return '[' + ', '.join(
            'None' if v is None else f'{float(v):.3f}'
            for v in values
        ) + ']'

    def publish_motor_angle_debug(self, motor_command, joint_angles, only_legs):
        """发布并节流打印每个电机位实际下发角度。

        调试话题：
            /motor_angle_debug

        消息类型：
            std_msgs/String，data 为 JSON。
        """
        if not self.ENABLE_MOTOR_ANGLE_DEBUG:
            return

        try:
            self._last_motor_angle_debug_seq += 1

            # 避免 90Hz 全量刷屏，默认每 30 帧输出一次。
            should_emit = (self._last_motor_angle_debug_seq % self.MOTOR_ANGLE_DEBUG_INTERVAL == 0)
            if not should_emit:
                return

            payload = self._build_motor_angle_debug_payload(motor_command, joint_angles, only_legs)

            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.motor_angle_debug_pub.publish(msg)
            self.stats['motor_angle_debug_published'] += 1

            self._log_motor_angle_debug(payload)

        except Exception as e:
            self.get_logger().error(f'发布电机角度调试信息失败: {e}')

    def publish_motor_command(self, motor_command):
        """
        发布电机命令到 /motor_command 话题。

        现在不再发布一个 commands 批量 JSON，
        而是每一块板单独发布一条 std_msgs/String。

        每条消息格式：
        {
            "port": 1,
            "board_id": 1,
            "angles": [160.0, 50.0]
        }
        """
        try:
            # build_motor_command 返回的是 list，每个元素对应一块控制板
            if isinstance(motor_command, list):
                for single_cmd in motor_command:
                    msg = String()
                    msg.data = json.dumps(single_cmd, ensure_ascii=False)
                    self.motor_command_pub.publish(msg)
                    self.stats['total_motor_commands_published'] += 1
                return

            # 兼容单条命令
            msg = String()
            msg.data = json.dumps(motor_command, ensure_ascii=False)
            self.motor_command_pub.publish(msg)
            self.stats['total_motor_commands_published'] += 1

        except Exception as e:
            self.get_logger().error(f'发布电机命令失败: {e}')

    def print_statistics(self):
        """打印统计信息"""
        elapsed = time.time() - self.stats['start_time']

        self.get_logger().info(
            f'\\n'
            f'📊 导纳控制节点统计:\\n'
            f'  ⏱️  运行时间: {elapsed:.1f}秒\\n'
            f'  📥 接收joint_command: {self.stats["total_joint_commands_received"]}帧\\n'
            f'  📥 接收motor_feedback: {self.stats["total_motor_feedback_received"]}帧\\n'
            f'  📤 发布motor_command: {self.stats["total_motor_commands_published"]}帧\\n'
            f'  🧪 电机角度调试发布: {self.stats["motor_angle_debug_published"]}次\\n'
            f'  🔧 导纳调整: {self.stats["admittance_adjustments"]}次\\n'
        )

    def shutdown(self):
        """关闭节点"""
        self.get_logger().info('关闭导纳控制节点...')


def main_admittance_node(args=None):
    """导纳控制节点主函数"""
    rclpy.init(args=args)
    node = AdmittanceControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main_admittance_node()
