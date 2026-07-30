#!/usr/bin/env python3
"""
电流监控与绘图节点 (Current Monitor Node)

功能：
  订阅 /serial_data 话题，记录每个关节的时间-电流数据；
  在节点关闭时（Ctrl+C）自动：
    1. 导出 CSV 数据
    2. 绘制时间-电流图像并导出为 PNG 文件

使用方式：
  监控全部关节：
    ros2 run impendence_control current_monitor --ros-args \
      -p output_dir:="./current_plots"

  监控单个关节：
    ros2 run impendence_control current_monitor --ros-args \
      -p output_dir:="./current_plots" \
      -p target_joints:="right_elbow_pitch"

  监控多个关节：
    ros2 run impendence_control current_monitor --ros-args \
      -p output_dir:="./current_plots" \
      -p target_joints:="right_elbow_pitch,left_elbow_pitch"

输出：
  - 总览 PNG 图像: <output_dir>/current_plot_<timestamp>.png
  - 单关节 PNG 图像: <output_dir>/current_<joint_name>_<timestamp>.png
  - CSV 数据: <output_dir>/current_data_<timestamp>.csv
"""

import rclpy
import time
import os
import csv
from collections import defaultdict
from rclpy.node import Node
from rdk_x5_multi_serial.msg import SerialData

# 尝试导入 matplotlib；若不可用则给出清晰提示
try:
    import matplotlib
    matplotlib.use('Agg')  # 非交互式后端，支持无 GUI 环境导出图像
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# 与 admittance_calculate.py 保持一致的映射表
JOINT_MOTOR_ROUTE = {
    'right_hip_roll':       {'serial_id': 1, 'board_id': 1, 'motor_index': 0, 'motor_count': 2},
    'right_hip_yaw':        {'serial_id': 1, 'board_id': 1, 'motor_index': 1, 'motor_count': 2},
    'right_knee_pitch':     {'serial_id': 1, 'board_id': 2, 'motor_index': 0, 'motor_count': 2},
    'right_ankle_yaw':      {'serial_id': 1, 'board_id': 2, 'motor_index': 1, 'motor_count': 2},
    'right_ankle_pitch':    {'serial_id': 1, 'board_id': 3, 'motor_index': 0, 'motor_count': 2},

    'left_hip_pitch':       {'serial_id': 1, 'board_id': 0, 'motor_index': 0, 'motor_count': 3},
    'right_hip_pitch':      {'serial_id': 1, 'board_id': 0, 'motor_index': 1, 'motor_count': 3},

    'left_hip_roll':        {'serial_id': 3, 'board_id': 1, 'motor_index': 0, 'motor_count': 2},
    'left_hip_yaw':         {'serial_id': 3, 'board_id': 1, 'motor_index': 1, 'motor_count': 2},
    'left_knee_pitch':      {'serial_id': 3, 'board_id': 2, 'motor_index': 0, 'motor_count': 2},
    'left_ankle_yaw':       {'serial_id': 3, 'board_id': 2, 'motor_index': 1, 'motor_count': 2},
    'left_ankle_pitch':     {'serial_id': 3, 'board_id': 3, 'motor_index': 0, 'motor_count': 2},

    'right_shoulder_pitch': {'serial_id': 4, 'board_id': 0, 'motor_index': 0, 'motor_count': 6},
    'left_shoulder_pitch':  {'serial_id': 4, 'board_id': 0, 'motor_index': 1, 'motor_count': 6},
    'right_shoulder_roll':  {'serial_id': 4, 'board_id': 1, 'motor_index': 0, 'motor_count': 2},
    'right_shoulder_yaw':   {'serial_id': 4, 'board_id': 1, 'motor_index': 1, 'motor_count': 2},
    'right_elbow_pitch':    {'serial_id': 4, 'board_id': 2, 'motor_index': 0, 'motor_count': 2},
    'right_wrist_yaw':      {'serial_id': 4, 'board_id': 3, 'motor_index': 0, 'motor_count': 2},

    'left_shoulder_roll':   {'serial_id': 6, 'board_id': 1, 'motor_index': 0, 'motor_count': 2},
    'left_shoulder_yaw':    {'serial_id': 6, 'board_id': 1, 'motor_index': 1, 'motor_count': 2},
    'left_elbow_pitch':     {'serial_id': 6, 'board_id': 2, 'motor_index': 0, 'motor_count': 2},
    'left_wrist_yaw':       {'serial_id': 6, 'board_id': 3, 'motor_index': 0, 'motor_count': 2},

    'neck_roll':            {'serial_id': 0, 'board_id': 7, 'motor_index': 0, 'motor_count': 2},
    'neck_yaw':             {'serial_id': 2, 'board_id': 6, 'motor_index': 1, 'motor_count': 2},
    'neck_pitch':           {'serial_id': 0, 'board_id': 7, 'motor_index': 1, 'motor_count': 2},

    'waist_pitch':          {'serial_id': 4, 'board_id': 0, 'motor_index': 4, 'motor_count': 6},
    'waist_roll':           {'serial_id': 4, 'board_id': 0, 'motor_index': 5, 'motor_count': 6},
    'waist_yaw':            {'serial_id': 1, 'board_id': 0, 'motor_index': 2, 'motor_count': 3},
}


class CurrentMonitor(Node):
    """电流监控节点 - 记录并在停止时导出 CSV 与图像"""

    def __init__(self):
        super().__init__('current_monitor')

        if not HAS_MATPLOTLIB:
            self.get_logger().error(
                '❌ matplotlib 未安装！请执行: pip install matplotlib'
            )
            raise ImportError('matplotlib is required for CurrentMonitor')

        # ========== 参数 ==========
        self.declare_parameter('motor_feedback_topic', '/serial_data')
        self.declare_parameter('output_dir', './current_plots')

        # 注意：
        # ROS2 Humble 中 declare_parameter('target_joints', [])
        # 容易被识别成 BYTE_ARRAY，导致传字符串时报类型错误。
        # 这里改成字符串参数，多个关节用逗号分隔。
        self.declare_parameter('target_joints', '')

        self.MOTOR_FEEDBACK_TOPIC = self.get_parameter('motor_feedback_topic').value
        self.output_dir = self.get_parameter('output_dir').value

        target_joints_param = self.get_parameter('target_joints').value

        if isinstance(target_joints_param, str) and target_joints_param.strip():
            self.target_joints = [
                name.strip()
                for name in target_joints_param.split(',')
                if name.strip()
            ]
        else:
            self.target_joints = []

        # 检查输入的关节名是否存在
        invalid_joints = [
            name for name in self.target_joints
            if name not in JOINT_MOTOR_ROUTE
        ]

        if invalid_joints:
            self.get_logger().warning(
                f'⚠️  以下 target_joints 不在 JOINT_MOTOR_ROUTE 中，将不会采集到数据: {invalid_joints}'
            )

        # ========== 数据存储 ==========
        # data[joint_name] = [(timestamp, current), ...]
        self.data = defaultdict(list)
        self.start_time = time.time()
        self.frame_count = 0
        self.last_action_id = -1
        self.last_frame_index = -1

        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)

        # ========== 订阅 ==========
        self.motor_feedback_sub = self.create_subscription(
            SerialData,
            self.MOTOR_FEEDBACK_TOPIC,
            self.motor_feedback_callback,
            10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 电流监控节点已启动')
        self.get_logger().info(f'  订阅话题: {self.MOTOR_FEEDBACK_TOPIC}')
        self.get_logger().info(f'  输出目录: {os.path.abspath(self.output_dir)}')

        if self.target_joints:
            self.get_logger().info(f'  监控关节: {self.target_joints}')
        else:
            self.get_logger().info(f'  监控关节: 全部 ({len(JOINT_MOTOR_ROUTE)} 个)')

        self.get_logger().info('  CSV 保存方式: 停止时保存一次')
        self.get_logger().info('  图像导出方式: 停止时导出')
        self.get_logger().info('=' * 60)

    # ==================== 数据采集 ====================

    def motor_feedback_callback(self, msg: SerialData):
        """接收新版 /serial_data 反馈，记录电流数据。"""
        try:
            serial_id = int(msg.port)
            board_id = int(msg.board_id)
            currents = [float(x) for x in msg.currents]
            action_id = int(msg.action_id)
            frame_index = int(msg.frame_index)

            if not currents:
                return

            now = time.time()

            for joint_name, route in JOINT_MOTOR_ROUTE.items():
                # 如果指定了目标关节，则过滤
                if self.target_joints and joint_name not in self.target_joints:
                    continue

                if int(route['serial_id']) != serial_id:
                    continue

                if int(route['board_id']) != board_id:
                    continue

                motor_index = int(route['motor_index'])
                if len(currents) <= motor_index:
                    continue

                current_val = currents[motor_index]
                self.data[joint_name].append((now, current_val))

            self.frame_count += 1
            self.last_action_id = action_id
            self.last_frame_index = frame_index

        except Exception as e:
            self.get_logger().error(f'处理反馈数据失败: {e}')

    # ==================== CSV 导出 ====================

    def save_csv(self, filepath: str = None):
        """将所有采集的数据导出为 CSV 文件。"""
        if not self.data:
            self.get_logger().warning('⚠️  没有采集到任何电流数据，跳过 CSV 保存')
            return None

        if filepath is None:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            filepath = os.path.join(
                self.output_dir,
                f'current_data_{timestamp}.csv'
            )

        all_records = []

        for joint_name, records in sorted(self.data.items()):
            for t, c in records:
                all_records.append({
                    'timestamp_abs': t,
                    'timestamp_rel': t - self.start_time,
                    'joint_name': joint_name,
                    'current': c,
                })

        all_records.sort(key=lambda x: x['timestamp_abs'])

        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    'timestamp_abs',
                    'timestamp_rel',
                    'joint_name',
                    'current',
                ]
            )
            writer.writeheader()
            writer.writerows(all_records)

        self.get_logger().info(
            f'📝 CSV 数据已保存: {filepath} ({len(all_records)} 行)'
        )

        return filepath

    # ==================== 绘图 ====================

    def plot_current_curves(self, save: bool = True, show: bool = False):
        """
        绘制所有已记录关节的时间-电流曲线。

        Args:
            save: 是否保存为 PNG
            show: 是否弹出显示，无 GUI 环境下请设为 False
        """
        if not self.data:
            self.get_logger().warning('⚠️  没有采集到任何电流数据，跳过绘图')
            return

        total_points = sum(len(v) for v in self.data.values())
        self.get_logger().info(
            f'📈 正在生成电流曲线图... ({total_points} 个数据点)'
        )

        # 筛选有足够数据点的关节
        active_joints = {
            k: v for k, v in self.data.items()
            if len(v) > 1
        }

        if not active_joints:
            self.get_logger().warning('⚠️  所有关节数据点不足，跳过绘图')
            return

        n_joints = len(active_joints)

        # 自适应子图布局：每行最多 3 列
        n_cols = min(3, n_joints)
        n_rows = (n_joints + n_cols - 1) // n_cols

        fig, axes = plt.subplots(
            n_rows,
            n_cols,
            figsize=(6 * n_cols, 4 * n_rows)
        )

        fig.suptitle(
            f'Joint Current vs Time\n'
            f'(Recorded: {time.strftime("%Y-%m-%d %H:%M:%S")})',
            fontsize=14,
            fontweight='bold'
        )

        # 确保 axes 可迭代
        if n_joints == 1:
            axes = [axes]
        else:
            axes = axes.flatten() if hasattr(axes, 'flatten') else axes

        for idx, (joint_name, records) in enumerate(sorted(active_joints.items())):
            ax = axes[idx]

            times = [t - self.start_time for t, _ in records]
            currents = [c for _, c in records]

            ax.plot(
                times,
                currents,
                linewidth=0.8,
                color='#2196F3',
                alpha=0.9
            )

            ax.set_title(joint_name, fontsize=10, fontweight='bold')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Current (mA)')
            ax.grid(True, alpha=0.3, linestyle='--')

            mean_c = sum(currents) / len(currents) if currents else 0
            max_c = max(currents) if currents else 0
            min_c = min(currents) if currents else 0

            ax.axhline(
                y=mean_c,
                color='red',
                linestyle='--',
                linewidth=0.6,
                alpha=0.6,
                label=f'Mean: {mean_c:.4f}mA'
            )

            ax.legend(fontsize=7, loc='upper right')

            stats_text = (
                f'Points: {len(records)}\n'
                f'Max: {max_c:.4f}mA\n'
                f'Min: {min_c:.4f}mA\n'
                f'Mean: {mean_c:.4f}mA'
            )

            ax.text(
                0.98,
                0.97,
                stats_text,
                transform=ax.transAxes,
                fontsize=7,
                verticalalignment='top',
                horizontalalignment='right',
                bbox=dict(
                    boxstyle='round',
                    facecolor='wheat',
                    alpha=0.5
                )
            )

        # 隐藏多余的子图
        for idx in range(n_joints, len(axes)):
            axes[idx].set_visible(False)

        fig.tight_layout()

        if save:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            filename = f'current_plot_{timestamp}.png'
            filepath = os.path.join(self.output_dir, filename)

            fig.savefig(filepath, dpi=150, bbox_inches='tight')

            self.get_logger().info(f'💾 电流曲线总图已保存: {filepath}')

            # 同时为每个有数据的关节单独保存一张图
            for joint_name, records in active_joints.items():
                self._plot_single_joint(joint_name, records, timestamp)

        if show:
            plt.show()

        plt.close(fig)

    def _plot_single_joint(self, joint_name: str, records: list, timestamp: str):
        """为单个关节绘制单独的大图。"""
        fig, ax = plt.subplots(figsize=(12, 5))

        times = [t - self.start_time for t, _ in records]
        currents = [c for _, c in records]

        ax.plot(times, currents, linewidth=1.0, color='#2196F3')
        ax.fill_between(times, currents, alpha=0.15, color='#2196F3')

        mean_c = sum(currents) / len(currents) if currents else 0
        max_c = max(currents) if currents else 0
        min_c = min(currents) if currents else 0

        ax.axhline(
            y=mean_c,
            color='red',
            linestyle='--',
            linewidth=0.8,
            alpha=0.7,
            label=f'Mean: {mean_c:.4f} mA'
        )

        ax.set_title(
            f'{joint_name} - Current vs Time',
            fontsize=13,
            fontweight='bold'
        )

        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Current (mA)', fontsize=11)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(fontsize=9)

        stats_text = (
            f'Samples: {len(records)} | '
            f'Max: {max_c:.4f}mA | '
            f'Min: {min_c:.4f}mA | '
            f'Mean: {mean_c:.4f}mA'
        )

        ax.text(
            0.5,
            -0.12,
            stats_text,
            transform=ax.transAxes,
            fontsize=9,
            ha='center',
            va='top',
            bbox=dict(
                boxstyle='round',
                facecolor='lightgray',
                alpha=0.7
            )
        )

        fig.tight_layout()

        safe_name = joint_name.replace('/', '_')
        filepath = os.path.join(
            self.output_dir,
            f'current_{safe_name}_{timestamp}.png'
        )

        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)

        self.get_logger().info(f'💾 单关节图已保存: {filepath}')

    # ==================== 生命周期 ====================

    def shutdown(self):
        """关闭节点时的清理工作：保存 CSV + 导出图像。"""
        self.get_logger().info('🔻 正在关闭电流监控节点...')

        total_points = sum(len(v) for v in self.data.values())
        self.get_logger().info(
            f'📊 共采集 {self.frame_count} 帧反馈, {total_points} 个电流数据点'
        )

        # 停止时保存一次 CSV
        self.save_csv()

        # 停止时导出图像
        self.plot_current_curves(save=True, show=False)

        self.get_logger().info('✅ 电流监控节点已关闭')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CurrentMonitor()
    except ImportError:
        print('\n' + '=' * 60)
        print('❌ 缺少 matplotlib 依赖，请先安装:')
        print('   pip install matplotlib')
        print('=' * 60 + '\n')
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('接收到中断信号 (Ctrl+C)')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
