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
  - 电流偏差 PNG 图像: <output_dir>/current_deviation_<timestamp>.png
  - CSV 数据: <output_dir>/current_data_<timestamp>.csv
"""

import rclpy
import time
import os
import csv
import math
from collections import defaultdict
from rclpy.node import Node
from rdk_x5_multi_serial.msg import SerialData
from impendence_control.admittance_calculate import (
    COMMON_PARAMS,
    JOINT_CONFIGS,
    JOINT_MOTOR_ROUTE,
)

# 尝试导入 matplotlib；若不可用则给出清晰提示
try:
    import matplotlib
    matplotlib.use('Agg')  # 非交互式后端，支持无 GUI 环境导出图像
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


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
        self.current_average_window = max(
            1,
            int(COMMON_PARAMS['filter_window_size']),
        )
        self.baseline_update_interval = max(
            1,
            int(COMMON_PARAMS['baseline_update_interval']),
        )
        self.feedback_counts = defaultdict(int)
        self.unexpected_feedback_counts = defaultdict(int)
        self.empty_current_feedback_counts = defaultdict(int)
        self.feedback_parse_error_count = 0

        # 根据 target_joints 推导本次监控期望收到的 (串口号, 板号)。
        # 未指定 target_joints 时，JOINT_MOTOR_ROUTE 中的全部路由均视为预期路由。
        self.expected_route_joints = defaultdict(list)
        for joint_name, route in JOINT_MOTOR_ROUTE.items():
            if self.target_joints and joint_name not in self.target_joints:
                continue

            route_key = (
                int(route['serial_id']),
                int(route['board_id']),
            )
            self.expected_route_joints[route_key].append(joint_name)

        self.expected_feedback_routes = set(self.expected_route_joints.keys())

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

        expected_routes_text = ', '.join(
            f'串口 {serial_id}/板号 {board_id}'
            for serial_id, board_id in sorted(self.expected_feedback_routes)
        )
        self.get_logger().info(
            f'  预期反馈路由: {expected_routes_text or "无有效路由"}'
        )
        self.get_logger().info(
            f'  电流窗口平均: {self.current_average_window} 点'
        )
        self.get_logger().info(
            f'  基线更新间隔: {self.baseline_update_interval} 点'
        )
        self.get_logger().info('  CSV 保存方式: 停止时保存一次')
        self.get_logger().info('  图像导出方式: 停止时导出')
        self.get_logger().info('=' * 60)

    # ==================== 数据采集 ====================

    def motor_feedback_callback(self, msg: SerialData):
        """接收新版 /serial_data 反馈，记录电流数据。"""
        try:
            serial_id = int(msg.port)
            board_id = int(msg.board_id)
            route_key = (serial_id, board_id)

            # 反馈条数按 /serial_data 消息计数，即使 currents 为空也计入。
            self.feedback_counts[route_key] += 1
            self.frame_count += 1
            if route_key not in self.expected_feedback_routes:
                self.unexpected_feedback_counts[route_key] += 1

            currents = [float(x) for x in msg.currents]
            action_id = int(msg.action_id)
            frame_index = int(msg.frame_index)
            self.last_action_id = action_id
            self.last_frame_index = frame_index

            if not currents:
                self.empty_current_feedback_counts[route_key] += 1
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

        except Exception as e:
            self.feedback_parse_error_count += 1
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

    @staticmethod
    def _moving_average(values: list, window_size: int) -> list:
        """计算与 CurrentFilter 一致的滑动窗口平均，返回等长序列。"""
        if not values:
            return []

        window_size = max(1, int(window_size))
        moving_average = []
        running_sum = 0.0

        for index, value in enumerate(values):
            running_sum += float(value)
            if index >= window_size:
                running_sum -= float(values[index - window_size])

            sample_count = min(index + 1, window_size)
            moving_average.append(running_sum / sample_count)

        return moving_average

    @staticmethod
    def _baseline_series(window_currents: list, update_interval: int) -> list:
        """按照导纳控制器的更新间隔重建阶梯状基线电流序列。"""
        update_interval = max(1, int(update_interval))
        baseline_currents = []
        baseline_current = 0.0

        for sample_index, window_current in enumerate(window_currents, start=1):
            if sample_index % update_interval == 0:
                baseline_current = float(window_current)
            baseline_currents.append(baseline_current)

        return baseline_currents

    @staticmethod
    def _admittance_enabled_series(
        deviations: list,
        collision_threshold: float,
        recovery_threshold: float,
        collision_confirm_count: int,
        recovery_confirm_count: int,
    ) -> list:
        """按照碰撞/恢复确认逻辑重建导纳是否启用的布尔序列。"""
        collision_confirm_count = max(1, int(collision_confirm_count))
        recovery_confirm_count = max(1, int(recovery_confirm_count))
        enabled = False
        collision_counter = 0
        recovery_counter = 0
        enabled_series = []

        for deviation in deviations:
            if math.isnan(deviation):
                enabled = False
                collision_counter = 0
                recovery_counter = 0
                enabled_series.append(0)
                continue

            if enabled:
                if deviation < recovery_threshold:
                    recovery_counter += 1
                    if recovery_counter >= recovery_confirm_count:
                        enabled = False
                        collision_counter = 0
                        recovery_counter = 0
                else:
                    recovery_counter = 0
            else:
                if deviation > collision_threshold:
                    collision_counter += 1
                    if collision_counter >= collision_confirm_count:
                        enabled = True
                        recovery_counter = 0
                else:
                    collision_counter = 0

            enabled_series.append(1 if enabled else 0)

        return enabled_series

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
            window_currents = self._moving_average(
                currents,
                self.current_average_window,
            )
            baseline_currents = self._baseline_series(
                window_currents,
                self.baseline_update_interval,
            )

            ax.plot(
                times,
                currents,
                linewidth=0.8,
                color='#2196F3',
                alpha=0.35,
                label='Raw current',
            )
            ax.plot(
                times,
                window_currents,
                linewidth=1.4,
                color='#FF9800',
                alpha=0.95,
                label=f'Window mean (N={self.current_average_window})',
            )
            ax.plot(
                times,
                baseline_currents,
                linewidth=1.2,
                color='#4CAF50',
                linestyle='--',
                drawstyle='steps-post',
                alpha=0.95,
                label=f'Baseline (K={self.baseline_update_interval})',
            )

            ax.set_title(joint_name, fontsize=10, fontweight='bold')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Current (mA)')
            ax.grid(True, alpha=0.3, linestyle='--')

            mean_c = sum(currents) / len(currents) if currents else 0
            max_c = max(currents) if currents else 0
            min_c = min(currents) if currents else 0

            ax.legend(fontsize=7, loc='upper right')

            stats_text = (
                f'Points: {len(records)}\n'
                f'Max: {max_c:.4f}mA\n'
                f'Min: {min_c:.4f}mA\n'
                f'Mean: {mean_c:.4f}mA\n'
                f'Last window mean: {window_currents[-1]:.4f}mA\n'
                f'Last baseline: {baseline_currents[-1]:.4f}mA'
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
            self._plot_current_deviations(active_joints, timestamp)

        if show:
            plt.show()

        plt.close(fig)

    def _plot_current_deviations(
        self,
        active_joints: dict,
        timestamp: str,
    ):
        """绘制首次基线建立后的 |实际电流 - 基线电流| 总览图。"""
        n_joints = len(active_joints)
        n_cols = min(3, n_joints)
        n_rows = (n_joints + n_cols - 1) // n_cols
        fig, axes = plt.subplots(
            n_rows,
            n_cols,
            figsize=(6 * n_cols, 4 * n_rows),
        )
        fig.suptitle(
            f'Current Deviation from Baseline\n'
            f'(Recorded: {time.strftime("%Y-%m-%d %H:%M:%S")})',
            fontsize=14,
            fontweight='bold',
        )

        if n_joints == 1:
            axes = [axes]
        else:
            axes = axes.flatten() if hasattr(axes, 'flatten') else axes

        for idx, (joint_name, records) in enumerate(sorted(active_joints.items())):
            ax = axes[idx]
            times = [t - self.start_time for t, _ in records]
            currents = [c for _, c in records]
            window_currents = self._moving_average(
                currents,
                self.current_average_window,
            )
            baseline_currents = self._baseline_series(
                window_currents,
                self.baseline_update_interval,
            )
            deviations = [
                (
                    abs(current - baseline)
                    if sample_index >= self.baseline_update_interval
                    else float('nan')
                )
                for sample_index, (current, baseline) in enumerate(
                    zip(currents, baseline_currents),
                    start=1,
                )
            ]

            ax.plot(
                times,
                deviations,
                linewidth=1.2,
                color='#9C27B0',
                label='|Raw current - baseline|',
            )

            config = JOINT_CONFIGS.get(joint_name)
            if config is not None:
                threshold_ma = float(config['current_threshold']) * 1000.0
                ax.axhline(
                    y=threshold_ma,
                    color='#F44336',
                    linestyle='--',
                    linewidth=1.0,
                    label=f'Collision threshold: {threshold_ma:.4f}mA',
                )
                recovery_threshold_ma = (
                    float(config['current_recovery_threshold']) * 1000.0
                )
                ax.axhline(
                    y=recovery_threshold_ma,
                    color='#00BCD4',
                    linestyle='-.',
                    linewidth=1.0,
                    label=(
                        f'Recovery threshold: '
                        f'{recovery_threshold_ma:.4f}mA'
                    ),
                )
                admittance_enabled = self._admittance_enabled_series(
                    deviations,
                    threshold_ma,
                    recovery_threshold_ma,
                    COMMON_PARAMS['collision_confirm_threshold'],
                    COMMON_PARAMS['recovery_confirm_threshold'],
                )

                state_ax = ax.twinx()
                state_ax.step(
                    times,
                    admittance_enabled,
                    where='post',
                    linewidth=1.2,
                    color='#795548',
                    label='Admittance active',
                )
                state_ax.set_ylabel(
                    'Admittance state',
                    color='#795548',
                )
                state_ax.tick_params(axis='y', labelcolor='#795548')
                state_ax.set_ylim(-0.05, 1.05)
                state_ax.set_yticks([0, 1])
                state_ax.set_yticklabels(['Disabled', 'Enabled'])
            else:
                state_ax = None

            if len(times) >= self.baseline_update_interval:
                baseline_ready_time = times[self.baseline_update_interval - 1]
                ax.axvline(
                    x=baseline_ready_time,
                    color='#4CAF50',
                    linestyle=':',
                    linewidth=1.0,
                    label='First baseline ready',
                )
            else:
                ax.text(
                    0.5,
                    0.5,
                    'Waiting for first baseline',
                    transform=ax.transAxes,
                    ha='center',
                    va='center',
                    fontsize=9,
                )

            ax.set_title(joint_name, fontsize=10, fontweight='bold')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Absolute current deviation (mA)')
            ax.grid(True, alpha=0.3, linestyle='--')
            lines, labels = ax.get_legend_handles_labels()
            if state_ax is not None:
                state_lines, state_labels = (
                    state_ax.get_legend_handles_labels()
                )
                lines += state_lines
                labels += state_labels
            ax.legend(lines, labels, fontsize=7, loc='upper right')

        for idx in range(n_joints, len(axes)):
            axes[idx].set_visible(False)

        fig.tight_layout()
        filepath = os.path.join(
            self.output_dir,
            f'current_deviation_{timestamp}.png',
        )
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f'💾 电流偏差总图已保存: {filepath}')

    # ==================== 生命周期 ====================

    def print_feedback_summary(self):
        """按串口号和板号打印本次运行收到的 /serial_data 反馈统计。"""
        total_feedback = sum(self.feedback_counts.values())
        unexpected_total = sum(self.unexpected_feedback_counts.values())
        all_routes = (
            self.expected_feedback_routes
            | set(self.feedback_counts.keys())
        )

        self.get_logger().info('=' * 70)
        self.get_logger().info(
            f'📡 /serial_data 反馈统计: 共收到 {total_feedback} 条，'
            f'涉及 {len(self.feedback_counts)} 个串口/板号组合'
        )

        if not all_routes:
            self.get_logger().warning('⚠️  当前没有有效的预期路由，也没有收到可识别的反馈')

        for serial_id, board_id in sorted(all_routes):
            route_key = (serial_id, board_id)
            count = self.feedback_counts.get(route_key, 0)
            empty_count = self.empty_current_feedback_counts.get(route_key, 0)
            expected_joints = self.expected_route_joints.get(route_key, [])

            if route_key in self.expected_feedback_routes:
                if count > 0:
                    status = '✅ 预期'
                else:
                    status = '⚠️ 预期但未收到'

                joints_text = ', '.join(expected_joints)
                route_detail = f'，关节: {joints_text}' if joints_text else ''
            else:
                status = '❌ 异常路由'
                route_detail = '，不属于本次 target_joints 的预期反馈'

            empty_detail = (
                f'，其中 currents 为空 {empty_count} 条'
                if empty_count
                else ''
            )
            self.get_logger().info(
                f'  {status} | 串口 {serial_id} | 板号 {board_id} | '
                f'{count} 条{empty_detail}{route_detail}'
            )

        if unexpected_total:
            self.get_logger().warning(
                f'❌ 检测到异常串口/板号反馈共 {unexpected_total} 条，'
                '请检查 RDK 发布范围、串口映射和 target_joints 配置'
            )
        else:
            self.get_logger().info('✅ 未检测到异常串口/板号反馈')

        if self.feedback_parse_error_count:
            self.get_logger().warning(
                f'⚠️  另有 {self.feedback_parse_error_count} 条反馈解析失败，'
                '未能完整归入上述统计'
            )

        self.get_logger().info('=' * 70)

    def shutdown(self):
        """关闭节点时的清理工作：保存 CSV + 导出图像。"""
        self.get_logger().info('🔻 正在关闭电流监控节点...')

        total_points = sum(len(v) for v in self.data.values())
        self.get_logger().info(
            f'📊 共采集 {self.frame_count} 帧反馈, {total_points} 个电流数据点'
        )
        self.print_feedback_summary()

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
