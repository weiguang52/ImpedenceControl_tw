# ImpendenceControl

ROS 2 Humble 机器人导纳控制包，包含三部分：

- `admittance_control`：接收关节目标和电机反馈，执行导纳控制并向 RDK 串口桥发送电机命令。
- `trajectory_planner`：生成单关节单向或多次往返的三次多项式轨迹，并发布 `/joint_command`。
- `current_monitor`：采集 `/serial_data` 电流/角度反馈和 `/joint_command`
  规划角度，退出时导出 CSV 和 PNG。

> [!WARNING]
> 本项目会向真实电机发送位置命令。首次调试建议关闭导纳控制、选择单个关节、降低运动角度和频率，并确保急停可用。

## 调试顺序

以下内容保留原文的执行顺序，并补充了 Markdown 排版和说明。

### 进入工程、加载环境并构建

```bash
cd ImpendenceControl

source install/setup.bash
source /home/sunrise/git_demo/install/setup.bash

colcon build
```

首次构建时 `install/setup.bash` 还不存在，可以跳过第一条 `source`；构建完成后重新加载：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/git_demo/install/setup.bash
source install/setup.bash
```

### 终端 1：启动导纳控制核心

启用导纳控制：

```bash
ros2 run impendence_control admittance_control --ros-args \
  -p enable_admittance:=true \
  -p debug_joint:="right_elbow_pitch"
```

关闭导纳控制，仅验证轨迹和串口链路：

```bash
ros2 run impendence_control admittance_control --ros-args \
  -p enable_admittance:=false \
  -p debug_joint:="left_elbow_pitch"
```

### 终端 2：启动轨迹规划节点

```bash
ros2 run impendence_control trajectory_planner --ros-args \
  -p joint_name:="left_shoulder_pitch" \
  -p start_pos:=270.0 \
  -p end_pos:=340.0 \
  -p duration:=8.0 \
  -p frequency:=90.0 \
  -p auto_start:=true

ros2 run impendence_control trajectory_planner --ros-args \
  -p joint_name:="left_shoulder_pitch" \
  -p start_pos:=130.0 \
  -p end_pos:=0.0 \
  -p duration:=8.0 \
  -p frequency:=90.0 \
  -p auto_start:=true
```

开启往返测试（一次往返包含 `起始角度 → 结束角度` 和
`结束角度 → 起始角度` 两个运动段）：

```bash
ros2 run impendence_control trajectory_planner --ros-args \
  -p joint_name:="left_elbow_pitch" \
  -p start_pos:=360.0 \
  -p end_pos:=230.0 \
  -p duration:=8.0 \
  -p frequency:=90.0 \
  -p enable_round_trip:=true \
  -p segment_wait_duration:=2.0 \
  -p round_trip_count:=3 \
  -p auto_start:=true
```

上例共执行 3 次往返，即 6 个运动段。相邻运动段之间在刚结束的角度保持
`0.5` 秒后再继续，最后一段结束后不再等待。若不需要冷却，可设置
`segment_wait_duration:=0.0`。

单向运动还可以在指定角度暂停。例如在 `0° → 360°` 运动经过 `160°`
时硬中断 1 秒：

```bash
ros2 run impendence_control trajectory_planner --ros-args \
  -p joint_name:="left_elbow_pitch" \
  -p start_pos:=360.0 \
  -p end_pos:=230.0 \
  -p duration:=8.0 \
  -p frequency:=90.0 \
  -p enable_motion_interrupt:=true \
  -p interrupt_position:=250.0 \
  -p interrupt_duration:=1.0 \
  -p interrupt_mode:="hard" \
  -p auto_start:=true
```

中断模式：

- `hard`：先为 `start_pos → end_pos` 生成一条完整三次轨迹，在
  `interrupt_position` 对应的原轨迹时刻冻结并保持最后一条位置命令；等待结束后
  从该轨迹时刻继续。因此中断点的原规划速度通常不为零。
- `soft`：提前将轨迹拆成 `start_pos → interrupt_position` 和
  `interrupt_position → end_pos` 两段三次轨迹。`duration` 按两段角度行程比例
  分配，第一段终点和第二段起点速度均为零，两段之间等待
  `interrupt_duration`。

两种模式下，`duration` 都只表示运动时间，不包含中断等待时间。例如
`duration:=8.0`、`interrupt_duration:=1.0` 时，总执行时间约为 9 秒。
指定位置中断仅用于单向模式，不能与 `enable_round_trip:=true` 同时开启。
`interrupt_position` 必须严格位于起始角度和结束角度之间，正向和反向运动均支持。

轨迹规划参数：

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `joint_name` | 目标关节名 | `right_elbow_pitch` |
| `start_pos` | 起始角度（°） | `0.0` |
| `end_pos` | 结束角度（°） | `10.0` |
| `duration` | 运动时间（秒） | `3.0` |
| `frequency` | 控制帧频率（Hz） | `50.0` |
| `only_legs` | 是否只下发 `LEG_JOINT_NAMES` 中的关节 | `false` |
| `auto_start` | 启动 3 秒后是否自动执行 | `true` |
| `enable_round_trip` | 是否启用往返运动；关闭时只执行起点到终点 | `false` |
| `segment_wait_duration` | 相邻运动段之间的冷却等待时间（秒），允许为 `0` | `0.0` |
| `round_trip_count` | 往返次数；一次往返包含正向、反向两个运动段 | `1` |
| `enable_motion_interrupt` | 是否开启单向轨迹指定位置中断 | `false` |
| `interrupt_position` | 中断角度（°），必须严格位于起点和终点之间 | `0.0` |
| `interrupt_duration` | 中断保持时间（秒），允许为 `0` | `1.0` |
| `interrupt_mode` | 中断方式：`hard` 或 `soft` | `hard` |

### 终端 3：启动电流监控节点

单关节监控：

```bash
ros2 run impendence_control current_monitor --ros-args \
  -p output_dir:="./current_plots" \
  -p target_joints:="left_elbow_pitch"
```

多关节监控：

```bash
ros2 run impendence_control current_monitor --ros-args \
  -p output_dir:="./current_plots" \
  -p target_joints:="right_elbow_pitch,left_elbow_pitch"
```

停止节点时会在 `output_dir` 中生成：

- `current_data_<时间>.csv`：时间、电流和关节名称。
- `current_plot_<时间>.png`：全部活动关节的电流总览，同时绘制原始电流、
  5 点短窗口、30 点长窗口和锁存基线；每个子图右侧角度轴同时绘制
  `/joint_command` 规划角度与 `/serial_data` 反馈角度。
- `current_deviation_<时间>.png`：全部活动关节的电流偏差总览，绘制首次基线
  建立后的 `|短窗口电流 - 锁存基线|`，并标出各关节的碰撞阈值与恢复阈值。
  每个子图右侧纵轴显示导纳状态：`0 / Disabled` 表示未启动，
  `1 / Enabled` 表示已启动。

右轴状态按照控制器相同的逻辑重建：首次基线建立前保持关闭；电流偏差连续达到
`collision_confirm_threshold` 次超过碰撞阈值后变为开启；连续达到
`recovery_confirm_threshold` 次满足方向感知恢复条件后恢复为关闭。恢复后清空
旧长窗口并进入重新武装期，期间从当前无阻碍电流重建长窗口和基线，不触发碰撞。

不再按关节名称分别输出单关节 PNG；两个 PNG 都使用多子图总览方式。

图中的蓝线为原始电流，橙线为 5 点短窗口电流，青线为 30 点长窗口电流，绿色
虚线阶梯为锁存基线；不再绘制全部样本的总体平均线。短窗口始终连续计算。正常
状态下长窗口持续采样，绿色基线逐点复制青色长窗口；短窗口相对基线第一次超过
碰撞阈值时，长窗口和绿色基线同时冻结。满足恢复确认条件后，长窗口恢复采样，
旧长窗口样本被丢弃，绿色基线根据释放后的新样本重新建立并继续跟随。灰色背景
区域表示冷启动屏蔽期，该区域只绘制原始电流，短窗口、长窗口、基线和 deviation
都不计算。

短、长窗口长度分别读取 `COMMON_PARAMS['short_filter_window_size']` 和
`COMMON_PARAMS['long_filter_window_size']`。长窗口填满前不执行碰撞判断，避免
启动瞬态误触发；窗口尚未填满时，图中的平均值使用已有样本计算。

同时会在终端按“串口号 + 板号”打印 `/serial_data` 反馈条数：

- `target_joints` 对应的串口/板号会标记为“预期”。
- 预期路由一条反馈都没有时，会标记为“预期但未收到”。
- 收到不属于当前 `target_joints` 的串口/板号数据时，会标记为“异常路由”。
- `currents` 为空的反馈仍计入反馈总数，并单独显示空电流条数。
- 未指定 `target_joints` 时，`JOINT_MOTOR_ROUTE` 中配置的全部路由均视为预期；
  收到映射表之外的串口/板号仍会报告为异常。

## RDK 串口话题协议

本项目按 `/home/sunrise/git_demo/src/rdk_x5_multi_serial` 的最新版协议收发：

- 订阅 `/serial_data`：`rdk_x5_multi_serial/msg/SerialData`，字段为
  `port`、`board_id`、`angles`、`currents`、`action_id`、`frame_index`。
- 发布 `/serial_cmd`：`std_msgs/msg/String`，每块控制板单独发布一条 JSON，
  字段为 `port`、`board_id`、`angles`、`action_id`、`frame_index`。
- `action_id` 固定为 `1`。
- `frame_index` 优先沿用 `/joint_command` 中的非负值；上游未提供或值小于 `0`
  时，由导纳控制节点从 `0` 开始递增。同一关节帧产生的所有控制板命令共用同一个
  `frame_index`。

`/serial_cmd` 示例：

```json
{
  "port": 4,
  "board_id": 2,
  "angles": [30.0, 0.0],
  "action_id": 1,
  "frame_index": 42
}
```

`/serial_data` 消息定义：

```text
uint8 port
uint8 board_id
float32[] angles
float32[] currents
int32 action_id
int32 frame_index
```

## 数据链路

```text
trajectory_planner
  └─ /joint_command (std_msgs/String)
       └─ admittance_control
            ├─ /serial_cmd (std_msgs/String) ──> rdk_x5_multi_serial ──> 电机
            ├─ /motor_angle_debug (std_msgs/String)
            └─ /serial_data (SerialData) <──── rdk_x5_multi_serial <── 电机反馈
                                                    └─ current_monitor
```

## 节点与话题

| 节点/可执行程序 | 方向 | 话题 | 类型 | 用途 |
| --- | --- | --- | --- | --- |
| `trajectory_planner` | 发布 | `/joint_command` | `std_msgs/msg/String` | 28 关节目标角 |
| `admittance_control` | 订阅 | `/joint_command` | `std_msgs/msg/String` | 原始关节命令 |
| `admittance_control` | 订阅 | `/serial_data` | `rdk_x5_multi_serial/msg/SerialData` | 角度、电流和追踪编号反馈 |
| `admittance_control` | 发布 | `/serial_cmd` | `std_msgs/msg/String` | 按控制板拆分后的电机命令 |
| `admittance_control` | 发布 | `/motor_angle_debug` | `std_msgs/msg/String` | 关节角到电机角的映射调试信息 |
| `current_monitor` | 订阅 | `/serial_data` | `rdk_x5_multi_serial/msg/SerialData` | 电流及反馈角度采集与绘图 |
| `current_monitor` | 订阅 | `/joint_command` | `std_msgs/msg/String` | 规划角度采集与绘图 |

## 导纳控制节点参数

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `enable_admittance` | 是否启用导纳计算 | `false` |
| `joint_command_topic` | 关节命令话题 | `/joint_command` |
| `motor_command_topic` | RDK 电机命令话题 | `/serial_cmd` |
| `motor_feedback_topic` | RDK 电机反馈话题 | `/serial_data` |
| `debug_joint` | 单关节调试过滤；空字符串表示全部关节 | `""` |
| `enable_motor_angle_debug` | 是否发布并打印电机角映射 | `true` |
| `motor_angle_debug_interval` | 每隔多少帧输出一次角度调试信息 | `30` |
| `motor_angle_debug_topic` | 电机角调试话题 | `/motor_angle_debug` |

### 电流—力矩标定模型

每个关节都在 `admittance_calculate.py` 的 `JOINT_CONFIGS` 中单独配置线性标定模型：

```text
力矩 (N·m) = torque_slope_nm_per_a × 电流 (A) + torque_intercept_nm
```

- `/serial_data.currents` 的原始值按 mA 接收，进入控制器前乘以 `0.001`
  转换为 A，再代入标定模型。
- `torque_slope_nm_per_a` 是当前关节的标定斜率，单位为 N·m/A。
- `torque_intercept_nm` 是当前关节的标定截距，单位为 N·m。
- 每个关节必须分别填写自己的标定结果；控制器会根据 `joint_id` 读取对应参数。
- `expected_torque` 仍是每个关节的期望力矩，单位为 N·m。
- `current_threshold` 和 `current_recovery_threshold` 仍用于基于电流偏差的
  碰撞检测，单位为 A，具体数值按关节独立配置。
- `damping_coeff`、`stiffness_coeff` 是各关节独立的导纳参数，应以当前
  `JOINT_CONFIGS` 和实机调试结果为准。
- 当 `stiffness_coeff` 为 `0` 时，本次控制会跳过位置调整并输出错误日志，
  避免除零产生异常命令。

当前 `JOINT_CONFIGS` 中的标定参数分组如下。这里仅用于调试核对，运行时始终以
源码中的每关节配置为准：

| 斜率（N·m/A） | 截距（N·m） | 关节 |
| ---: | ---: | --- |
| `1.182535` | `-0.025172` | `right_shoulder_roll`、`right_elbow_pitch`、`left_shoulder_roll`、`left_elbow_pitch`、`right_shoulder_pitch`、`neck_pitch`、`right_hip_roll`、`right_knee_pitch`、`right_ankle_pitch`、`left_knee_pitch`、`left_ankle_pitch` |
| `1.328629` | `-0.022189` | `right_shoulder_yaw`、`right_wrist_yaw`、`left_shoulder_yaw`、`left_wrist_yaw`、`left_shoulder_pitch`、`neck_roll`、`neck_yaw`、`waist_pitch`、`waist_roll`、`right_hip_pitch`、`left_hip_pitch`、`waist_yaw`、`right_hip_yaw`、`right_ankle_yaw`、`left_hip_roll`、`left_hip_yaw`、`left_ankle_yaw` |

当前已单独实机调整的关节参数如下，运行时仍以
`src/impendence_control/admittance_calculate.py` 中的配置为准：

| 关节 | 碰撞阈值（A） | 恢复阈值（A） | 期望力矩（N·m） | 导纳方向 | 阻尼 | 刚度 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `right_shoulder_roll` | `0.07` | `0.03` | `0.04` | `-1.0` | `0.001` | `2.0` |
| `right_elbow_pitch` | `0.04` | `0.02` | `0.02` | `-1.0` | `0.001` | `1.0` |
| `left_elbow_pitch` | `0.04` | `0.02` | `0.04` | `-1.0` | `0.001` | `2.0` |
| `right_shoulder_pitch` | `0.10` | `0.05` | `0.04` | `-1.0` | `0.001` | `2.0` |

### 电流处理、碰撞检测与导纳计算

#### 执行链路

每次收到 `/joint_command` 后，导纳节点使用最近一次 `/serial_data` 缓存，按以下
顺序处理每个关节：

1. 每个关节首次进入导纳处理后，先执行 `1.0 s` 冷启动屏蔽；期间完全忽略电流，
   不更新窗口、不建立基线、不判断碰撞，也不输出导纳调整。
2. RDK 的 `currents[]` 按 mA 接收，乘以 `CURRENT_MA_TO_A = 0.001` 转换为 A。
3. 根据 `port + board_id + motor_index` 找到对应关节的电流和实际角度。
4. 当前电流始终加入 5 点短窗口；正常状态下同时加入 30 点长窗口。
5. 正常状态下，让 `baseline_current` 逐点跟随长窗口电流。
6. 使用短窗口电流与更新前基线的绝对偏差触发碰撞，并使用首次超限时记录的偏差
   方向判断阻碍是否解除。
7. 使用当前关节的斜率和截距，将当前电流转换为力矩。
8. 计算力矩误差、速度变化和本周期新增调控量。
9. 碰撞状态下累计调控量并对累计总量限幅；累计量绝对值大于 `0.01°` 时，
   使用“规划角度 + 累计调控量”替换原规划角度。

#### 滑动平均与基准电流

两个 `CurrentFilter` 分别保存短窗口和长窗口样本，并维护滑动和：

```text
短窗口电流 = 最近 5 个电流样本之和 / 当前短窗口样本数
长窗口电流 = 最近 30 个电流样本之和 / 当前长窗口样本数
```

默认短窗口为 `5` 个样本，长窗口为 `30` 个样本。长窗口填满后，使用当时的
长窗口电流建立首次基线。首次基线建立后，正常状态下每次控制都会让基线复制
最新长窗口电流。

> [!IMPORTANT]
> 当前实现中，关节初始化时 `baseline_current = 0 A`。前 `1.0 s` 冷启动电流
> 完全丢弃，屏蔽结束后才从空窗口开始累计。随后长窗口填满 30 个有效样本之前
> 不会执行电流偏差阈值判断，也不会进入碰撞状态；首次基线建立后才启用
> 碰撞与恢复判断。冷启动结束后，短窗口在任何状态下都持续计算；第一次超出碰撞
> 阈值时，长窗口和基线电流同时冻结，疑似碰撞与已确认碰撞期间的样本都不会进入
> 长窗口。满足恢复确认条件后清空旧长窗口，在重新武装期内使用释放后的电流重新
> 填满长窗口，基线复制新长窗口并逐点跟随。
> 在 90 Hz 且该关节每帧均执行的情况下，短窗口约为 `0.06 s`，长窗口约为
> `0.33 s`，因此碰撞检测最早约在启动后 `1.33 s` 开始。实际时间会随命令频率
> 和该关节是否参与当前命令变化。

#### 碰撞检测与恢复

电流偏差定义为：

```text
short_current = short_filter.get_average()
long_current = long_filter.get_average()
signed_deviation = short_current - baseline_current
current_deviation = abs(short_current - baseline_current)
```

未处于碰撞状态时：

```text
current_deviation > current_threshold
```

第一次满足条件时立即冻结基线，并从该次开始累计；连续满足
`collision_confirm_threshold` 次后进入碰撞状态。尚未确认碰撞时，若偏差不再
超过碰撞阈值则碰撞计数清零。第一次超限时同时记录：

```text
collision_direction = sign(signed_deviation)
```

后续的碰撞确认只累计原碰撞方向上的连续超限，避免电流越过基线后在相反方向上
继续累计同一次碰撞。

已处于碰撞状态时：

```text
directional_deviation = collision_direction × signed_deviation
directional_deviation < current_recovery_threshold
```

连续满足 `recovery_confirm_threshold` 次后退出碰撞状态；任意一次不满足，恢复
计数清零。因为恢复偏差保留了碰撞方向，所以电流越过冻结基线时，
`directional_deviation` 会变为负数，可正确识别为卸载或回弹，不会因绝对偏差
仍然很大而继续保持碰撞。

退出碰撞后会清空旧长窗口，并等待 `recovery_rearm_samples` 个释放后样本重新
建立长窗口和基线；在此期间导纳关闭、碰撞检测禁用。这样新的无阻碍负载电流不会
继续与碰撞前的旧基线比较。恢复阈值通常应小于碰撞阈值，方向判断、双阈值和重新
武装期共同用于避免状态抖动和放开阻碍后的误触发。

当前碰撞确认次数为 `5`，恢复确认次数为 `5`。如果控制频率为 90 Hz 且每帧均
执行，理论上两者分别需要约 `56 ms`；滑动平均窗口本身还会增加响应延迟，
实际时延也受 ROS 调度和反馈频率影响。

#### 力矩与位置调整

当前关节的标定力矩和力矩误差为：

```text
measured_torque = torque_slope_nm_per_a × current_current + torque_intercept_nm
torque_error = measured_torque - expected_torque
```

代码根据实际角度计算速度及相邻两次速度变化：

```text
velocity = (current_position - last_position) / dt
velocity_difference = velocity - last_velocity
```

每个控制周期首先计算本周期新增调控量：

```text
directed_torque_error = admittance_direction × torque_error

position_adjustment_increment =
    (directed_torque_error - damping_coeff × velocity_difference)
    / stiffness_coeff
```

`admittance_direction` 按关节独立配置。`+1` 保持电流—力矩模型的原方向，`-1`
只反转外力对应的让步方向，阻尼项仍保持抑制运动变化的符号。根据
`left_shoulder_roll` 的实机阻挡测试，该关节设为 `-1.0`；其余未显式配置的关节
默认使用 `+1.0`，需要逐关节通过低限幅实机测试确认。

碰撞状态持续期间，本周期新增量会叠加到上一周期保留的累计调控量；限幅作用于
累计总量，而不是单独作用于本周期新增量：

```text
accumulated_position_adjustment = clamp(
    accumulated_position_adjustment + position_adjustment_increment,
    -max_position_adjustment,
    +max_position_adjustment
)

target_position =
    planned_position + accumulated_position_adjustment
```

因此，即使规划角度保持不变，上一次已经应用的调控量也会在下一周期保留，并继续
叠加本周期计算出的新增量。碰撞解除、冷启动屏蔽或刚度参数无效时，累计调控量复位
为 `0°`，目标位置回到当前规划轨迹。默认累计调控量最大绝对值为 `30°`。

当 `stiffness_coeff` 接近零时，代码会放弃本次调整并返回规划位置，避免除零。

#### 电流与导纳相关参数

每关节参数位于 `JOINT_CONFIGS`：

| 参数 | 单位 | 作用 | 调大后的主要影响 |
| --- | --- | --- | --- |
| `current_threshold` | A | 进入碰撞状态的电流偏差阈值 | 降低灵敏度，更不容易触发 |
| `current_recovery_threshold` | A | 退出碰撞状态的电流偏差阈值 | 更容易退出碰撞，但过大可能产生状态抖动 |
| `torque_slope_nm_per_a` | N·m/A | 当前关节电流—力矩模型斜率 | 相同电流对应更大的计算力矩 |
| `torque_intercept_nm` | N·m | 当前关节电流—力矩模型截距 | 整体平移计算力矩 |
| `expected_torque` | N·m | 导纳控制的期望力矩 | 改变力矩误差的平衡点 |
| `admittance_direction` | `+1` 或 `-1` | 当前关节外力到让步方向的符号 | 只反转力矩产生的调控方向 |
| `damping_coeff` | 当前角度制实现的数值增益 | 抑制相邻控制周期的速度变化 | 通常减小快速位置调整 |
| `stiffness_coeff` | 当前角度制实现的数值增益 | 力矩误差到角度调整的比例分母 | 相同力矩误差产生更小的角度调整 |

所有关节共用参数位于 `COMMON_PARAMS`：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `collision_confirm_threshold` | `5` | 连续多少次超过电流阈值才确认碰撞 |
| `recovery_confirm_threshold` | `5` | 连续多少次满足恢复条件才确认恢复 |
| `short_filter_window_size` | `5` | 碰撞响应短窗口长度 |
| `long_filter_window_size` | `30` | 正常趋势与基线候选长窗口长度 |
| `recovery_rearm_samples` | `30` | 恢复后重建长窗口且暂不触发碰撞的样本数 |
| `cold_start_ignore_duration_sec` | `1.0 s` | 每个关节首次处理时完全忽略电流的时长 |
| `max_position_adjustment` | `30.0°` | 累计导纳调控量的绝对值上限 |

调参建议：

- 电流噪声较大时，可以增大短窗口或确认次数，但响应会变慢。
- 长窗口过短会追随异常变化，过长会延迟基线恢复；当前数据建议从 `30` 点开始。
- 应先根据正常运动电流波动设置碰撞和恢复阈值，再调节刚度、阻尼与最大调整量。
- 修改斜率或截距后，应重新检查 `expected_torque` 和实际位置调整方向。
- 腿部当前部分碰撞阈值为 `2 A`，明显高于上肢配置；调试时应结合实际电流曲线
  判断这是有意屏蔽还是最终阈值。

> [!NOTE]
> `current_monitor` 节点直接记录 RDK 消息中的电流数值并以 mA 标注；导纳控制节点
> 则会将同一反馈乘以 `0.001` 后按 A 参与碰撞和力矩计算。对照日志或 CSV 时应注意
> 两个节点显示单位不同。图中的短窗口、长窗口和基线均以 mA 显示，其窗口长度
> 与导纳控制器保持一致。电流偏差图在长窗口填满前留空，因为该阶段控制器同样
> 不会执行阈值判断。

电流监控参数：

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `motor_feedback_topic` | RDK 电机反馈话题 | `/serial_data` |
| `joint_command_topic` | 用于记录规划角度的关节命令话题 | `/joint_command` |
| `output_dir` | CSV 和图片输出目录 | `./current_plots` |
| `target_joints` | 逗号分隔的关节名；空字符串表示全部 | `""` |

## 单独验证话题

确认 RDK 消息已经是最新版：

```bash
ros2 interface show rdk_x5_multi_serial/msg/SerialData
```

观察反馈和追踪编号：

```bash
ros2 topic echo /serial_data
```

观察本项目实际下发给 RDK 的 JSON：

```bash
ros2 topic echo /serial_cmd
```

检查话题类型与发布/订阅者：

```bash
ros2 topic info /serial_cmd -v
ros2 topic info /serial_data -v
```

绕过轨迹节点，手动测试一块控制板：

```bash
ros2 topic pub --once /serial_cmd std_msgs/msg/String \
  "{data: '{\"port\":4,\"board_id\":2,\"angles\":[30.0,0.0],\"action_id\":1,\"frame_index\":0}'}"
```

> [!CAUTION]
> 手动发布前必须核对 `port`、`board_id`、电机数量、零位和角度范围。错误映射可能造成机器人突然运动。

## 常见问题

### 找不到 `rdk_x5_multi_serial`

先构建并加载 RDK 工作空间，再构建本项目：

```bash
cd /home/sunrise/git_demo
source /opt/ros/humble/setup.bash
colcon build --packages-select rdk_x5_multi_serial
source install/setup.bash

cd /home/sunrise/ImpendenceControl
colcon build
source install/setup.bash
```

### `SerialData` 没有 `action_id` 或 `frame_index`

当前终端加载了旧版 RDK 接口。重新构建 `rdk_x5_multi_serial`，并确认
`ros2 interface show rdk_x5_multi_serial/msg/SerialData` 输出包含这两个字段。

### `/serial_cmd` 有数据但电机不动作

依次检查：

1. `rdk_x5_multi_serial` 节点是否启动、串口是否成功打开。
2. `/serial_cmd` 的消息类型是否为 `std_msgs/msg/String`。
3. JSON 中的 `port`、`board_id` 和 `angles` 数量是否匹配硬件。
4. `action_id` 是否为 `1`，`frame_index` 是否为非负递增值。
5. 通过 RDK 日志确认命令是否进入发送队列。

### 电流监控没有生成图

- 确认 `/serial_data` 中 `currents` 非空。
- 确认 `target_joints` 名称存在于 `JOINT_MOTOR_ROUTE`。
- 安装绘图库：`python3 -m pip install matplotlib`。
- 使用 `Ctrl+C` 正常停止节点，CSV 和 PNG 会在退出阶段生成。

## 项目结构

```text
ImpendenceControl/
├── src/impendence_control/
│   ├── admittance_calculate.py  # 导纳控制、关节/电机映射、串口话题收发
│   ├── trajectory_planner.py    # 单关节三次多项式轨迹
│   └── current_monitor.py       # 电流采集、CSV 和 PNG 导出
├── resource/
├── package.xml
├── setup.cfg
├── setup.py
├── README.md
└── AGENTS.md
```

`build/`、`install/`、`log/`、`current_plots/` 是本机构建或运行产物，不应提交。

## 开发验证

```bash
cd /home/sunrise/ImpendenceControl
source /opt/ros/humble/setup.bash
source /home/sunrise/git_demo/install/setup.bash
colcon build --packages-select impendence_control
```

构建后建议至少执行：

```bash
source install/setup.bash
ros2 pkg executables impendence_control
ros2 interface show rdk_x5_multi_serial/msg/SerialData
```

## 远端仓库

- GitHub：<https://github.com/weiguang52/ImpedenceControl_tw>
- Linux 工作目录：`/home/sunrise/ImpendenceControl`
- RDK 协议源码：`/home/sunrise/git_demo/src/rdk_x5_multi_serial`
