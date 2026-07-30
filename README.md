# ImpendenceControl

ROS 2 Humble 机器人导纳控制包，包含三部分：

- `admittance_control`：接收关节目标和电机反馈，执行导纳控制并向 RDK 串口桥发送电机命令。
- `trajectory_planner`：生成单关节三次多项式轨迹并发布 `/joint_command`。
- `current_monitor`：采集 `/serial_data` 电流反馈，退出时导出 CSV 和 PNG。

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
  -p debug_joint:="right_elbow_pitch"
```

### 终端 2：启动轨迹规划节点

```bash
ros2 run impendence_control trajectory_planner --ros-args \
  -p joint_name:="right_elbow_pitch" \
  -p start_pos:=0.0 \
  -p end_pos:=90.0 \
  -p duration:=8.0 \
  -p frequency:=90.0 \
  -p auto_start:=true
```

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

### 终端 3：启动电流监控节点

单关节监控：

```bash
ros2 run impendence_control current_monitor --ros-args \
  -p output_dir:="./current_plots" \
  -p target_joints:="right_elbow_pitch"
```

多关节监控：

```bash
ros2 run impendence_control current_monitor --ros-args \
  -p output_dir:="./current_plots" \
  -p target_joints:="right_elbow_pitch,left_elbow_pitch"
```

停止节点时会在 `output_dir` 中生成：

- `current_data_<时间>.csv`：时间、电流和关节名称。
- `current_plot_<时间>.png`：全部活动关节的电流总览。
- `current_<关节名>_<时间>.png`：单关节电流曲线。

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
| `current_monitor` | 订阅 | `/serial_data` | `rdk_x5_multi_serial/msg/SerialData` | 电流采集与绘图 |

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

电流监控参数：

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `motor_feedback_topic` | RDK 电机反馈话题 | `/serial_data` |
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
