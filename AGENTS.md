# AGENTS.md

## 项目与仓库

- 项目名称：`ImpendenceControl`
- Linux 工作目录：`/home/sunrise/ImpendenceControl`
- GitHub 仓库：<https://github.com/weiguang52/ImpedenceControl_tw>
- Git remote：`origin`
- 默认分支：`main`
- RDK 最新协议来源：`/home/sunrise/git_demo/src/rdk_x5_multi_serial`

## 开发约束

- 修改串口话题前，必须先以 RDK 项目当前的 `SerialData.msg` 和
  `multi_serial_bridge.cpp` 为准核对字段。
- `/serial_cmd` 使用 `std_msgs/msg/String`，JSON 字段为 `port`、`board_id`、
  `angles`、`action_id`、`frame_index`。
- `/serial_data` 使用 `rdk_x5_multi_serial/msg/SerialData`。
- 当前项目的 `action_id` 固定为 `1`；不要改为上游透传或自动递增，除非用户明确要求。
- 同一关节控制帧拆出的多块控制板命令必须共用同一个 `frame_index`。
- 保持 README 中现有调试命令的相对顺序：构建 → 导纳控制 → 轨迹规划 →
  电流监控 → RDK 协议。可以补充和美化，但不要打乱顺序。
- 不提交 `build/`、`install/`、`log/`、`current_plots/`、Python 缓存或编辑器临时文件。

## 构建与检查

在 Linux 远端执行：

```bash
cd /home/sunrise/ImpendenceControl
source /opt/ros/humble/setup.bash
source /home/sunrise/git_demo/install/setup.bash
python3 -m py_compile src/impendence_control/*.py
colcon build --packages-select impendence_control
```

消息协议检查：

```bash
ros2 interface show rdk_x5_multi_serial/msg/SerialData
ros2 topic info /serial_cmd -v
ros2 topic info /serial_data -v
```

## 提交与推送

- 尽量在 Linux 工作目录内执行 Git 操作，不在 Windows 本地副本中维护独立提交。
- 提交前运行 `git status --short` 和上述构建检查。
- 提交信息应直接描述变更，例如：
  `docs: improve debugging guide and record repository workflow`。
- 推送目标为 `origin/main`：

```bash
git add README.md AGENTS.md .gitignore src package.xml setup.py setup.cfg resource
git commit -m "docs: improve debugging guide and record repository workflow"
git push -u origin main
```

- 禁止提交密钥、令牌、串口运行日志和机器人采集数据。
- 禁止使用 `git push --force`，除非用户明确要求并确认影响。

