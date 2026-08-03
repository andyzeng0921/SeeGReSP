# 306 桌面场景与可视复位姿态标定

本流程不会自动解锁硬件。`environment_scene.yaml` 与
`visible_reset_pose.yaml` 必须分别经过现场核对并保留操作员、时间和来源样本；
任一文件的 `verified` 不是 `true` 时，MoveIt 会拒绝抓取或复位规划。

## 1. 采集并拟合桌面

```bash
cd "/home/ubuntu/zeng-Visual Grasping"
./tools/graspctl.sh start
./tools/graspctl.sh scene bottle
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONPATH="$PWD" third_party/venv/bin/python tools/calibrate_environment.py \
  runtime/scene_samples/<样本1> \
  runtime/scene_samples/<样本2> \
  runtime/scene_samples/<样本3> \
  --output runtime/calibration/environment_scene.candidate.yaml
```

建议在机器人、桌子和头部均静止时采集至少 5 个同帧 RGB-D 样本。多样本模式分别
拟合每一帧，再对顶面高度和四条可见边界取中位数；高度极差超过 3 cm 会拒绝融合。
用卷尺核对 `top_height_m`、盒体中心和长宽，并在 RViz 确认桌面盒不与机器人当前
模型重叠。核对后将候选复制为 `config/environment_scene.yaml`，填写操作员和时间，
最后才可把 `verified` 改为 `true`。

## 2. 采集可视复位姿态

先由现场厂商示教/手动安全方式把双臂放到头部相机能同时看到双腕和双 TCP 的位置。
不得用本项目未标定的旧 `current` SRDF 状态直接运动。随后重新运行 `scene`，并执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/calibrate_visible_reset.py \
  runtime/scene_samples/<可视姿态样本目录> \
  --output runtime/calibration/visible_reset_pose.candidate.yaml
```

工具会拒绝任何腕部/TCP 不在图像内或没有对齐深度证据的样本。现场核对关节值、
限位和模型后，将候选复制为 `config/visible_reset_pose.yaml`，填写操作员和时间，再设置
`verified: true`。

## 3. 仅规划复位

```bash
./tools/build_project.sh
./tools/graspctl.sh start
./tools/graspctl.sh reset-plan
```

复位规划按厂商能力拆成“左臂 → 右臂”两段，并在已标定桌面场景中检查碰撞。
`reset-plan` 只发布 RViz 目标影子，不存在硬件执行入口。

## 4. 2026-08-03 实机更新

- 厂商保护已恢复为 `expected=31, ready=31, armed=true`。
- 头部俯仰低速移动至 `-31.99°` 后锁定；左右夹爪均进入 `1280×720` 头部 RGB-D
  画面。右 TCP 深度一致性通过（预测 `0.551 m`，观测 `0.574 m`，误差 `22.8 mm`）；
  左 TCP 像素邻域无有效深度，仍不能作为完整可视复位验收样本。
- 固定头部连续 5 帧的桌面深度融合候选为：顶面 `z=0.77097 m`，尺寸
  `0.58518 × 0.77775 × 0.06 m`，中心
  `[0.87001, 0.21870, 0.74097] m`；高度极差 `14.8 mm`，总内点 `125735`。
- 候选文件为 `runtime/calibration/environment_scene.head_minus32.fused.candidate.yaml`，
  保持 `verified: false`，必须经过 RViz 重叠检查后才能用于 MoveIt。

## 5. 当前实机阻断

- 左 TCP 尚无深度一致性证据，不能生成完整可视复位标定样本。
- 当前瓶子预抓取目标经确定性 11 种子 KDL 检查仍不可达，需先完成复位/底盘相对
  桌面的现场定位，再重新生成候选。
