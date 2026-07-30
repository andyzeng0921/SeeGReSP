# Robot 306 自适应视觉抓取

ROS 2 包名：`adaptive_object_grasping`

项目唯一部署目录：

```text
/home/ubuntu/zeng-Visual Grasping
```

所有源码、模型、虚拟环境、仿真实验、运行缓存和日志都保存在该目录内。本项目
不向厂商 `robot_env` 安装依赖。

## 当前能力

- AutoLife S2 左、右机械臂各 7 DoF；厂商轨迹格式为腰腿 4 + 左臂 7 +
  右臂 7，共 18 个值。
- 实时 RGB-D、YOLO11 实例分割/跟踪、目标选择、GraspNet 6D 候选、
  MoveIt 规划、双臂轨迹适配和可视化。
- 当前执行语义是“一只主动臂抓取，另一只臂保持”，不是双臂协同搬运。
- 真实运动只有 `MotionExecutor` 一个出口，且交付配置保持锁定。
- 已跑通 robosuite `TwoArmLift / Panda × 2` 的 14 维双七轴无硬件仿真。

完整设计和参考项目分别见：

- `docs/ARCHITECTURE_V2_ZH.md`
- `docs/HIGH_STAR_DUAL_ARM_REFERENCES_ZH.md`
- `HANDOFF.md`

## V2 数据链

```text
RealSense RGB-D
  -> YOLO11-seg + track id
  -> 项目内原子 observation 文件（RGB/depth/mask/intrinsics）
  -> token + size + SHA256
  -> GraspNet + 场景碰撞过滤
  -> 对同一 track 的新帧重验证
  -> MoveIt dry-run
  -> MotionExecutor 安全门
  -> AutoLife 18 关节适配器（当前锁定）
```

选中目标的完整观测只写到：

```text
/home/ubuntu/zeng-Visual Grasping/runtime/selected_observations
```

ROS 服务只传递令牌、长度和摘要，接收端会校验项目根目录、文件类型、大小、
SHA256、track、时间戳、frame、shape 和 dtype，消费后删除。这样避免把约 5 MB
的 RGB-D 数据来回塞入 DDS 服务，也不再拼接三路相互错帧的缓存。

## 构建与纯测试

```bash
cd "/home/ubuntu/zeng-Visual Grasping"
bash tools/build_project.sh
third_party/venv/bin/python tools/check_dual_arm_profile.py
```

`build_project.sh` 会构建包并运行测试。源码配置修改后必须重新构建，因为节点读取
的是 `install/adaptive_object_grasping/share/...` 下的已安装配置。

## 安全试运行

以下命令不会请求机器人运动：

```bash
cd "/home/ubuntu/zeng-Visual Grasping"
tools/graspctl.sh start
tools/graspctl.sh status
tools/graspctl.sh list
tools/graspctl.sh plan bottle auto
tools/graspctl.sh logs 200
tools/graspctl.sh stop
```

`plan` 的安全失败是正常结果：目标移动、遮挡、候选被碰撞/宽度/方向过滤，或
重验证位置变化超过阈值时都会 fail closed。不要为得到“成功”而放宽这些阈值。

`graspctl.sh stop` 只停止本项目 ROS 节点，不是机器人停止命令，也不能终止已经
下发的厂商轨迹。现场立即停止只能使用经过验证的实体急停。

## 当前执行锁

交付配置为：

```yaml
dry_run: true
allow_hardware_execution: false
enable_pbvs_hardware_follow: false
```

即使 `/check_grasp_hardware` 返回 `ready=true`，响应仍会明确包含
`hardware_execution_locked=true`。探针只证明反馈链当前在线，不能证明相机/TCP
标定、环境碰撞、实体急停或现场验收已经完成。

当前唯一允许接入真实执行的后端是 `moveit_py_vendor_execution`。旧
`vendor_rrt`、`vendor_task_space` 仅保留 dry-run；PBVS 直接跟随硬件不受支持，
开启其开关会阻止解锁。

在完成 PlanningScene 环境障碍、手眼与双 TCP 标定、夹爪端点映射、实体急停试验
和签名现场验收之前，不得修改执行锁，也不得使用 `execute`。

## 模型与许可证

- YOLO 默认使用 `models/yolo/yolo11n-seg.pt`，只能识别其训练类别。
- GraspNet baseline 与权重由使用者按其学术/非商业许可独立安装；本项目只保留
  薄适配层。
- 高 Star 参考代码按许可证和职责边界借鉴，没有整仓复制进实机控制链。
