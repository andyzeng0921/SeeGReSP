# Robot 306 视觉抓取 V2 交接

更新时间：2026-07-30

## 结论

V2 已把感知快照、抓取推理、目标重验证、MoveIt 规划和厂商执行边界拆开，并默认
fail closed。本轮没有向机械臂发送任何运动或夹爪命令。

服务器唯一工作目录：

```text
/home/ubuntu/zeng-Visual Grasping
```

本轮修改前备份：

```text
/home/ubuntu/zeng-Visual Grasping/runtime/backups/pre-architecture-v2-20260730-153500.tar.gz
```

## 已完成

1. 固化 AutoLife S2 左 7 + 右 7 关节契约，以及厂商腰腿 4 + 双臂 14 的
   18 值轨迹顺序。
2. `MotionExecutor` 成为唯一硬件出口：执行互斥、精确确认令牌、现场验收、
   新鲜硬件状态和执行配置缺一不可。
3. 真实执行只允许 `moveit_py_vendor_execution`。厂商 RRT/末端位姿后端只可
   dry-run；PBVS 硬件直发已移除。
4. 任一硬件命令发布后若响应超时或异常，锁存 unknown-motion fault；进程内
   无清除入口，后续请求全部拒绝。没有验证的软件 stop，实体急停始终是权威。
5. 硬件探针严格检查 31/31 heartbeat、protection、self-collision、关节通信和
   完整双 EEF 内容；探针 ready 本身永远不能解锁。
6. 删除业务代码写厂商私有 `/dev/shm/pid_loop_state_*` 的行为。
7. MoveIt `time_from_start` 被重采样到厂商 18 值轨迹，总时长不短于规划结果；
   发布后只接受更新的双 EEF 反馈作为完成依据。
8. YOLO 使用实例 mask，并把同一帧 RGB、depth、mask、内参原子写入项目内
   observation store；GraspNet 通过受校验的令牌消费。
9. GraspNet 启动时预加载并 GPU warm-up；场景点云先按目标附近裁剪，再以
   1 cm 确定性体素化后进入官方碰撞检测。
10. 推理后重新选择同一 track 的新帧，校验类别、frame、三维位移、bbox 中心和
    IoU；验证通过后才更新时间戳并进入规划。
11. 协调器区分 unavailable/timeout/error，关闭 cancel 竞态，拒绝空确认、
    并发 goal 和 unknown-motion 状态下的新请求。
12. 已在项目内隔离运行 robosuite 1.5.2 + MuJoCo 3.3.7 双 Panda 冒烟测试。

## 当前验证事实

- 完整 ROS 测试：72 tests，0 errors，0 failures，0 skipped。
- 双 Panda：两个 `[7]` 关节 observation、14 维 action、10 个物理步，
  `hardware_connected=false`。
- 仿真结果：
  `experiments/robosuite_dual_panda/smoke-result.json`
- 结果 SHA256：
  `6a73312d2b654eeae872176659c684d40ab617d552bae10cd07acb2b6b826600`
- 现场只读探针最近一次为 expected=31、ready=31、lost=0、armed=true、
  protection=NORMAL、self_collide=0，同时仍报告
  `hardware_execution_locked=true`。
- V2 热启动后代表性 GraspNet 总耗时约 0.284–0.578 s；裁剪/体素化把约
  43 万点的场景降到约 6.5 千点后再做碰撞检测。

当前杂乱且会变化的现场画面中，V2 曾到达碰撞过滤和目标重验证阶段，但没有得到
一次满足全部新鲜度、碰撞、方向、宽度、重验证和 MoveIt 条件的完整成功 dry-run：
目标有消失/移动，部分批次无候选，部分候选被安全过滤。这是诚实的当前状态，
不能用 2026-07-28 的旧成功记录替代 V2 验收，也不应通过放宽阈值制造成功。

## 操作

构建和测试：

```bash
cd "/home/ubuntu/zeng-Visual Grasping"
bash tools/build_project.sh
third_party/venv/bin/python tools/check_dual_arm_profile.py
```

无运动检查：

```bash
tools/graspctl.sh start
tools/graspctl.sh status
tools/graspctl.sh list
tools/graspctl.sh plan bottle auto
tools/graspctl.sh logs 200
tools/graspctl.sh stop
```

注意：

- 修改 `config/*.yaml` 后必须重新构建再启动。
- `plan` 不发布运动命令，失败时查看日志中的 raw candidate 和各过滤计数。
- `stop` 只停止本项目 ROS 栈，不是机器人急停。
- 不要运行 `execute`，也不要更改执行锁。

## 真机前仍必须完成

1. 把桌面、货架、地面、机器人周边固定物注入 MoveIt PlanningScene。
2. 用标定板完成相机外参实测并记录误差。
3. 实测左右 TCP 位于两指夹持中心，完成夹爪反馈端点映射。
4. 复核 MoveIt 关节限制、碰撞矩阵和 pregrasp/grasp/lift 连续起点。
5. 现场验证实体急停：按下、停止、复位、重新使能，全程有人守急停。
6. 在隔离工作区用轻质软物体、最低速度完成空载和单臂验收。
7. 首次使用先从公开模板创建本地验收文件，再逐项填写并签署；名义 CAD/URDF
   值不得标记为实测：

```bash
cp config/site_acceptance.example.yaml config/site_acceptance.yaml
```
8. 由现场负责人单独审阅执行配置和精确确认令牌后，才可安排真机试车。

完成以上事项前，保持：

```yaml
dry_run: true
allow_hardware_execution: false
enable_pbvs_hardware_follow: false
```

## 已知限制

- 当前是一臂抓取、另一臂保持；`Both_Arms` 组和双 Panda 仿真不等于实机原子
  双臂协同。
- 厂商没有标准 `FollowJointTrajectory`，18 值适配器不能逐点保留 MoveIt
  速度/加速度。
- 没有公开且经验证的软件 stop；已发轨迹的立即停止只能依赖实体急停。
- 环境 PlanningScene 尚未完成，因此不能进行实机运动。
- 默认 COCO 分割权重不能识别任意未知类别。
- GraspNet baseline/权重受上游许可约束。
- OpenClaw 只负责自然语言任务入口，不进入实时控制环，也不能直接生成关节命令。

## 历史记录说明

`docs/LIVE_VALIDATION_ZH.md` 中的 2026-07-28 厂商 RRT/PBVS 和旧 dry-run 结果仅供
追溯。V2 已改变接口、候选过滤、新鲜度语义和唯一执行后端；旧结果不是当前验收，
旧解锁步骤不得使用。
