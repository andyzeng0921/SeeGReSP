# Robot 306 视觉抓取 V2 交接

更新时间：2026-07-30

## 2026-08-03 抓取任务范围与最新现场帧纠正

当前项目只做双臂视觉抓取，不控制底盘。Nav2、地图、AMCL、`cmd_vel`、里程计和激光雷达均不属于当前手臂抓取的解锁条件；`navigate_before_grasp` 必须保持 `false`。此前鱼眼会议室画面来自导航脚本固定读取的 `camera_image_buffer_head_left_jpeg`，不是抓取链路的最新现场帧，不得用作抓取判断。

最新抓取样本已从 AutoLife RGB-D 共享内存 `rgbd_head_color` 与 `rgbd_head_depth` 同帧截取：

```text
runtime/scene_samples/1785737622442864128-bottle-1/
```

- 图像为 1280×720 非鱼眼头部 RGB-D 画面。
- 瓶子：`track_id=1`，置信度 `0.8059`，深度 `0.737 m`。
- 瓶子在 `Link_Zero_Point` 下约为 `[0.5841, 0.1485, 0.8824] m`，几何最近机械臂为左臂。
- 左右腕和 TCP 仍在相机画面外，因此 `visibility_reposition_needed=true`。
- 本次仅采集和理解内存帧，没有执行复位或发送任何运动命令；抓取栈随后已停止。

限制必须分层，不能全部做成全局常量：

1. **所有目标共同且不可取消的手臂安全门禁**：相关机械臂厂商心跳与保护状态、实体急停监督、经过验收的复位姿态、机器人自碰撞、桌面/固定物碰撞模型、关节限位、连续轨迹、反馈新鲜度、执行互斥及精确确认令牌。
2. **按目标类别配置的抓取约束**：允许抓取宽度、最低候选分数、目标深度带、接近方向与倾角、预抓取距离、目标移动阈值、提升距离、首选机械臂及目标工作空间。瓶子应使用独立 profile；盒子、杯子和其他目标不得直接复用瓶子参数。
3. **按单次观测动态计算的约束**：目标三维位置、实例 mask、点云尺度、可达臂、桌面净空、碰撞检查和 IK/OMPL 结果。模型输出只能收紧约束，不能取消第一层安全门禁。

当前代码仍以全局 `graspnet.yaml`、`coordinator.yaml` 和 `motion.yaml` 参数为主，目标 profile 尚未贯通 GraspNet、协调器和 MotionExecutor。下一步应新增统一、可审计的 `target_profiles.yaml`，并使用 `GraspCandidate.label` 在三个边界重复校验；在此完成并测试前，仅将现有参数视为瓶子试验参数，不用于其他类别的实机动作。

## 2026-08-03 实机前检更新（导航与 VL 集成）

### 新增能力

1. **视觉导航管线（navctl.sh）** — 独立于抓取栈的只读感知与导航干跑脚本，支持：
   - `navctl.sh status`：检查 ROS 2 / cmd_vel / 头部相机
   - `navctl.sh camera`：从共享内存 `/dev/shm/camera_image_buffer_head_left_jpeg` 抓取头部左相机帧
   - `navctl.sh see`：发送相机帧给 `qwen3-vl-8b-instruct`（GPUStack API）获取场景理解
   - `navctl.sh decide`：让 VL 模型决策移动方向（forward/left/right/backward/stop）
   - `navctl.sh move`/`navctl.sh go`：演练底盘速度指令 / Nav2 地图导航目标
   - `navctl.sh execute`：始终拒绝硬件运动
   - `navctl.sh stop`：提示使用实体急停，不发布未经验证的软件停止命令

2. **OpenClaw visual-navigation skill** — `agent_workspace/skills/visual-navigation/SKILL.md`，OpenClaw 可通过自然语言调用相机、VL 分析和导航干跑；不能控制底盘

3. **Qwen-RobotNav 导航模型适配器（navigation_core.py）** — 完整的 Qwen-RobotNav 接口定义：
   - 5 种任务模式（VLN / PointNav / ObjNav / Tracking / Driving）
   - 可控观测协议（token_budget、temporal_decay、camera_weights、frame_sample_mode）
   - 8 路点输出 + DONE 解析
   - RoboStral Navigate 适配器保留为参考

4. **GPUStackBackend** — `navigation_core.py` 新增类，直接接入 GPUStack API：
   - `vl_chat(image, prompt)` → 发送图像给 `qwen3-vl-8b-instruct`
   - `llm_chat(system, user)` → 发送文本给 `qwen3.5-35b-a3b`
   - 带重试（指数退避）、超时、优雅降级到 placeholder
   - 配置 `api_url` / `api_key` 后自动启用，未配置或不可达时安全回退到 placeholder

5. **GPUStack VL 推理实测** — 已完成真实相机到 VL 的端到端验证，但输出不能直接作为运动命令：
   - 头部左相机（1920×1080 JPEG）→ base64 → `qwen3-vl-8b-instruct`
   - 最新帧为会议室鱼眼画面，近处可见桌腿、椅子、线缆和机器人自身结构
   - 模型输出 `DIRECTION: forward`，却错误声称前方没有障碍；因此必须增加度量深度/激光避障和 Nav2 局部代价地图，禁止把单帧 VL 方向直接发布到底盘

### 已解决的已知限制

- OpenClaw 仍只负责自然语言任务入口，不进入实时控制环，也不能直接生成关节或底盘命令。新增的 `visual-navigation` skill + `navctl.sh` 只提供真实相机/VL 与 dry-run；源码中的 `execute` 始终拒绝硬件执行。

- ~~navigation_core.py 纯 placeholder 推理。~~
  → 已注入 `GPUStackBackend`，配置 `api_url` 后自动切换到真实 VL+LLM 推理。未配置时安全回退到 placeholder，不影响测试。

### 当前约束（不变）

- 厂商保护心跳：`expected=31, ready=20, lost=0, armed=false` — 厂商保护进程未就绪，禁止 resets/抓取/夹爪/底盘动作
- 桌面碰撞配置 `config/environment_scene.yaml` 尚未现场验收，`verified: false`
- 可视复位配置 `config/visible_reset_pose.yaml` 尚未现场示教，`verified: false`
- 执行配置保持 `dry_run: true`、`allow_hardware_execution: false`
- `navctl.sh execute` 和 `graspctl.sh execute` 均不得绕过心跳、环境场景或现场验收

### 测试状态

- 完整 ROS 测试：139 tests, 0 errors, 0 failures, 0 skipped
- 导航核心测试：56 tests (pytest)，覆盖 GPUStackBackend 创建/验证/降级/编码、QwenRobotNavAdapter 双后端和仓库凭据检查

### 2026-08-03 导航实机只读复核

- `navctl.sh status`：`ROS2_OK`、头部左相机正常，但 `CMD_VEL_UNAVAILABLE`。
- ROS 图：`/cmd_vel`、`/manual_cmd_vel`、`/odom`、`/scan` 不存在；`/map` 与 `/amcl_pose` 的发布者均为 0；没有发现 Nav2、AMCL、map server、controller server、planner server 或 BT navigator 节点。
- 因此不存在可取消后立即安全运动的“测试限制”；缺失的是实机定位、里程计、障碍传感器、控制器和保护链路。
- GPUStack 凭据已从 `config/navigation.yaml` 移除，节点现在通过 `GPUSTACK_API_URL` 与 `GPUSTACK_API_KEY` 环境变量获取，仓库扫描未发现 `gpustack_...` 密钥模式。已经暴露过的密钥仍建议轮换。
- 本次没有发布 `cmd_vel`，没有发送关节、夹爪或底盘命令；抓取栈检查后已停止。

## 2026-08-03 实机前检更新

用户要求每次实机运行前先复位双臂。本轮按"状态检查 → 复位仅规划"执行前检，门禁未通过，所以没有执行复位、抓取、夹爪或底盘动作，也没有发送任何关节命令；检查后已停止项目 ROS 栈。

- 厂商保护心跳：`expected=31, ready=20, lost=0, armed=false`，缺少左臂、右臂和颈部就绪位。必须从厂商保护进程或设备侧恢复，禁止伪造 ready 位。
- 桌面碰撞配置 `config/environment_scene.yaml` 尚未现场验收，`verified: false`。候选文件是 `runtime/calibration/environment_scene.candidate.yaml`，不能直接当成已验收配置。
- 可视复位配置 `config/visible_reset_pose.yaml` 尚未现场示教和验收，`verified: false`；当前左右腕/TCP不在头部相机图像内。
- 执行配置保持 `dry_run: true`、`allow_hardware_execution: false`、`require_environment_scene: true`。
- 当前只有安全的 `tools/graspctl.sh reset-plan`，尚无真实 `reset-execute`；不得用抓取执行入口代替复位。
- 修复了 colcon `--symlink-install` 下合法 `config` 符号链接被误判为路径逃逸的问题。修复后复位规划正确停在“环境场景尚未现场验收”的门禁。
- 服务器重新构建成功：`139 tests, 0 errors, 0 failures, 0 skipped`。

接手顺序必须是：恢复心跳到 `31/31 + armed` → 现场测量并在 RViz 核验桌面碰撞盒 → 厂家示教模式采集并验收双臂可视复位姿态 → `reset-plan` 成功 → 实现独立低速且受确认令牌保护的 `reset-execute` → 每轮强制“复位并视觉验证”后才允许重新感知和抓取规划。首次运动必须隔离工作区并由现场人员持续守住实体急停。

上述条件完成前，不得修改执行锁，不得运行 `graspctl execute`，不得绕过心跳、环境场景或现场验收。

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
- ~~OpenClaw 只负责自然语言任务入口，不进入实时控制环。~~
  → **已解决**：新增 `visual-navigation` skill + `navctl.sh`，OpenClaw 可通过受确认令牌保护的 `navctl.sh` 发送 cmd_vel 和 Nav2 目标（离散动作，非实时闭环）。

## 历史记录说明

`docs/LIVE_VALIDATION_ZH.md` 中的 2026-07-28 厂商 RRT/PBVS 和旧 dry-run 结果仅供
追溯。V2 已改变接口、候选过滤、新鲜度语义和唯一执行后端；旧结果不是当前验收，
旧解锁步骤不得使用。

## 2026-08-03 头部自视与 RGB-D 桌面标定

- 实机运行前保护状态为 `NORMAL`、心跳 `31/31`、`armed=true`；双臂保持复位姿态。
- 头部通过厂家复位/使能和受锁定的渐进控制桥低速移动到俯仰 `-31.99°`，到位后立即
  锁定。最新头部 RGB-D 帧中左右夹爪均完整可见，瓶子 track 为 `806`，深度
  `0.837 m`，基座坐标 `[0.64386, 0.14518, 0.88200] m`。
- 右 TCP 的模型/深度误差为 `22.8 mm`，状态 `depth_consistent`；左 TCP 投影在图内，
  但投影像素缺少有效深度，状态仍为 `projected_only_not_depth_confirmed`。
- `calibrate_environment.py` 已支持多样本融合；固定头部采集 5 帧后，对桌面高度与四条
  边界分别取中位数。融合结果：顶面 `z=0.77097 m`，尺寸
  `0.58518 × 0.77775 × 0.06 m`，中心 `[0.87001, 0.21870, 0.74097] m`，
  高度极差 `14.8 mm`，总内点 `125735`。
- 远端候选：`runtime/calibration/environment_scene.head_minus32.fused.candidate.yaml`；
  本地交接副本：`handoff/calibration/environment_scene.head_minus32.fused.candidate.yaml`。
  该文件保持 `verified: false`，尚未替换正式 PlanningScene 配置。
- 新增融合回归测试后完整结果：`141 tests, 0 errors, 0 failures, 0 skipped`。

### RViz/MoveIt 桌面碰撞盒复核

- 使用同一融合候选临时注入 MoveIt PlanningScene；当前状态碰撞检查通过，流程随后按
  预期停在未验收的可视复位姿态，没有规划或发送运动。
- RViz 中以橙色半透明盒叠加桌面，现场 RGB 画面、当前机器人模型和桌面盒同时显示；
  双臂、双夹爪及机器人本体未与桌面盒穿插。
- 验收截图：本地 `handoff/rviz-environment-overlap-check-pass.png`。
- 检查后正式 `config/environment_scene.yaml` 已按 SHA256 恢复原文件，仍为
  `verified: false`；临时 Marker、RViz 和抓取栈均已关闭。
- 修复 Jazzy RViz 的 MarkerArray 配置字段：使用 `Topic`，不再使用未生效的
  `Marker Topic`。

## 2026-08-03 瓶子抓取尝试与 Agent skill

- 新增 `agent_workspace/skills/rgbd-tabletop-simulation/`，用于精确 RGB-D 采样、
  五帧桌面融合、PlanningScene 临时注入、MoveIt 当前状态碰撞检查、RViz 叠加和
  配置恢复。`quick_validate.py` 验证通过。
- 实机前检查：保护 `NORMAL`、心跳 `31/31`、`armed=true`；双臂反馈与复位目标的
  最大偏差约 `0.022°`，头部俯仰 `-32.01°`。
- 新鲜瓶子样本：track `1`，置信度 `0.625`，深度 `0.835 m`，基座坐标
  `[0.64394, 0.14639, 0.88448] m`，几何最近为左臂。
- GraspNet 左臂干运行失败：4 个原始候选的分数和宽度均通过，但 4 个均被
  `horizontal_filter` 拒绝，最终候选为 0；未进入 MoveIt，未发送任何运动。
- 实机执行仍被正式门禁阻止：`dry_run: true`、`allow_hardware_execution: false`，
  camera extrinsic、左右 TCP、gripper mapping、实体急停测试和 workspace 签字均未
  验收。不得由 Agent 伪造这些记录或放宽抓取过滤器。

## 2026-08-03 瓶子侧抓方向等价变换与 MoveIt 复测

- `graspnet_core.py` 新增绕局部接近轴滚转和水平侧抓候选生成。该变换严格保持
  GraspNet 抓取点、接近轴和预抓取直线，不投影、不反转接近方向，也不放宽原有
  `30°` 接近/闭合倾角阈值。
- `graspnet_server.py` 只对配置中的 `bottle` 类启用 `+90°/-90°/180°` 分支：
  `±90°` 把圆柱瓶不合理的竖直闭合轴转换为侧向闭合，`180°` 保留平行夹爪方向
  等价分支以尝试不同腕部 IK；其他类别维持原过滤行为。
- 新增回归用例验证：侧抓滚转保持接近轴、可恢复竖直闭合轴、不会修复本身过陡的
  接近轴、保留平行夹爪 `180°` 等价方向。远端正式测试为 `139 pytest`，ROS 安装后
  `145 colcon tests`，均为零失败。
- 复测前硬件只读探针：保护 `NORMAL`、心跳 `31/31`、`armed=true`；同时报告
  `hardware_execution_locked=true`。使用已经 RViz 重叠检查过的五帧桌面候选临时
  注入 PlanningScene，仅调用 `plan bottle left`，action 明确为 `execute=false`。
- 本轮方向过滤问题已解除：GraspNet 侧抓候选进入 MoveIt，协调器检查了两个
  `score=0.494` 的左臂方向等价分支。两者都在预抓取 KDL IK 阶段无解：
  `[0.4654,-0.1204,0.8577] m`（四元数
  `[0.5458,0.1386,0.3655,0.7412]`）及
  `[0.4289,-0.0775,0.8352] m`（四元数
  `[-0.7412,-0.3655,0.1386,0.5458]`）。因此没有形成可执行路径，更没有发送运动。
- 干运行退出后抓取栈已停止；正式 `config/environment_scene.yaml` 已恢复为
  `verified: false`，恢复前后 SHA256 均为
  `0d0e5aa524233e79b2f14b51ffd4de4fddb05b18818a3bcdb023b1cad47a5450`。
  下一步应解决左臂 TCP/IK 可达域或生成更靠近左臂自然腕姿的侧抓接近方向，不能通过
  取消桌面碰撞、放宽倾角或绕过现场验收来制造成功。

## 2026-08-03 左臂自然腕姿/TCP 可达域侧抓

- 新增 `wrist_aligned_side_grasp_rows`：对瓶子使用当前左 TCP 到精确 RGB-D 瓶子
  三维中心的向量作为接近轴，以基座重力方向叉乘构造水平闭合轴，并在 `180°`
  平行夹爪等价方向中选择与当前 TCP 旋转距离较小的分支。分数、宽度等仍来自
  GraspNet；新姿态在成为候选前重新进入 GraspNet 官方局部场景点云碰撞过滤。
- 不再把网络抖动较大的候选平移直接作为瓶子自然侧抓中心；使用 action 所携带的同帧
  RGB-D 目标中心。现场最新代表性目标为
  `[0.65955, 0.14802, 0.88820] m`，左 TCP 到目标距离 `0.33515 m`。
- 第一轮自然腕姿复测中，两个 `score=0.454` 候选已越过 KDL IK，原来的
  `MoveIt KDL IK found no Left_Arm solution for pregrasp` 不再出现；随后暴露 MoveIt
  原始轨迹重复时间戳。已给 OMPL 管线加入官方
  `AddTimeOptimalParameterization` 与 `ValidateSolution` response adapters，禁止未定时
  轨迹进入厂商适配器。
- 使用精确瓶子中心后的最终复测生成 3 个经点云碰撞过滤的自然腕姿行、6 个最终候选，
  分数范围 `[0.174, 0.434]`。通过协调器阈值的 4 个候选均未再报告预抓取 KDL IK
  无解，但 `AddTimeOptimalParameterization` 无法为当前机器人模型生成时间轨迹，
  MoveIt 因而返回 `MoveIt planning failed for pregrasp`。这是新的剩余阻塞，完整
  pregrasp/grasp/lift 干运行尚未成功，绝不能执行实机。
- 当前完整测试：`144 pytest`，安装态 `150 colcon tests`，零失败。所有复测均为
  `execute=false`；保护状态 `NORMAL`、心跳 `31/31`、`armed=true`，同时硬件锁仍为
  true。抓取栈已停止，正式桌面配置恢复为 `verified: false`，SHA256 仍为
  `0d0e5aa524233e79b2f14b51ffd4de4fddb05b18818a3bcdb023b1cad47a5450`。
- 下一步应补全/核验 AutoLife S2 七轴关节的速度与加速度限制，再单独验证 TOTG 或
  Ruckig 时间参数化；不得删除严格时间戳校验或人为填充未经动力学约束的执行时间。
