# Robot 306 视觉抓取架构 V2

日期：2026-07-30

## 能力边界

- AutoLife S2：左臂 7 DoF + 右臂 7 DoF。
- 厂商轨迹：腰腿 4 + 左臂 7 + 右臂 7，共 18 个值。
- 当前任务：选择一只主动臂抓取，另一只臂保持；不是双臂协同抓取。
- `Both_Arms` 14-DoF 组用于联合模型与碰撞研究，不代表驱动支持原子双臂执行。
- MoveIt 是唯一可接真实执行的规划后端；厂商没有
  `FollowJointTrajectory`，由受控适配器生成 18 值轨迹。

## 数据流

```text
OpenClaw / 未来策略（可选任务层）
                    |
                    v
PickObject Action / GraspCoordinator
                    |
                    v
YOLO11-seg + track id + 同帧 RGB-D
                    |
                    v
项目内原子 observation store
token + size + SHA256（DDS 只传元数据）
                    |
                    v
GraspNet + 目标邻域点云 + 官方碰撞检测
                    |
                    v
同 track 新帧目标重验证
                    |
                    v
dual_arm_core / target_validation_core（纯逻辑）
                    |
                    v
MoveIt 单臂规划 + Both_Arms 机器人模型
                    |
                    v
MotionExecutor（唯一硬件边界）
互斥 / 确认 / 现场验收 / protection / unknown-motion
                    |
                    v
AutoLife 18 关节轨迹与夹爪接口（交付状态锁定）
```

任务层、OpenClaw、VLA 或学习策略都不能直接发布厂商运动话题。

## 原子观测

独立的 best-effort RGB/depth/info 缓存会组合不同帧；把约 5 MB RGB-D 直接嵌入
ROS 服务又造成明显延迟。V2 改为：

1. YOLO 在选择目标时原子保存精确 RGB、depth、mask、intrinsics、track、
   stamp 和 frame。
2. 文件仅位于项目内
   `runtime/selected_observations`，目录权限 0700、文件 0600、最多保留 32 个。
3. `SelectObject` 返回 token、payload size 和 SHA256。
4. GraspNet 拒绝目录逃逸、非法 token、非普通文件、长度/摘要不符、元数据不符、
   shape/dtype 不符；成功读取后删除文件。

路径根由 `ADAPTIVE_GRASP_PACKAGE_ROOT` 明确传入，不依赖当前工作目录。

## 分段新鲜度

- YOLO 结果最大年龄：0.5 s。
- 选择观察最大年龄：0.35 s；协调器可在 2 s 内每 50 ms 重试同一 track。
- observation 解码预算：0.5 s。
- 推理完成时源帧最大年龄：1.5 s。
- 每个新 observation 只推理一次，不对同一过期帧重复三次。
- 推理后重新选择同一 track，要求更新的时间戳、同 label/frame、有效三维点，
  三维位移不超过 0.015 m、bbox 中心不超过 12 px、IoU 不低于 0.60。
- 只有重验证通过，候选及其 Pose 时间戳才更新到新帧并进入规划。
- MotionExecutor 接受的候选最大年龄为 1.0 s。更新时间戳不表示对新像素重新
  推理，只表示在上述明确阈值内确认目标几何没有发生实质变化。

## 碰撞和性能

GraspNet 启动时预加载网络并做 GPU warm-up。碰撞检测前，完整场景点云先裁剪到
目标包围区域外扩 0.20 m，再以 1 cm 确定性体素化，最后进入上游官方碰撞检测器。
代表性热启动总耗时约 0.284–0.578 s，避免原先约 48 万点场景带来的秒级碰撞
开销。NMS/排序后最多 100 个候选进入 `collision_threshold=0.01` 的碰撞过滤；
接近扫掠距离为 0.12 m，`apply_grasp_depth=false`。分数、夹爪宽度、方向和碰撞
的拒绝计数会写入响应/日志。

该过滤仍是基于观测点云的 model-free 几何检查，不是完整 PlanningScene，不能
覆盖未观测桌面、地面、人体或整机连杆。

## 强制安全不变量

真实命令只有同时满足以下条件才可能发出：

```text
dry_run=false
AND allow_hardware_execution=true
AND backend=moveit_py_vendor_execution
AND enable_pbvs_hardware_follow=false
AND 精确操作员确认令牌
AND site_acceptance 无未完成项
AND hardware probe 新鲜且 ready=true
AND heartbeat expected=ready=31, lost=0, armed=true
AND protection=NORMAL, self_collide=0
AND MotionExecutor 获得唯一执行锁
```

`execute=true && dry_run=true` 明确失败，不会降级成伪执行。硬件请求一旦发布，
后续异常或超时会锁存 unknown-motion fault，本进程没有清除路径。由于没有验证的
软件 stop，发出请求后不接受“取消即已停止”的错误语义；立即停止必须使用实体急停。

候选还要通过 finite、时间戳、工作区、夹爪宽度、双 EEF 完整性和发布后新反馈
检查。执行器不信任协调器已做过这些验证，而是在唯一出口再次校验。

## 已移除或收口的不合理设计

1. 不再由 CLI 单独承担现场验收；执行节点再次强制检查。
2. 直接 Action/Service 也必须精确确认，不能绕开脚本。
3. 不再写厂商私有 `/dev/shm/pid_loop_state_*`。
4. 不再把腕部安装链接默认当作夹持 TCP。
5. 不再用独立三路缓存拼接目标观察。
6. 不再把大 RGB-D payload 在多个 DDS 服务间复制。
7. 不再对同一过期观察重复 GraspNet 推理。
8. PBVS 直接硬件跟随已移除；启用其开关反而成为解锁 blocker。
9. `vendor_rrt`、`vendor_task_space` 只保留 dry-run；真机只允许 MoveIt 轨迹适配。
10. 探针 ready 始终报告硬件仍锁定，避免把在线状态误当现场验收。
11. 协调器区分 unavailable/timeout/error，并在任何已发硬件请求的不确定结果后
    fail closed。

## 仍未解决

- 尚未把桌面、货架、地面和旁人等环境障碍注入 PlanningScene。
- pregrasp/grasp/lift 的连续起点还需进一步联调。
- 厂商接口只有总 duration，不能逐点保留 MoveIt 速度/加速度。
- 相机外参、双 TCP、夹爪端点和实体急停仍待现场实测签名。
- 当前变化的现场画面下，V2 尚未产生满足全部硬化条件的完整 MoveIt dry-run；
  已到达碰撞过滤和目标重验证阶段，安全拒绝按设计生效。
- 真正双臂协同仍需要双末端原子目标、联合时间参数化和实机碰撞验收。

## 无运动验证

```bash
cd "/home/ubuntu/zeng-Visual Grasping"
bash tools/build_project.sh
third_party/venv/bin/python tools/check_dual_arm_profile.py
tools/graspctl.sh start
tools/graspctl.sh status
tools/graspctl.sh list
tools/graspctl.sh plan bottle auto
tools/graspctl.sh stop
```

`stop` 仅停止本项目 ROS 栈，不是机器人停止命令。
