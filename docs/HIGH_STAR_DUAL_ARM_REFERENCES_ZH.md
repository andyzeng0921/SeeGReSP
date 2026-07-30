# 高 Star 视觉抓取与双臂 7 轴参考

核对日期：2026-07-30。Star 为当日约数，会随时间变化。以下只引用各项目
官方仓库；本项目没有把这些大型仓库复制进实机控制链。

## 推荐参考栈

| 项目 | 约 Star / 许可证 | 本项目借鉴点 | 采用方式 |
|---|---:|---|---|
| [LeRobot](https://github.com/huggingface/lerobot) | 26k / Apache-2.0 | observation/action、数据、训练、硬件适配分层 | 后续离线数据层；不直接输出电机命令 |
| [MuJoCo](https://github.com/google-deepmind/mujoco) | 14k / Apache-2.0 | 稳定动力学与碰撞仿真 | 推荐作为双 Panda 离线实验底座 |
| [robosuite](https://github.com/ARISE-Initiative/robosuite) | 2.5k / MIT | `TwoArm*` 环境、双 Panda 2×7 轴、任务/本体解耦 | 首选双臂参考实现 |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | 3.8k / 模型分别授权 | Panda、FR3、Sawyer、iiwa、Gen3、xArm7、ALOHA 模型 | 仅按单模型许可证引入 |
| [MoveIt 2](https://github.com/moveit/moveit2) | 1.9k / BSD-3-Clause | URDF/SRDF、联合规划、碰撞、时间参数化 | 已作为规划层；306 用厂商轨迹执行 |
| [cuRobo](https://github.com/NVlabs/curobo) | 1.7k / Apache-2.0 | GPU IK、碰撞与高自由度轨迹优化 | 可替换实验后端，不作为当前依赖 |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | 3.6k / BSD-2-Clause | 独立 FK/Jacobian/动力学校验 | 后续交叉校验工具 |
| [robomimic](https://github.com/ARISE-Initiative/robomimic) | 1.5k / MIT | 模仿学习配置、数据与算法注册分层 | 后续离线学习层 |

双臂策略参考包括
[RoboTwin](https://github.com/RoboTwin-Platform/RoboTwin)、
[RDT-1B](https://github.com/thu-ml/RoboticsDiffusionTransformer)、
[ALOHA](https://github.com/tonyzhaozh/aloha)、
[ACT](https://github.com/tonyzhaozh/act) 和
[Diffusion Policy](https://github.com/real-stanford/diffusion_policy)。
它们适合借鉴同步数据、action chunk、时域窗口和任务成功判定；策略输出必须经过
本项目的规划器与安全门，不能直接发布到厂商控制话题。

视觉抓取方面，项目继续使用
[GraspNet baseline](https://github.com/graspnet/graspnet-baseline) 的薄适配层。
还可对照 [GPD](https://github.com/atenpas/gpd)、
[GG-CNN](https://github.com/dougsm/ggcnn) 和
[Visual Pushing and Grasping](https://github.com/andyzeng/visual-pushing-grasping)
的候选表示、工作区裁剪与失败重试。

## 为什么先试双 Panda，而不是直接换实机算法

robosuite 的双 Panda 正好是 2×7 轴，可以先验证：

1. 左右臂 14 关节顺序与 observation/action schema；
2. 两臂、腕部、夹爪和场景的碰撞；
3. 同一时间轴上的同步动作；
4. `TwoArmLift`、`TwoArmPegInHole` 等任务成功条件；
5. 策略失败不会碰到 306 实机。

本项目当前新增 `Both_Arms` 14-DoF SRDF 组用于联合规划/碰撞配置检查，但没有
宣称厂商驱动已经支持原子双臂同步执行。稳定默认路径仍是“选择一臂抓取，另一臂
保持当前状态”。

## 已运行的双七轴仿真冒烟测试

不是只写参考清单：已在目标机项目目录内实际创建并运行隔离环境：

```text
experiments/robosuite_dual_panda/
  requirements.txt       robosuite==1.5.2, mujoco==3.3.7, numpy==1.26.4
  smoke_dual_panda.py    无界面、无 ROS、无 AutoLife SDK
  smoke-result.json      可复核的运行结果
```

结果为 `TwoArmLift / parallel / Panda × 2` 成功 reset 并推进 10 个物理步；两路
`robot*_joint_pos` 的 shape 均为 `[7]`，联合 action dimension 为 14，reward 与
关节值均为 finite。`hardware_connected=false`。第一次尝试还发现 MuJoCo
3.11.0 已改变 robosuite 使用的接口，因此固定到已验证的 3.3.7；实验依赖只安装在
该目录 `.deps`，没有污染 ROS 或项目运行 venv。

`smoke-result.json` 的 SHA256 为：

```text
6a73312d2b654eeae872176659c684d40ab617d552bae10cd07acb2b6b826600
```

该结果只证明“双 7 轴仿真骨架可运行”，不代表 AutoLife S2 已支持同步双臂实机
执行，也不证明本项目的相机、TCP、夹爪或场景碰撞已经验收。

## 许可证边界

- GraspNet baseline 为学术、非营利、非商业许可；权重也应单独核对，不能默认
  进入商业交付核心。
- AnyGrasp SDK 是机器绑定二进制 SDK，不是宽松开源替代。
- Contact-GraspNet 使用 NVIDIA 自定义许可证，需单独审核。
- MuJoCo Menagerie 的每个模型有自己的许可证。
- 第三方依赖应固定 commit，并保存 LICENSE/NOTICE；不得把代码混入业务模块。

## 本项目的落地结论

从这些参考实现中采用的是“接口与职责分层”，而不是整仓复制：

```text
相机/点云
  -> 原子 observation + 时间一致的实例感知
  -> GraspCandidate[]
  -> 同一 track 新帧重验证
  -> 单臂选择 / 双臂任务层
  -> MoveIt 规划与碰撞
  -> 安全授权和连续保护状态
  -> AutoLife S2 厂商适配器
```

所有后续实验、模型、缓存和运行记录仍必须位于：

```text
/home/ubuntu/zeng-Visual Grasping/
```
