# robosuite 双 Panda 冒烟测试

这个实验只验证一个隔离的、无界面的仿真基线：

- robosuite `TwoArmLift`
- `parallel` 双机器人布局
- 两台 Panda，每台机械臂 7 DoF
- 不订阅或发布本项目 ROS 话题
- 不加载 AutoLife SDK，不连接真实机器人

版本固定在 `robosuite==1.5.2`。目标机没有 `python3-venv` / `ensurepip`，
因此用项目已有 Python 的 pip 把依赖安装到本目录 `.deps`，并用一个不带 pip 的
`.venv` 解释器启动；不会修改系统 Python、ROS 2 或本项目运行依赖。

运行：

```bash
cd "/home/ubuntu/zeng-Visual Grasping/experiments/robosuite_dual_panda"
python3 -m venv --without-pip .venv
../../third_party/venv/bin/python -m pip install \
  --target .deps --requirement requirements.txt
PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/.deps" MUJOCO_GL=egl \
  .venv/bin/python smoke_dual_panda.py
```

成功时会生成 `smoke-result.json`。这个冒烟测试只证明仿真环境可创建、两路 7
轴状态可观测且物理引擎能推进；它不证明本项目已经具备真实双臂同步执行能力。
