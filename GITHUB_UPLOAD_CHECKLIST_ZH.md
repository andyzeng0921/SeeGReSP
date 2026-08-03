# GitHub 发布检查清单

此目录是 2026-07-30 从 Robot 306 的以下目录导出的源码副本：

```text
/home/ubuntu/zeng-Visual Grasping
```

## 已排除

以下内容体积大、可重建或包含运行时数据，没有进入本地 GitHub 副本：

```text
build/
install/
log/
test-log/
runtime/
models/
third_party/
.pytest_cache/
experiments/robosuite_dual_panda/.deps/
experiments/robosuite_dual_panda/.venv/
```

保留了 robosuite 的 `requirements.txt`、冒烟脚本和结果 JSON，可以重新安装并复现。
GraspNet、YOLO 权重、OpenClaw、Python/CUDA 环境均由项目安装脚本重新获取。

## 公开仓库前必须人工确认

1. 为本项目选择许可证。当前没有项目级 `LICENSE`，不要默认把第三方许可证当作
   本项目许可证。
2. 确认 AutoLife S2 的 URDF 和 `meshes/` 允许公开再发布；未经供应商许可时应
   从公开版本移除，并提供由用户自行放置资源的说明。
3. 检查 `docs/HARDWARE_DISCOVERY.md`、`HANDOFF.md`、配置和 Git 历史中是否包含
   不希望公开的机器人编号、相机序列号、内部路径或现场信息。
4. 当前导出扫描没有发现 API key、订阅地址或私钥。服务器被排除的
   `runtime/openclaw-state` 中存在 OpenClaw 设备私钥，因此绝不能把 `runtime/`
   强制加入或另行整目录上传。
5. `config/site_acceptance.example.yaml` 与
   `agent_workspace/openclaw.example.json` 仅为公开模板。真实验收记录和实际
   OpenClaw 配置已加入 `.gitignore`。
6. 不要提交模型权重、虚拟环境、日志、相机画面、ROS bag 或现场验收签名文件。

GraspNet baseline 及权重受其上游学术/非商业条款约束，没有包含在此导出中。
MuJoCo Menagerie 模型也应逐个核对许可证。

首次在目标机使用时：

```bash
cp config/site_acceptance.example.yaml config/site_acceptance.yaml
cp agent_workspace/openclaw.example.json agent_workspace/openclaw.json
```

## 建议提交步骤

本地交付时所有当前变更已经暂存，但尚未创建 commit。先审阅；如果之后又修改了
文件，再执行一次 `git add --all`。

```bash
git status --short
git add --all
git diff --cached --check
git diff --cached
git commit -m "feat: harden dual-arm visual grasping architecture"
git branch -M main
git remote add origin <YOUR_GITHUB_REPOSITORY_URL>
git push -u origin main
```

如果使用 GitHub Desktop，选择 **Add an Existing Repository from your hard drive**，
目录指向当前 `zeng-Visual-Grasping` 文件夹，然后审阅变更、提交并 Publish。
