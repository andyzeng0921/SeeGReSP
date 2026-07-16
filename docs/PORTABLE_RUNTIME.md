# Portable runtime layout

Run `tools/install_ai_environment.sh` from the package source directory. The
installer keeps every optional AI runtime asset inside this package:

```text
adaptive_object_grasping/
  third_party/venv/
  third_party/virtualenv.pyz
  third_party/graspnet-baseline/
  models/yolo/yolo11n-seg.pt
  models/graspnet/checkpoint-rs.tar
```

The launch wrappers locate the package root automatically, so the package can be
copied to another ROS 2 workspace without retaining the original username or
absolute workspace path. Rebuild the workspace after moving it.

`third_party/` and `models/` are intentionally ignored by Git because they can
contain several gigabytes of machine-specific binaries and licensed weights.
Use an archive, external drive, Git LFS, or an artifact store when the complete
runtime must be transferred. A Python virtual environment is not guaranteed to
work across different CPU architectures, operating systems, or CUDA versions;
in those cases, rerun the installer on the target robot.

When the operating system does not provide `python3-venv`, the installer uses a
package-local `virtualenv.pyz` bootstrap instead of installing a system package.
