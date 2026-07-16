# Portable runtime layout

Run `tools/install_ai_environment.sh` from the package source directory. The
installer keeps every optional AI runtime asset inside this package:

```text
adaptive_object_grasping/
  third_party/venv/
  third_party/virtualenv.pyz
  third_party/wheels/
  third_party/pip-cache/
  third_party/ultralytics-config/
  third_party/matplotlib-config/
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

The package-local pip cache is purged after a successful install to reduce the
folder size. Set `KEEP_PIP_CACHE=1` before running the installer only when an
offline reinstall cache is intentionally required.

When the operating system does not provide `python3-venv`, the installer uses a
package-local `virtualenv.pyz` bootstrap instead of installing a system package.
The installer is idempotent and does not clear an existing environment. Set
`PYTORCH_WHEEL_MIRROR_URL` to a trusted mirror URL when the official PyTorch CDN
is slow; the downloaded wheel is accepted only when it matches the SHA256 from
the official PyTorch package index.

On ROS 2 Jazzy/Python 3.12, the installer deliberately ignores graspnetAPI
1.2.11's obsolete pins for NumPy 1.20.3 and transforms3d 0.3.1, and installs
Python-3.12-compatible versions before installing graspnetAPI itself.
The online adapter loads only `graspnetAPI.grasp.GraspGroup`; it does not import
the optional Dex-Net dataset-evaluation stack or require `autolab_core`.
