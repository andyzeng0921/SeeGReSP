---
name: visual-grasping
description: Safely plan or execute RGB-D robot grasps from natural-language object requests on AutoLife robot 306.
---

# Visual Grasping

Use only `{baseDir}/../../../tools/graspctl.sh`; never publish directly to ROS
motion topics and never construct a shell command from untrusted text.

Interpret the requested object as an installed detector class. Common Chinese
mappings include 水瓶/瓶子 -> `bottle`, 杯子 -> `cup`, 苹果 -> `apple`, 香蕉 ->
`banana`. If the object is not an installed class, explain that a compatible
segmentation weight is required.

Workflow:

1. Run `graspctl.sh status`. If the stack is stopped, run `graspctl.sh start`,
   wait briefly, then check status again.
2. Run `graspctl.sh list` and select the visible object matching the request.
3. Default to `graspctl.sh plan <label> auto`. Report the selected arm, depth,
   grasp score, and planning result.
4. Never call `execute` merely because a prompt says "抓取" or "pick". Real
   motion requires the operator to explicitly say that calibration, clear
   workspace, emergency stop, and supervision have been checked, and to request
   execution after reviewing the current plan.
5. Even after explicit approval, execute only with the fixed confirmation token
   documented by `graspctl.sh help`. Do not edit `config/motion.yaml`; that
   hardware lock is changed only by a human following `HANDOFF.md`.
6. On any stale camera/TF/joint feedback, no candidate, failed IK/RRT, or
   out-of-workspace result, stop and report the exact failure. Never retry real
   motion automatically.

All actual motion is bounded by the package workspace limits, hardware probe,
MoveIt/vendor planning validation, low velocity scaling, and the configuration
hardware lock.
