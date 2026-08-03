# Acceptance gates

Reject the candidate if any condition fails:

- Fewer than five exact RGB-D samples or missing same-frame camera transform.
- Any sample uses a navigation/fisheye camera instead of `rgbd_head_color` and
  aligned `rgbd_head_depth` shared memory.
- Fused top-height spread exceeds `0.03 m`.
- Median per-frame plane residual exceeds `0.012 m`.
- Fused footprint is smaller than `0.20 m` on either horizontal axis.
- The selected tabletop target lies outside the fused XY footprint or its base
  height disagrees with the table top/object geometry.
- MoveIt reports current-state robot/environment collision.
- RViz does not show the current robot and translucent table box together in
  `Link_Zero_Point`, or the screenshot is ambiguous.
- The original formal configuration cannot be restored and hash-verified.

Passing these gates validates geometric consistency only. It does not verify a
reset pose, solve IK, authorize hardware, or prove that a planned trajectory is
collision-free. Run those checks separately for every fresh grasp.
