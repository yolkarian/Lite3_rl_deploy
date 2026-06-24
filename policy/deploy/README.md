# Lite3 deploy policies

This directory is kept for alternative exports. The current original-compatible Zig runner defaults to:

```text
policy/ppo/policy.onnx
```

Expected ONNX I/O for the original-compatible runner:

- inputs: `raw_obs` `[117]`, `raw_obs_history` `[40,117]`
- output: `policy_action` `[12]`

Zig applies the same postprocessing as the original C++ deployment: clip action, multiply by `0.25`, add policy default joint positions, and clamp joint targets. DOF position observations are absolute joint angles, not offsets.
