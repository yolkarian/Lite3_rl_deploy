# Lite3 deploy policies

Place deploy ONNX files here.

Expected default path:

```text
policy/deploy/lite3_policy.onnx
```

The Zig runner expects the deploy graph I/O contract:

- inputs: `raw_obs` `[B,117]`, `raw_obs_history` `[B,40,117]`
- output: `joint_target` `[B,12]`

Use `scripts/export_policy_from_legged_training.sh` to export a policy from a training checkpoint.