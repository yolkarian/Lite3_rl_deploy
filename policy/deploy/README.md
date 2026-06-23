# Lite3 deploy policies

Place deploy ONNX files here.

Expected default path:

```text
policy/deploy/lite3_policy.onnx
```

The Zig runner expects the default single-sample deploy graph I/O contract:

- inputs: `raw_obs` `[117]`, `raw_obs_history` `[40,117]`
- output: `joint_target` `[12]`

The default export includes normalizer/pre-postprocessing inside ONNX. Use `scripts/export_policy_from_legged_training.sh` to export a policy from a training checkpoint.
