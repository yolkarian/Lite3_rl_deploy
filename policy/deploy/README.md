# Lite3 deploy policies

Put `legged-training` exported deploy ONNX files here.

Expected default path:

```text
policy/deploy/lite3_policy.onnx
```

Export from the sibling training repository:

```bash
cd ../legged-training
uv run python scripts/export_policy_onnx.py \
  --policy-path outputs/<run>/checkpoints/latest.eqx \
  --output-path ../Lite3_rl_deploy/policy/deploy/lite3_policy.onnx \
  --metadata-path ../Lite3_rl_deploy/policy/deploy/lite3_policy.metadata.json
```

The Zig runner expects the default deploy graph from `export_policy_onnx.py`:

- inputs: `raw_obs` `[B,117]`, `raw_obs_history` `[B,40,117]`
- output: `joint_target` `[B,12]`
