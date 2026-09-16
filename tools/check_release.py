"""Read-only release audit. Never imports a policy or sends robot commands."""
import argparse
import hashlib
import json
from pathlib import Path

import yaml


def audit(root: Path) -> dict:
    directory = root / "rl_sar/policy/g1/running"
    config = yaml.safe_load((directory / "config.yaml").read_text(encoding="utf-8"))["g1/running"]
    source = dict(line.split("=", 1) for line in
                  (directory / "source.txt").read_text(encoding="utf-8").splitlines()
                  if "=" in line)
    policy = directory / config["model_name"]
    errors = []
    digest = None
    if not policy.is_file():
        errors.append("Policy is missing; run git lfs pull.")
    else:
        with policy.open("rb") as stream:
            prefix = stream.read(128)
            stream.seek(0)
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if prefix.startswith(b"version https://git-lfs.github.com/spec/v1"):
            errors.append("Policy is an LFS pointer; run git lfs pull.")
        elif not prefix:
            errors.append("Policy is empty.")
    if config["num_observations"] != int(source["num_observations"]):
        errors.append("Observation dimension differs between config and source metadata.")
    dofs = config["num_of_dofs"]
    if dofs != int(source["num_actions"]):
        errors.append("Action dimension differs between config and source metadata.")
    for key in ("rl_kp", "rl_kd", "fixed_kp", "fixed_kd", "action_scale",
                "torque_limits", "default_dof_pos", "clip_actions_lower", "clip_actions_upper"):
        if len(config[key]) != dofs:
            errors.append(f"{key} must contain {dofs} entries.")
    if sorted(config["joint_mapping"]) != list(range(dofs)):
        errors.append("joint_mapping must be a permutation of the joint indices.")
    return {"ok": not errors, "policy_sha256": digest,
            "source": source.get("source_name"), "errors": errors,
            "scope": "File integrity and configuration only; no inference or hardware validation."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        result = audit(args.root)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        result = {"ok": False, "errors": [str(error)]}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
