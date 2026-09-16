import importlib.util
from pathlib import Path

import pytest
import yaml

spec = importlib.util.spec_from_file_location(
    "release_audit", Path(__file__).resolve().parents[2] / "tools/check_release.py")
audit_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit_module)


@pytest.fixture
def release(tmp_path):
    directory = tmp_path / "rl_sar/policy/g1/running"
    directory.mkdir(parents=True)
    config = {"model_name": "policy.pt", "num_observations": 96, "num_of_dofs": 29,
              "joint_mapping": list(range(29))}
    for key in ("rl_kp", "rl_kd", "fixed_kp", "fixed_kd", "action_scale",
                "torque_limits", "default_dof_pos", "clip_actions_lower", "clip_actions_upper"):
        config[key] = [0] * 29
    (directory / "config.yaml").write_text(yaml.safe_dump({"g1/running": config}))
    (directory / "source.txt").write_text("num_observations=96\nnum_actions=29\n")
    (directory / "policy.pt").write_bytes(b"fixture-only-not-a-real-policy")
    return tmp_path, directory, config


def test_lfs_pointer_is_not_accepted_as_policy(release):
    root, directory, _ = release
    (directory / "policy.pt").write_bytes(b"version https://git-lfs.github.com/spec/v1\n")
    result = audit_module.audit(root)
    assert not result["ok"]
    assert "LFS pointer" in result["errors"][0]


def test_wrong_joint_mapping_is_rejected(release):
    root, directory, config = release
    config["joint_mapping"] = [0] * 29
    (directory / "config.yaml").write_text(yaml.safe_dump({"g1/running": config}))
    assert not audit_module.audit(root)["ok"]


def test_valid_file_audit_returns_digest(release):
    result = audit_module.audit(release[0])
    assert result["ok"]
    assert len(result["policy_sha256"]) == 64
