"""Coverage proof for the hardening task: a servo position outside the
hardware-legal electrical range (``config.ELECTRICAL_LIMITS``) must never be
accepted, from ANY entry point -- direct API, CLI parsing, config.json loading,
poses.json loading, sequence file loading, ``pose --save``/``teach`` write paths,
overrides, ramp interpolation, or the transport layer itself.

Table-driven wherever the same illegal-value set is checked across multiple
entry points, per the task's explicit test matrix:

  GRIP at 700 / 1000 / 1499 / 2501 / -5 / 0 / "abc" / 1e9 must be rejected via
  every entry point: direct API, CLI move, CLI --joints, pose file, sequence
  file, pose --save, and raw transport call.

  Every non-gripper servo at 499 / 2501 must be rejected.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent

from robohand import cli, config
from robohand import poses as pose_store
from robohand import sequences
from robohand.arm import Arm
from robohand.protocol import IllegalServoTarget, MotionToken
from robohand.safety import CalibrationRequired, PositionOutOfRange, SafetyLayer
from robohand.sim import SimTransport

GRIP_ID = config.CANONICAL_JOINTS["GRIP"]
NON_GRIP_JOINTS = [n for n in config.JOINT_NAMES if n != "GRIP"]

# The exact table called out in the task: integers that are illegal for GRIP no
# matter which entry point they arrive through (below the 1500 electrical floor,
# above the 2500 ceiling, negative, or zero).
GRIP_ILLEGAL_INT_VALUES = [700, 1000, 1499, 2501, -5, 0]


# ---------------------------------------------------------------------------
# 1) Direct API (SafetyLayer.move / config.validate_position_value)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", GRIP_ILLEGAL_INT_VALUES)
def test_direct_api_rejects_illegal_grip_int(value):
    """config.validate_position_value is the explicit hardware-law check: it must
    reject every illegal GRIP integer, naming the electrical range.
    """
    with pytest.raises(config.InvalidPosition):
        config.validate_position_value("GRIP", value)


@pytest.mark.parametrize("value", ["abc", 1e9, 1500.0, True])
def test_direct_api_rejects_illegal_grip_non_int(value):
    with pytest.raises(config.InvalidPosition):
        config.validate_position_value("GRIP", value)


@pytest.mark.parametrize("value", GRIP_ILLEGAL_INT_VALUES)
def test_direct_api_safety_layer_rejects_illegal_grip(value):
    """SafetyLayer.move (the real move choke point) must also refuse every
    illegal GRIP value -- as a CalibrationRequired (uncalibrated, provisional
    band) or PositionOutOfRange refusal, never a silent clamp -- and must send
    zero MOVE packets to the transport.
    """
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises((CalibrationRequired, PositionOutOfRange)):
            safety.move({"GRIP": value}, duration_ms=400, wait=False)
    finally:
        safety.close()
    assert [e for e in transport.log if e.get("cmd") == "MOVE"] == []
    assert transport.stall_ms_total() == 0


@pytest.mark.parametrize("value", ["abc", 1e9])
def test_direct_api_safety_layer_rejects_illegal_grip_non_int(value):
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises(PositionOutOfRange):
            safety.move({"GRIP": value}, duration_ms=400, wait=False)
    finally:
        safety.close()
    assert [e for e in transport.log if e.get("cmd") == "MOVE"] == []


@pytest.mark.parametrize("name", NON_GRIP_JOINTS)
@pytest.mark.parametrize("value", [499, 2501])
def test_direct_api_rejects_non_grip_out_of_electrical_range(name, value):
    with pytest.raises(config.InvalidPosition):
        config.validate_position_value(name, value)

    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises((CalibrationRequired, PositionOutOfRange)):
            safety.move({name: value}, duration_ms=400, wait=False)
    finally:
        safety.close()
    assert [e for e in transport.log if e.get("cmd") == "MOVE"] == []


# ---------------------------------------------------------------------------
# 2) CLI: move --joints '{"GRIP": ...}'
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", GRIP_ILLEGAL_INT_VALUES)
def test_cli_move_joints_rejects_illegal_grip(value, capsys):
    rc = cli.main(["--sim", "move", "--joints", json.dumps({"GRIP": value})])
    assert rc != 0
    captured = capsys.readouterr()
    assert "GRIP" in captured.err


@pytest.mark.parametrize("value", ['"abc"', "1e9"])
def test_cli_move_joints_rejects_illegal_grip_non_int(value, capsys):
    rc = cli.main(["--sim", "move", "--joints", '{"GRIP": %s}' % value])
    assert rc != 0
    captured = capsys.readouterr()
    assert "GRIP" in captured.err


def test_cli_move_joints_rejects_malformed_json(capsys):
    rc = cli.main(["--sim", "move", "--joints", "not json"])
    assert rc != 0
    captured = capsys.readouterr()
    assert "JSON" in captured.err


def test_cli_move_joints_rejects_unknown_joint_name(capsys):
    rc = cli.main(["--sim", "move", "--joints", '{"NOT_A_JOINT": 1500}'])
    assert rc != 0
    captured = capsys.readouterr()
    assert "unknown joint" in captured.err.lower()


@pytest.mark.parametrize("name", NON_GRIP_JOINTS)
@pytest.mark.parametrize("value", [499, 2501])
def test_cli_move_joints_rejects_non_grip_out_of_range(name, value, capsys):
    rc = cli.main(["--sim", "move", "--joints", json.dumps({name: value})])
    assert rc != 0
    captured = capsys.readouterr()
    assert name in captured.err


# ---------------------------------------------------------------------------
# 3) CLI: move --servo GRIP --pos ...
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", GRIP_ILLEGAL_INT_VALUES)
def test_cli_move_servo_pos_rejects_illegal_grip(value, capsys):
    rc = cli.main(["--sim", "move", "--servo", "GRIP", "--pos", str(value)])
    assert rc != 0
    captured = capsys.readouterr()
    assert "GRIP" in captured.err


def test_cli_move_servo_pos_rejects_non_integer(capsys):
    # argparse itself enforces --pos is an int; this is still a clear non-zero
    # exit failure, just raised one layer earlier than InvalidPosition.
    with pytest.raises(SystemExit) as exc_info:
        cli.main(["--sim", "move", "--servo", "GRIP", "--pos", "abc"])
    assert exc_info.value.code != 0


# ---------------------------------------------------------------------------
# 4) CLI: grip --step (non-positive step must be rejected, not abs()'d)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("step", [-5, 0])
def test_cli_grip_rejects_non_positive_step(step, capsys):
    rc = cli.main(["--sim", "grip", "open", "--step", str(step)])
    assert rc != 0
    captured = capsys.readouterr()
    assert "--step" in captured.err


# ---------------------------------------------------------------------------
# 5) pose file loading: an out-of-range pose must be rejected at load time
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", GRIP_ILLEGAL_INT_VALUES)
def test_pose_file_rejects_illegal_grip_at_load(tmp_path, value):
    path = tmp_path / "poses.json"
    path.write_text(json.dumps({"bad": {"positions": {"GRIP": value}, "duration_ms": 500}}))
    with pytest.raises(config.InvalidPosition):
        pose_store.load_poses(path)


@pytest.mark.parametrize("name", NON_GRIP_JOINTS)
@pytest.mark.parametrize("value", [499, 2501])
def test_pose_file_rejects_non_grip_out_of_range_at_load(tmp_path, name, value):
    path = tmp_path / "poses.json"
    path.write_text(json.dumps({"bad": {"positions": {name: value}, "duration_ms": 500}}))
    with pytest.raises(config.InvalidPosition):
        pose_store.load_poses(path)


def test_pose_file_shipped_data_still_loads(tmp_path):
    """The real shipped poses.json must still load cleanly after this hardening."""
    poses = pose_store.load_poses(pose_store.DEFAULT_POSES_PATH)
    assert "home" in poses


# ---------------------------------------------------------------------------
# 6) sequence file loading: same validation, applied wholesale, before motion
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", GRIP_ILLEGAL_INT_VALUES)
def test_sequence_file_rejects_illegal_grip_at_load(tmp_path, value):
    path = tmp_path / "seq.json"
    path.write_text(json.dumps([{"positions": {"GRIP": value}, "duration_ms": 500}]))
    with pytest.raises(config.InvalidPosition):
        sequences.load_sequence(path)


def test_sequence_with_one_illegal_step_performs_zero_motion(tmp_path, monkeypatch):
    """A sequence with two legal steps and one illegal step must be rejected
    wholesale, before any motion starts -- the simulator must receive ZERO move
    commands, not two-out-of-three.
    """
    steps = [
        {"positions": {"ELBOW": 1600}, "duration_ms": 500},
        {"positions": {"GRIP": 700}, "duration_ms": 500},  # illegal: below 1500 floor
        {"positions": {"WRIST_ROLL": 1500}, "duration_ms": 500},
    ]
    path = tmp_path / "seq.json"
    path.write_text(json.dumps(steps))

    with pytest.raises(config.InvalidPosition):
        sequences.load_sequence(path)

    # Also prove play_sequence's own defence-in-depth re-validation catches it
    # even if a caller bypassed load_sequence and constructed steps in-process.
    with Arm(sim=True) as arm:
        with pytest.raises(config.InvalidPosition):
            sequences.play_sequence(arm, steps)
        move_entries = [e for e in arm._transport.log if e.get("cmd") == "MOVE"]
        assert move_entries == [], (
            f"expected zero MOVE packets for a sequence with an illegal step, got {move_entries}"
        )
        assert arm._transport.stall_ms_total() == 0


def test_sequence_file_rejects_unknown_pose_reference(tmp_path):
    path = tmp_path / "seq.json"
    path.write_text(json.dumps([{"pose": "no_such_pose", "duration_ms": 500}]))
    with pytest.raises(config.InvalidPosition):
        sequences.load_sequence(path)


# ---------------------------------------------------------------------------
# 7) pose --save / teach write paths: must refuse to WRITE an illegal value
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", GRIP_ILLEGAL_INT_VALUES)
def test_pose_save_refuses_to_write_illegal_grip(tmp_path, value):
    path = tmp_path / "poses.json"
    with pytest.raises(config.InvalidPosition):
        pose_store.save_pose("bad", {"GRIP": value}, duration_ms=500, path=path)
    assert not path.exists(), "an illegal pose must never be written to disk"


def test_pose_save_refuses_illegal_value_without_disturbing_existing_file(tmp_path):
    path = tmp_path / "poses.json"
    pose_store.save_pose("good", {"GRIP": 1900}, duration_ms=500, path=path)
    before = path.read_text()
    with pytest.raises(config.InvalidPosition):
        pose_store.save_pose("bad", {"GRIP": 700}, duration_ms=500, path=path)
    assert path.read_text() == before, "a rejected save must not corrupt/touch the existing file"


def test_cli_teach_rejects_illegal_hand_typed_position(tmp_path, monkeypatch):
    poses_path = tmp_path / "poses.json"
    monkeypatch.setenv("ROBOHAND_POSES", str(poses_path))
    inputs = iter(["700"])  # hand-typed GRIP position, below the 1500 floor
    monkeypatch.setattr("builtins.input", lambda *_a: next(inputs))
    rc = cli.main(["--sim", "teach", "bad_taught_pose", "--servo", "GRIP"])
    assert rc == 1
    assert not poses_path.exists()


def test_cli_teach_rejects_non_integer_hand_typed_position(tmp_path, monkeypatch):
    poses_path = tmp_path / "poses.json"
    monkeypatch.setenv("ROBOHAND_POSES", str(poses_path))
    inputs = iter(["abc"])
    monkeypatch.setattr("builtins.input", lambda *_a: next(inputs))
    rc = cli.main(["--sim", "teach", "bad_taught_pose", "--servo", "GRIP"])
    assert rc == 1
    assert not poses_path.exists()


def test_cli_pose_save_writes_only_legal_positions(tmp_path, monkeypatch):
    """pose --save reads current (already-validated) commanded positions -- prove
    the full CLI round trip still only ever persists legal values.
    """
    poses_path = tmp_path / "poses.json"
    monkeypatch.setenv("ROBOHAND_POSES", str(poses_path))
    rc = cli.main(["--sim", "move", "--servo", "GRIP", "--pos", "1900"])
    assert rc == 0
    rc = cli.main(["--sim", "pose", "--save", "current"])
    assert rc == 0
    saved = json.loads(poses_path.read_text())
    assert saved["current"]["positions"]["GRIP"] == 1900


# ---------------------------------------------------------------------------
# 8) config.json loading: malformed configs must fail loudly at import time
# ---------------------------------------------------------------------------


def _malformed_config_result(tmp_path, mutate) -> subprocess.CompletedProcess:
    base = json.loads((REPO_ROOT / "robohand" / "data" / "config.json").read_text())
    mutate(base)
    cfg_path = tmp_path / "config.json"
    cfg_path.write_text(json.dumps(base))
    env = dict(os.environ)
    env["ROBOHAND_CONFIG"] = str(cfg_path)
    env["ROBOHAND_TRANSPORT"] = "sim"
    env["PYTHONPATH"] = str(REPO_ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    return subprocess.run(
        [sys.executable, "-c", "import robohand.config"],
        capture_output=True,
        text=True,
        env=env,
        cwd=str(REPO_ROOT),
        timeout=30,
    )


def test_config_json_rejects_inverted_min_max(tmp_path):
    def mutate(cfg):
        cfg["servos"]["ELBOW"]["min"] = 2000
        cfg["servos"]["ELBOW"]["max"] = 1000

    result = _malformed_config_result(tmp_path, mutate)
    assert result.returncode != 0
    assert "ELBOW" in result.stderr
    assert "invert" in result.stderr.lower()


def test_config_json_rejects_out_of_electrical_range(tmp_path):
    def mutate(cfg):
        cfg["servos"]["ELBOW"]["max"] = 9999

    result = _malformed_config_result(tmp_path, mutate)
    assert result.returncode != 0
    assert "ELBOW" in result.stderr
    assert "electrical" in result.stderr.lower()


def test_config_json_rejects_grip_below_electrical_floor(tmp_path):
    """GRIP's config-level min must never be widened below its 1500 electrical floor."""

    def mutate(cfg):
        cfg["servos"]["GRIP"]["min"] = 700

    result = _malformed_config_result(tmp_path, mutate)
    assert result.returncode != 0
    assert "GRIP" in result.stderr
    assert "electrical" in result.stderr.lower()


def test_config_json_rejects_unknown_joint_name(tmp_path):
    def mutate(cfg):
        cfg["servos"]["FOOBAR"] = {"id": 7, "min": 500, "max": 2500, "center": 1500, "calibrated": False}

    result = _malformed_config_result(tmp_path, mutate)
    assert result.returncode != 0
    assert "FOOBAR" in result.stderr
    assert "unknown joint" in result.stderr.lower()


def test_config_json_rejects_string_value(tmp_path):
    def mutate(cfg):
        cfg["servos"]["ELBOW"]["min"] = "five hundred"

    result = _malformed_config_result(tmp_path, mutate)
    assert result.returncode != 0
    assert "ELBOW" in result.stderr
    assert "integer" in result.stderr.lower()


def test_config_json_rejects_center_outside_own_band(tmp_path):
    def mutate(cfg):
        cfg["servos"]["ELBOW"]["center"] = 5000

    result = _malformed_config_result(tmp_path, mutate)
    assert result.returncode != 0
    assert "ELBOW" in result.stderr
    assert "center" in result.stderr.lower()


def test_config_json_shipped_file_still_loads_cleanly(tmp_path):
    """The real shipped config.json must still import cleanly after this hardening
    (i.e. none of the new checks are so strict they reject known-good data).
    """
    env = dict(os.environ)
    env.pop("ROBOHAND_CONFIG", None)
    env["ROBOHAND_TRANSPORT"] = "sim"
    env["PYTHONPATH"] = str(REPO_ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    result = subprocess.run(
        [sys.executable, "-c", "import robohand.config; print('OK')"],
        capture_output=True,
        text=True,
        env=env,
        cwd=str(REPO_ROOT),
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert "OK" in result.stdout


# ---------------------------------------------------------------------------
# 9) Overrides cannot escape the electrical range or the GRIP floor
# ---------------------------------------------------------------------------


def test_override_cannot_take_grip_below_1500():
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises(PositionOutOfRange):
            safety.move(
                {"GRIP": 1499},
                duration_ms=400,
                wait=False,
                allow_uncalibrated_override=True,
                override_reason="test: override must never reach below 1500",
            )
    finally:
        safety.close()
    assert [e for e in transport.log if e.get("cmd") == "MOVE"] == []


@pytest.mark.parametrize("name", NON_GRIP_JOINTS)
def test_override_cannot_escape_electrical_range(name):
    elec_lo, elec_hi = config.electrical_bounds(name)
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises(PositionOutOfRange):
            safety.move(
                {name: elec_hi + 1},
                duration_ms=400,
                wait=False,
                allow_uncalibrated_override=True,
                override_reason="test: override must never escape the electrical ceiling",
            )
        with pytest.raises(PositionOutOfRange):
            safety.move(
                {name: elec_lo - 1},
                duration_ms=400,
                wait=False,
                allow_uncalibrated_override=True,
                override_reason="test: override must never escape the electrical floor",
            )
    finally:
        safety.close()
    assert [e for e in transport.log if e.get("cmd") == "MOVE"] == []


def test_override_grip_within_configured_band_still_succeeds():
    """The override must still work for a request that IS inside the joint's
    configured band -- it only refuses requests that remain out of range.
    """
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        safety.move(
            {"GRIP": 2200},
            duration_ms=400,
            wait=False,
            allow_uncalibrated_override=True,
            override_reason="test: in-band override request must succeed",
        )
    finally:
        safety.close()
    move_entries = [e for e in transport.log if e.get("cmd") == "MOVE"]
    assert len(move_entries) == 1
    assert transport.stall_ms_total() == 0


# ---------------------------------------------------------------------------
# 10) Ramp interpolation: every intermediate step must stay in range
# ---------------------------------------------------------------------------


def test_ramp_intermediates_all_within_electrical_and_effective_bounds():
    transport = SimTransport()
    safety = SafetyLayer(transport, max_step_per_move=50, install_signal_handlers=False)
    try:
        # 1650 -> 2250 spans GRIP's provisional band end to end; with a 50-unit
        # max step this forces many ramp steps, maximizing the chance of an
        # off-by-one intermediate escaping the band if the clamp logic were wrong.
        safety.move({"GRIP": 2250}, current={"GRIP": 1650}, duration_ms=1000, wait=False)
    finally:
        safety.close()

    move_entries = [e for e in transport.log if e["cmd"] == "MOVE"]
    assert len(move_entries) > 1
    elec_lo, elec_hi = config.electrical_bounds("GRIP")
    eff_lo, eff_hi = config.effective_bounds("GRIP")
    for entry in move_entries:
        for servo_id, target in entry["targets"]:
            assert elec_lo <= target <= elec_hi, f"ramp intermediate {target} escaped electrical range"
            assert eff_lo <= target <= eff_hi, f"ramp intermediate {target} escaped effective (policy) range"
    assert transport.stall_ms_total() == 0


def test_ramp_intermediates_within_range_for_calibrated_style_long_move(monkeypatch):
    """Same proof, but forcing calibrated=True style effective bounds (full
    configured min/max) so the ramp spans a wider, still-legal range.
    """
    transport = SimTransport()
    safety = SafetyLayer(transport, max_step_per_move=30, install_signal_handlers=False)
    try:
        safety.move(
            {"ELBOW": 1800},
            current={"ELBOW": 1200},
            duration_ms=1000,
            wait=False,
            allow_uncalibrated_override=True,
            override_reason="test: wide ramp within configured band",
        )
    finally:
        safety.close()

    move_entries = [e for e in transport.log if e["cmd"] == "MOVE"]
    assert len(move_entries) > 1
    elec_lo, elec_hi = config.electrical_bounds("ELBOW")
    for entry in move_entries:
        for servo_id, target in entry["targets"]:
            assert elec_lo <= target <= elec_hi
    assert transport.stall_ms_total() == 0


# ---------------------------------------------------------------------------
# 11) Transport layer backstop: even a valid MotionToken cannot carry an
#     illegal value through move_servos()
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("value", GRIP_ILLEGAL_INT_VALUES)
def test_transport_backstop_rejects_illegal_grip_even_with_valid_token(value):
    transport = SimTransport()
    token = MotionToken(id(transport))
    with pytest.raises(IllegalServoTarget):
        transport.move_servos([(GRIP_ID, value)], 500, token=token)
    assert transport.log == []


@pytest.mark.parametrize("value", ["abc", 1e9, True])
def test_transport_backstop_rejects_illegal_grip_non_int(value):
    transport = SimTransport()
    token = MotionToken(id(transport))
    with pytest.raises(IllegalServoTarget):
        transport.move_servos([(GRIP_ID, value)], 500, token=token)
    assert transport.log == []


@pytest.mark.parametrize("name", NON_GRIP_JOINTS)
@pytest.mark.parametrize("value", [499, 2501])
def test_transport_backstop_rejects_non_grip_out_of_range(name, value):
    servo_id = config.CANONICAL_JOINTS[name]
    transport = SimTransport()
    token = MotionToken(id(transport))
    with pytest.raises(IllegalServoTarget):
        transport.move_servos([(servo_id, value)], 500, token=token)
    assert transport.log == []


def test_transport_backstop_accepts_legal_value():
    transport = SimTransport()
    token = MotionToken(id(transport))
    transport.move_servos([(GRIP_ID, 1900)], 500, token=token)
    assert len(transport.log) == 1
    assert transport.stall_ms_total() == 0


# ---------------------------------------------------------------------------
# 12) Duration/time parameters: non-positive or absurd durations rejected
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("duration_ms", [0, -1, -1000])
def test_safety_layer_rejects_non_positive_duration(duration_ms):
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises(config.InvalidPosition):
            safety.move({"GRIP": 1900}, duration_ms=duration_ms, wait=False)
    finally:
        safety.close()
    assert [e for e in transport.log if e.get("cmd") == "MOVE"] == []


def test_safety_layer_rejects_absurd_duration():
    transport = SimTransport()
    safety = SafetyLayer(transport, install_signal_handlers=False)
    try:
        with pytest.raises(config.InvalidPosition):
            safety.move({"GRIP": 1900}, duration_ms=10**9, wait=False)
    finally:
        safety.close()
    assert [e for e in transport.log if e.get("cmd") == "MOVE"] == []


@pytest.mark.parametrize("duration_ms", [0, -1])
def test_transport_backstop_rejects_non_positive_duration(duration_ms):
    transport = SimTransport()
    token = MotionToken(id(transport))
    with pytest.raises(IllegalServoTarget):
        transport.move_servos([(GRIP_ID, 1900)], duration_ms, token=token)
    assert transport.log == []
