"""The run budgets must be derived from the settings, not hardcoded for the defaults.

Found during independent verification: `max_stall_ms`, `max_torque_ms` and
`max_wall_clock_s` were fixed constants whose arithmetic was correct *only* at the
shipped defaults. Any knob that scales the work silently invalidated them:

  - `confirmations=3` makes the true worst-case stall 19,200ms against a fixed
    14,400ms cap, so every legitimate full-arm run would abort.
  - halving `step` doubles the probe packets, blowing a fixed torque cap.

An abort is safe (nothing is written) but it means a correct calibration cannot
complete, and the operator has no way to tell a real safety trip from an arithmetic
bug. These tests pin the scaling relationship so the budgets cannot silently drift
out of step with the settings again.
"""

import pytest

from robohand.calibrate import (
    CalibrationSettings,
    derive_packet_budget,
    derive_stall_budget,
    derive_torque_budget,
    derive_wall_clock_budget,
)


@pytest.mark.parametrize("confirmations", [2, 3, 4, 5, 10, 20])
def test_stall_cap_always_covers_the_derived_worst_case(confirmations):
    """The cap must never be lower than the worst case it is meant to bound."""
    s = CalibrationSettings(confirmations=confirmations)
    worst = derive_stall_budget(s.step_duration_ms, confirmations)
    assert s.max_stall_ms >= worst, (
        f"confirmations={confirmations}: cap {s.max_stall_ms} < worst case {worst}; "
        "a legitimate run would abort"
    )


def test_stall_worst_case_is_set_by_confirmations_not_search_distance():
    """Once a joint is pinned every later step is a no-motion miss, so the number of
    STALLING packets is set by `confirmations`. Widening the search must not change it."""
    a = derive_stall_budget(400, 2)
    b = derive_stall_budget(400, 3)
    assert b > a
    # one extra confirmation == one extra stalling packet per joint per direction
    assert b - a == 6 * 2 * 400


@pytest.mark.parametrize("step", [30, 20, 15, 10, 5])
def test_torque_and_wall_budgets_scale_as_the_step_shrinks(step):
    """A finer step means proportionally more probe packets; the budgets must follow."""
    s = CalibrationSettings(step=step)
    packets = derive_packet_budget(step, s.verification_moves)
    assert s.max_torque_ms >= packets * s.step_duration_ms
    assert s.max_wall_clock_s >= packets * s.step_duration_ms / 1000.0


def test_finer_step_gets_a_strictly_larger_budget():
    assert CalibrationSettings(step=10).max_torque_ms > CalibrationSettings(step=30).max_torque_ms
    assert CalibrationSettings(step=10).max_wall_clock_s > CalibrationSettings(step=30).max_wall_clock_s


def test_derived_defaults_still_match_the_documented_default_arithmetic():
    """At the shipped defaults the derivation must reproduce the hand-checked numbers,
    so this change is a generalisation rather than a silent loosening."""
    s = CalibrationSettings()
    assert s.max_stall_ms == 14_400  # 6 joints x 2 directions x 3 packets x 400ms
    assert 170_000 <= s.max_torque_ms <= 200_000
    assert 800 <= s.max_wall_clock_s <= 1000


@pytest.mark.parametrize(
    "kwargs,field,expected",
    [
        ({"max_stall_ms": 999}, "max_stall_ms", 999),
        ({"max_torque_ms": 12_345}, "max_torque_ms", 12_345),
        ({"max_wall_clock_s": 42.5}, "max_wall_clock_s", 42.5),
    ],
)
def test_explicit_budget_overrides_are_still_honoured(kwargs, field, expected):
    """Deriving the default must not take away the operator's ability to set a tighter one."""
    assert getattr(CalibrationSettings(**kwargs), field) == expected


def test_budgets_remain_validated_after_derivation():
    with pytest.raises(Exception):
        CalibrationSettings(max_stall_ms=-1)
    with pytest.raises(Exception):
        CalibrationSettings(max_torque_ms=0)
    with pytest.raises(Exception):
        CalibrationSettings(max_wall_clock_s=0)


def test_settings_remain_immutable():
    """Derivation uses object.__setattr__; that must not make the dataclass mutable."""
    s = CalibrationSettings()
    with pytest.raises(Exception):
        s.max_stall_ms = 1  # type: ignore[misc]
