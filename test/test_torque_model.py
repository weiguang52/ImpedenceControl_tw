import pytest

from impendence_control.torque_model import current_to_torque


EXAMPLE_SLOPE_NM_PER_A = 1.328629
EXAMPLE_INTERCEPT_NM = -0.022189


def test_current_to_torque_at_zero_current():
    assert current_to_torque(
        0.0,
        EXAMPLE_SLOPE_NM_PER_A,
        EXAMPLE_INTERCEPT_NM,
    ) == pytest.approx(EXAMPLE_INTERCEPT_NM)


def test_current_to_torque_at_one_ampere():
    expected = EXAMPLE_SLOPE_NM_PER_A + EXAMPLE_INTERCEPT_NM
    assert current_to_torque(
        1.0,
        EXAMPLE_SLOPE_NM_PER_A,
        EXAMPLE_INTERCEPT_NM,
    ) == pytest.approx(expected)


def test_current_to_torque_supports_signed_current():
    expected = EXAMPLE_SLOPE_NM_PER_A * -0.5 + EXAMPLE_INTERCEPT_NM
    assert current_to_torque(
        -0.5,
        EXAMPLE_SLOPE_NM_PER_A,
        EXAMPLE_INTERCEPT_NM,
    ) == pytest.approx(expected)


def test_second_calibrated_model():
    slope = 1.182535
    intercept = -0.025172
    current = 0.5
    assert current_to_torque(current, slope, intercept) == pytest.approx(
        slope * current + intercept
    )
