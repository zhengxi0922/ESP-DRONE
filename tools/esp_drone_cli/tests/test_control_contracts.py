from __future__ import annotations


MOTOR_LAYOUT = (
    {"name": "M1", "x": -1, "y": +1, "spin_is_cw": False},
    {"name": "M2", "x": +1, "y": +1, "spin_is_cw": True},
    {"name": "M3", "x": +1, "y": -1, "spin_is_cw": False},
    {"name": "M4", "x": -1, "y": -1, "spin_is_cw": True},
)


def project_rate_from_body_gyro(gyro_x: float, gyro_y: float, gyro_z: float) -> tuple[float, float, float]:
    return (-gyro_y, gyro_x, -gyro_z)


def mix_axis_outputs(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, throttle: float = 0.2) -> dict[str, float]:
    outputs: dict[str, float] = {}
    for motor in MOTOR_LAYOUT:
        value = throttle
        value += (-motor["x"]) * roll
        value += (-motor["y"]) * pitch
        value += (-1.0 if motor["spin_is_cw"] else 1.0) * yaw
        outputs[motor["name"]] = value
    return outputs


def assert_higher(outputs: dict[str, float], higher: tuple[str, str], lower: tuple[str, str]) -> None:
    for inc_name in higher:
        for dec_name in lower:
            assert outputs[inc_name] > outputs[dec_name], f"{inc_name} should be above {dec_name}: {outputs}"


def test_project_rate_from_body_gyro_matches_axis_truth_table():
    roll_rate, pitch_rate, yaw_rate = project_rate_from_body_gyro(0.0, -30.0, 0.0)
    assert roll_rate == 30.0
    assert pitch_rate == 0.0
    assert yaw_rate == 0.0

    roll_rate, pitch_rate, yaw_rate = project_rate_from_body_gyro(45.0, 0.0, 0.0)
    assert roll_rate == 0.0
    assert pitch_rate == 45.0
    assert yaw_rate == 0.0

    roll_rate, pitch_rate, yaw_rate = project_rate_from_body_gyro(0.0, 0.0, -60.0)
    assert roll_rate == 0.0
    assert pitch_rate == 0.0
    assert yaw_rate == 60.0


def test_mixer_positive_and_negative_roll_direction():
    assert_higher(mix_axis_outputs(roll=+0.05), ("M1", "M4"), ("M2", "M3"))
    assert_higher(mix_axis_outputs(roll=-0.05), ("M2", "M3"), ("M1", "M4"))


def test_mixer_positive_and_negative_pitch_direction():
    assert_higher(mix_axis_outputs(pitch=+0.05), ("M3", "M4"), ("M1", "M2"))
    assert_higher(mix_axis_outputs(pitch=-0.05), ("M1", "M2"), ("M3", "M4"))


def test_mixer_positive_and_negative_yaw_direction():
    assert_higher(mix_axis_outputs(yaw=+0.05), ("M1", "M3"), ("M2", "M4"))
    assert_higher(mix_axis_outputs(yaw=-0.05), ("M2", "M4"), ("M1", "M3"))
