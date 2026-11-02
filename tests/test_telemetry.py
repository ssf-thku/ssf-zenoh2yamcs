import math

import msgspec
import pytest

from ssf_zenoh2yamcs.telemetry import imu_parameters, joint_parameters


def imu_payload() -> bytes:
    return msgspec.json.encode(
        {
            "stamp_s": 123.0,
            "imu_prim_path": "/Robots/pioneer/imu",
            "imu": {
                "lin_acc_m_s2": {"ax": 1.0, "ay": 2.0, "az": 3.0},
                "ang_vel_rad_s": {"gx": 0.1, "gy": 0.2, "gz": 0.3},
                "orientation_rpy_deg": {"roll": 10.0, "pitch": 20.0, "yaw": 30.0},
            },
        }
    )


def test_imu_matches_omnilrs_parameters() -> None:
    values = imu_parameters(imu_payload())
    assert values == {
        "/Rover/cdh/imu_accelerometer": {"ax": 1.0, "ay": 2.0, "az": 3.0},
        "/Rover/cdh/imu_gyroscope": {"gx": 0.1, "gy": 0.2, "gz": 0.3},
        "/Rover/cdh/imu_orientation": {"roll": math.radians(10.0), "pitch": math.radians(20.0), "yaw": math.radians(30.0)},
    }


def test_joint_telemetry_preserves_all_names_and_values() -> None:
    states = [["wheel_1", -7.0, 2.0, None], ["arm", 1.0, None, 4.0]]
    payload = msgspec.msgpack.encode([123, 100, states])
    value = joint_parameters(payload)["/Rover/motor/joint_telemetry"]
    assert msgspec.json.decode(value) == [123, 100, states]


def test_nonfinite_joint_state_is_rejected() -> None:
    payload = msgspec.msgpack.encode([123, 0, [["wheel_1", float("nan")]]])
    with pytest.raises(ValueError, match="Non-finite"):
        joint_parameters(payload)


def test_invalid_imu_schema_is_rejected() -> None:
    with pytest.raises(msgspec.ValidationError):
        imu_parameters(b'{"imu":{}}')
