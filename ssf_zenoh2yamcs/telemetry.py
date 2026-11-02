"""Decode the existing OmniLRS/SSF messages without importing Isaac or motion stack."""

import math

import msgspec

from .yamcs import Value


class VectorAcceleration(msgspec.Struct):
    ax: float
    ay: float
    az: float


class VectorGyroscope(msgspec.Struct):
    gx: float
    gy: float
    gz: float


class Orientation(msgspec.Struct):
    roll: float
    pitch: float
    yaw: float


class ImuValues(msgspec.Struct):
    lin_acc_m_s2: VectorAcceleration
    ang_vel_rad_s: VectorGyroscope
    orientation_rpy_deg: Orientation


class ImuFrame(msgspec.Struct):
    stamp_s: float
    imu: ImuValues


# Wire schema from motion-stack-2/src/ms_zenoh_bridge/ms_zenoh_bridge/utils.py.
# Defining the wire structs here avoids importing the full motion stack and its
# simulation, visualization and kinematics dependencies.
class JointState(msgspec.Struct, array_like=True):
    name: str
    position: float | None = None
    velocity: float | None = None
    effort: float | None = None


class JointFrame(msgspec.Struct, array_like=True):
    sec: int
    nano: int
    states: list[JointState]


_IMU = msgspec.json.Decoder(ImuFrame)
_JOINTS = msgspec.msgpack.Decoder(JointFrame)


def imu_parameters(payload: bytes, prefix: str = "/Rover/cdh") -> dict[str, Value]:
    frame = _IMU.decode(payload)
    imu = frame.imu
    acceleration = msgspec.to_builtins(imu.lin_acc_m_s2)
    gyroscope = msgspec.to_builtins(imu.ang_vel_rad_s)
    orientation = {key: math.radians(value) for key, value in msgspec.to_builtins(imu.orientation_rpy_deg).items()}
    if not all(
        math.isfinite(value)
        for vector in (acceleration, gyroscope, orientation)
        for value in vector.values()
    ):
        raise ValueError("IMU contains non-finite values")
    return {
        f"{prefix}/imu_accelerometer": acceleration,
        f"{prefix}/imu_gyroscope": gyroscope,
        f"{prefix}/imu_orientation": orientation,
    }


def joint_parameters(payload: bytes) -> dict[str, Value]:
    """Forward all named joint states, preserving missing values and timestamps."""
    frame = _JOINTS.decode(payload)
    for state in frame.states:
        if any(value is not None and not math.isfinite(value) for value in (state.position, state.velocity, state.effort)):
            raise ValueError(f"Non-finite joint state for {state.name}")
    return {"/Rover/motor/joint_telemetry": msgspec.json.encode(msgspec.to_builtins(frame)).decode()}
