"""Decode the SSF MDB's UDP commands and map them to existing Zenoh APIs."""

import logging
import math
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import asyncio_for_robotics as afor
import msgspec
from asyncio_for_robotics.zenoh import current_session

from .yamcs import CommandReceiver, Value

logger = logging.getLogger(__name__)
NS = {"x": "http://www.omg.org/spec/XTCE/20180204"}
CAMERA_REQUEST_KEY = "cameras/capture"
DEFAULT_MDB = (
    Path(__file__).resolve().parents[2]
    / "yamcs-mission-control/yamcs-server/src/main/yamcs/mdb/rover.xml"
)

# Route supported motion-stack commands and Isaac runtime camera requests.
# Commands requiring other OmniLRS subsystems (power, tether, etc.) are excluded.
ROUTES = {
    "/Rover/camera/camera_capture_high": CAMERA_REQUEST_KEY,
    "/Rover/camera/camera_capture_group": CAMERA_REQUEST_KEY,
    "/Rover/motor/drive_straight": "drive_straight",
    "/Rover/motor/drive_turn": "drive_turn",
    "/Rover/motor/set_arm_joint": "set_arm_joint",
    "/Rover/motor/set_arm_tip": "set_arm_tip",
    "/Rover/cdh/deploy_solar_panel": "solar_panel",
    "/Rover/cdh/deploy_manipulator": "manipulator",
    "/Rover/cdh/deploy_legs": "legs",
    **{
        f"/Rover/cdh/{name}": name
        for name in (
            "cancel_motion",
            "approach_rock",
            "grasp_rock",
            "lift_rock",
            "store_rock",
            "open_gripper",
            "close_gripper",
        )
    },
}


@dataclass(frozen=True)
class Argument:
    name: str
    format: str | None
    labels: dict[int, str] | None = None
    terminator: bytes | None = None


@dataclass(frozen=True)
class Command:
    name: str
    wire_name: bytes
    arguments: tuple[Argument, ...]

    def decode(self, data: bytes) -> dict[str, Value]:
        offset = len(self.wire_name)
        result: dict[str, Value] = {}
        for arg in self.arguments:
            if arg.terminator is not None:
                end = data.find(arg.terminator, offset)
                if end < 0:
                    raise ValueError(f"Missing string terminator in {self.name}")
                result[arg.name] = data[offset:end].decode("ascii")
                offset = end + len(arg.terminator)
                continue
            try:
                value = struct.unpack_from(arg.format, data, offset)[0]
            except struct.error as exc:
                raise ValueError(
                    f"Truncated argument {arg.name} in {self.name}"
                ) from exc
            offset += struct.calcsize(arg.format)
            if arg.labels is not None:
                if value not in arg.labels:
                    raise ValueError(f"Unknown enum value {value} for {arg.name}")
                value = arg.labels[value]
            elif not math.isfinite(value):
                raise ValueError(f"Non-finite {arg.name} in {self.name}")
            result[arg.name] = value
        if offset != len(data):
            raise ValueError(f"Unexpected trailing bytes in {self.name}")
        return result


def load_commands(path: Path) -> tuple[Command, ...]:
    """Read the supported fixed-name, packed float/enum layouts from XTCE.

    This is the subset used by SSF's existing UDP link, not a general XTCE
    decoder. Reject a changed layout at startup instead of guessing its bytes.
    """
    root = ET.parse(path).getroot()
    parents = {child: parent for parent in root.iter() for child in parent}
    commands: list[Command] = []
    for meta in root.findall(".//x:MetaCommand", NS):
        node = parents[meta]
        parts = [meta.attrib["name"]]
        while node is not None:
            if node.tag == f"{{{NS['x']}}}SpaceSystem":
                parts.append(node.attrib["name"])
            node = parents.get(node)
        name = "/" + "/".join(reversed(parts))
        if name not in ROUTES:
            continue
        metadata = parents[parents[meta]]
        type_set = metadata.find("x:ArgumentTypeSet", NS)
        types = {
            item.attrib["name"]: item
            for item in (type_set if type_set is not None else ())
        }
        arguments = {
            item.attrib["name"]: item.attrib["argumentTypeRef"]
            for item in meta.findall("x:ArgumentList/x:Argument", NS)
        }
        entries = meta.findall("x:CommandContainer/x:EntryList/*", NS)
        if not entries or entries[0].tag != f"{{{NS['x']}}}FixedValueEntry":
            raise ValueError(f"Expected fixed command identifier for {name}")
        wire = bytes.fromhex(entries[0].attrib["binaryValue"])
        if not wire or int(entries[0].attrib["sizeInBits"]) != len(wire) * 8:
            raise ValueError(f"Invalid command identifier for {name}")
        specs: list[Argument] = []
        for entry in entries:
            location = entry.find("x:LocationInContainerInBits", NS)
            if location is not None and (
                location.attrib.get("referenceLocation", "previousEntry")
                != "previousEntry"
                or location.findtext("x:FixedValue", default="0", namespaces=NS) != "0"
            ):
                raise ValueError(f"Unsupported command entry offset in {name}")
        for entry in entries[1:]:
            if entry.tag != f"{{{NS['x']}}}ArgumentRefEntry":
                raise ValueError(f"Unsupported command entry in {name}")
            arg_name = entry.attrib["argumentRef"]
            arg_type = types[arguments[arg_name]]
            encoding = arg_type.find("x:FloatDataEncoding", NS)
            if encoding is not None:
                bits = int(encoding.attrib["sizeInBits"])
                order = encoding.attrib.get("byteOrder", "mostSignificantByteFirst")
                if (
                    encoding.attrib.get("encoding") != "IEEE754_1985"
                    or bits not in (32, 64)
                    or order
                    not in ("mostSignificantByteFirst", "leastSignificantByteFirst")
                ):
                    raise ValueError(f"Unsupported float encoding in {name}")
                fmt = (">" if order == "mostSignificantByteFirst" else "<") + (
                    "f" if bits == 32 else "d"
                )
                specs.append(Argument(arg_name, fmt))
            elif arg_type.tag == f"{{{NS['x']}}}EnumeratedArgumentType":
                encoding = arg_type.find("x:IntegerDataEncoding", NS)
                if encoding is None or encoding.attrib.get("sizeInBits") != "8":
                    raise ValueError(f"Expected 8-bit enum in {name}")
                labels = {
                    int(item.attrib["value"]): item.attrib["label"]
                    for item in arg_type.findall("x:EnumerationList/x:Enumeration", NS)
                }
                specs.append(Argument(arg_name, ">B", labels))
            elif arg_type.tag == f"{{{NS['x']}}}StringArgumentType":
                termination = arg_type.findtext(".//x:TerminationChar", namespaces=NS)
                if termination is None:
                    raise ValueError(f"Expected terminated string in {name}")
                terminator = bytes.fromhex(termination)
                if len(terminator) != 1:
                    raise ValueError(
                        f"Expected single-byte string terminator in {name}"
                    )
                specs.append(Argument(arg_name, None, terminator=terminator))
            else:
                raise ValueError(f"Unsupported argument type in {name}: {arg_name}")
        if any(other.wire_name == wire for other in commands):
            raise ValueError(f"Ambiguous UDP command identifier: {wire!r}")
        commands.append(Command(name, wire, tuple(specs)))
    if not commands:
        raise ValueError(f"No supported rover commands in {path}")
    return tuple(
        sorted(commands, key=lambda command: len(command.wire_name), reverse=True)
    )


def translate(
    data: bytes, commands: tuple[Command, ...]
) -> tuple[str, dict[str, Value] | list[str]]:
    for command in commands:
        if data.startswith(command.wire_name):
            values = command.decode(data)
            topic = ROUTES[command.name]
            if topic == CAMERA_REQUEST_KEY:
                if command.name.endswith("camera_capture_group"):
                    names = [name.strip() for name in values["camera_names"].split(",")]
                else:
                    names = [values["camera_name"]]
                if not names or any(not name or "/" in name for name in names):
                    raise ValueError("Select camera short names")
                return topic, list(dict.fromkeys(names))
            if topic == "set_arm_joint":
                values = {
                    "joint_name": values["joint_name"],
                    "position_rad": math.radians(values["position_deg"]),
                }
            if topic == "set_arm_tip":
                values = {
                    "frame_id": (
                        "l_Gripper_Base" if values["frame_id"] == "End-Effector"
                        else values["frame_id"]
                    ),
                    "position_m": [values[key] for key in ("x_m", "y_m", "z_m")],
                    "orientation_rpy_rad": [
                        math.radians(values[key])
                        for key in ("roll_deg", "pitch_deg", "yaw_deg")
                    ],
                }
            if topic == "drive_turn":
                # YAMCS arguments are deg and deg/s; the motion stack uses radians.
                values = {key: math.radians(value) for key, value in values.items()}
            if topic in ("drive_straight", "drive_turn"):
                magnitude = "distance" if topic == "drive_straight" else "angle"
                speed = "linear_velocity" if topic == "drive_straight" else "angular_velocity"
                if values[magnitude] < 0:
                    values[speed] = -abs(values[speed])
                values[magnitude] = abs(values[magnitude])
            if topic in ("solar_panel", "manipulator", "legs"):
                values = {"action": values["deployment"]}
            return topic, values
    raise ValueError("Unsupported or unknown YAMCS command")


@afor.scoped
async def forward_commands(mdb: Path, host: str, port: int, base_keyexpr: str) -> None:
    commands = load_commands(mdb)
    receiver = CommandReceiver(host, port)
    session = current_session()
    logger.info("Listening for YAMCS commands on UDP %s:%d", host, port)
    forwarded: set[str] = set()
    while True:
        data = await receiver.receive()
        try:
            suffix, values = translate(data, commands)
        except ValueError as exc:
            logger.warning("Rejected YAMCS command: %s", exc)
            continue
        key = suffix if suffix == CAMERA_REQUEST_KEY else f"{base_keyexpr}/{suffix}"
        session.put(key, msgspec.json.encode(values))
        if suffix not in forwarded:
            logger.info("Forwarding YAMCS commands to %s", key)
            forwarded.add(suffix)
