import asyncio
import math
import struct
from pathlib import Path
from unittest.mock import Mock

import msgspec
import pytest
import zenoh
from asyncio_for_robotics.zenoh import Sub, auto_context

from ssf_zenoh2yamcs import bridge
from ssf_zenoh2yamcs.commands import load_commands, translate
from ssf_zenoh2yamcs.yamcs import CommandReceiver

MDB = Path(__file__).parent / "fixtures/rover_commands.xml"


@pytest.mark.parametrize(
    ("packet", "topic", "expected"),
    [
        (
            b"drive_straight" + struct.pack(">ff", -1, 1),
            "drive_straight",
            {"linear_velocity": -1.0, "distance": 1.0},
        ),
        (
            b"drive_turn" + struct.pack(">ff", -30, 90),
            "drive_turn",
            {"angular_velocity": -math.pi / 6, "angle": math.pi / 2},
        ),
        (b"drive_straight" + struct.pack(">ff", 1, -1), "drive_straight", {"linear_velocity": -1.0, "distance": 1.0}),
        (b"drive_straight" + struct.pack(">ff", -1, -1), "drive_straight", {"linear_velocity": -1.0, "distance": 1.0}),
        (b"drive_turn" + struct.pack(">ff", 30, -90), "drive_turn", {"angular_velocity": -math.pi / 6, "angle": math.pi / 2}),
        (b"drive_turn" + struct.pack(">ff", -30, -90), "drive_turn", {"angular_velocity": -math.pi / 6, "angle": math.pi / 2}),
        (b"deploy_solar_panel\x01", "solar_panel", {"action": "DEPLOY"}),
        (b"deploy_solar_panel\x00", "solar_panel", {"action": "STOW"}),
        (b"deploy_manipulator\x01", "manipulator", {"action": "DEPLOY"}),
        (b"deploy_legs\x01", "legs", {"action": "UNPACK"}),
        (b"deploy_legs\x00", "legs", {"action": "PACK"}),
        *[
            (name.encode(), name, {})
            for name in (
                "cancel_motion",
                "approach_rock",
                "grasp_rock",
                "lift_rock",
                "store_rock",
                "open_gripper",
                "close_gripper",
            )
        ],
    ],
)
def test_commands_match_existing_motion_stack_api(
    packet: bytes,
    topic: str,
    expected: dict,
) -> None:
    suffix, values = translate(packet, load_commands(MDB))
    assert suffix == topic
    assert (
        values == pytest.approx(expected)
        if topic.startswith("drive_")
        else values == expected
    )


@pytest.mark.parametrize(
    "packet",
    [
        b"",
        b"not_a_command",
        b"camera_capture_high",
        b"drive_straight",
        b"drive_straight" + struct.pack(">ff", 1, 1) + b"extra",
        b"drive_straight" + struct.pack(">ff", float("nan"), 1),
        b"drive_turn" + struct.pack(">ff", 1, float("inf")),
        b"deploy_legs\xff",
        b"open_gripper\x00",
    ],
)
def test_invalid_packets_are_rejected(packet: bytes) -> None:
    with pytest.raises(ValueError):
        translate(packet, load_commands(MDB))


def test_receiver_closes_on_bind_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    sock = Mock()
    sock.bind.side_effect = OSError("port already in use")
    monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.socket.socket", lambda *args: sock)
    with pytest.raises(OSError, match="port already in use"):
        CommandReceiver(scope=None)
    sock.close.assert_called_once()


def test_command_only_application_udp_to_real_zenoh(
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    config = zenoh.Config.from_json5("""{
        mode: "peer", listen: {endpoints: []},
        scouting: {multicast: {enabled: false}, gossip: {enabled: false}}
    }""")

    async def check() -> None:
        sock = Mock()
        incoming: asyncio.Queue[bytes] = asyncio.Queue()
        ready = asyncio.Event()

        async def recvfrom(
            udp_socket: object, size: int
        ) -> tuple[bytes, tuple[str, int]]:
            assert udp_socket is sock
            assert size >= 65507  # enough room for a complete IPv4 UDP datagram
            ready.set()
            return await incoming.get(), ("127.0.0.1", 45000)

        monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.socket.socket", lambda *args: sock)
        monkeypatch.setattr(asyncio.get_running_loop(), "sock_recvfrom", recvfrom)
        with zenoh.open(config) as session, auto_context(session):
            monkeypatch.setattr(bridge, "auto_context", lambda: auto_context(session))
            sub = Sub("test/rover/drive_straight", scope=None)
            received: list[zenoh.Sample] = []
            sub.asap_callback.append(received.append)
            samples = sub.listen_reliable()
            task = asyncio.create_task(
                bridge.run(
                    bridge.Config(
                        imu=False,
                        encoders=False,
                        cameras=False,
                        mdb=MDB,
                        base_keyexpr="test/rover",
                    )
                )
            )
            try:
                async with asyncio.timeout(3):
                    await ready.wait()
                    sock.bind.assert_called_once_with(("127.0.0.1", 10025))
                    sock.setblocking.assert_called_once_with(False)
                    # Unknown/malformed traffic must not kill the UDP receiver.
                    incoming.put_nowait(b"camera_capture_high")
                    incoming.put_nowait(b"drive_straight")
                    incoming.put_nowait(b"drive_straight" + struct.pack(">ff", -1, 1))
                    # Poll because the sandbox blocks asyncio's wakeup socket.
                    while len(received) < 1:
                        await asyncio.sleep(0.01)
                    sample = await anext(samples)
                    assert msgspec.json.decode(sample.payload.to_bytes()) == {
                        "linear_velocity": -1.0,
                        "distance": 1.0,
                    }
                    # Each datagram is forwarded, including identical commands.
                    incoming.put_nowait(b"drive_straight" + struct.pack(">ff", -1, 1))
                    while len(received) < 2:
                        await asyncio.sleep(0.01)
                    await anext(samples)
                assert not task.done()
            finally:
                task.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await task
                await samples.aclose()
                sub.close()
            sock.close.assert_called_once()
            assert not session.is_closed()

    asyncio.run(check())
    assert caplog.text.count("Rejected YAMCS command") == 2
