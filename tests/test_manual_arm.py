"""Contracts shared by the existing YAMCS MDB and motion-stack main."""
import asyncio
import math
import struct
from pathlib import Path

import asyncio_for_robotics as afor
import msgspec
import pytest
import zenoh
from asyncio_for_robotics.zenoh import Sub, auto_context

from ssf_zenoh2yamcs import bridge
from ssf_zenoh2yamcs.commands import load_commands, translate
from ssf_zenoh2yamcs.telemetry import command_status_parameters, joint_parameters

MDB = Path(__file__).parent / 'fixtures/rover_commands.xml'


@pytest.mark.parametrize('joint', range(6))
def test_joint_targets_are_absolute_radians(joint):
    topic, values = translate(b'set_arm_joint' + struct.pack('>Bf', joint, -90), load_commands(MDB))
    assert topic == 'set_arm_joint'
    assert values == {'joint_name': f'ajoint_{joint}', 'position_rad': -math.pi / 2}


@pytest.mark.parametrize(('frame', 'expected'), [(0, 'base_link'), (1, 'l_Gripper_Base')])
def test_ik_preserves_relative_translation_and_converts_rpy(frame, expected):
    topic, values = translate(b'set_arm_tip' + struct.pack('>B6f', frame, .125, -.25, .5, 90, -45, 180), load_commands(MDB))
    assert topic == 'set_arm_tip'
    assert values['frame_id'] == expected
    assert values['position_m'] == [.125, -.25, .5]
    assert values['orientation_rpy_rad'] == pytest.approx([math.pi / 2, -math.pi / 4, math.pi])


@pytest.mark.parametrize('packet', [
    b'set_arm_joint' + struct.pack('>Bf', 6, 0),
    b'set_arm_joint' + struct.pack('>Bf', 0, float('nan')),
    b'set_arm_tip' + struct.pack('>B6f', 2, 0, 0, 0, 0, 0, 0),
    b'set_arm_tip' + struct.pack('>B6f', 0, 0, 0, float('inf'), 0, 0, 0),
    b'set_arm_tip' + struct.pack('>B5f', 0, 0, 0, 0, 0, 0),
])
def test_malformed_manual_commands_are_rejected(packet):
    with pytest.raises(ValueError):
        translate(packet, load_commands(MDB))


def test_joint_readback_converts_radians_to_degrees_without_losing_other_joints():
    states = [[f'ajoint_{i}', math.pi / 2, None, None] for i in reversed(range(6))] + [['wheel_1', 1.0, None, None]]
    values = joint_parameters(msgspec.msgpack.encode([123, 0, states]))
    assert values['/Rover/motor/arm_joint_positions'] == [90.0] * 6
    assert msgspec.json.decode(values['/Rover/motor/joint_telemetry']) == [123, 0, states]
    # Missing joints must not become fabricated zero readings.
    assert '/Rover/motor/arm_joint_positions' not in joint_parameters(msgspec.msgpack.encode([124, 0, states[1:]]))


@pytest.mark.parametrize('status', ['STARTED', 'COMPLETED', 'FAILED', 'CANCELLED'])
def test_command_feedback_keeps_request_id_and_error(status):
    payload = {'request_id': 'motion-id', 'command': 'set_arm_tip', 'status': status, 'error': 'unreachable' if status == 'FAILED' else None}
    values = command_status_parameters(msgspec.json.encode(payload))
    assert msgspec.json.decode(values['/Rover/motor/command_status']) == payload


def test_invalid_feedback_is_rejected():
    with pytest.raises(ValueError):
        command_status_parameters(b'{"request_id":1,"command":"set_arm_tip","status":"UNKNOWN"}')


def test_feedback_burst_preserves_started_and_terminal_events(monkeypatch):
    config = zenoh.Config.from_json5('''{
        mode: "peer", listen: {endpoints: []},
        scouting: {multicast: {enabled: false}, gossip: {enabled: false}}
    }''')

    async def check():
        ready = asyncio.Event()
        received = []
        subscriptions = []

        class Sink:
            def publish(self, values):
                received.append(msgspec.json.decode(values['/Rover/motor/command_status']))

        def subscribe(topic):
            sub = Sub(topic)
            subscriptions.append(sub)
            ready.set()
            return sub

        monkeypatch.setattr(bridge, 'Sub', subscribe)
        with zenoh.open(config) as session, auto_context(session):
            async with afor.Scope() as scope:
                task = scope.task_group.create_task(bridge.forward('test/command_status', command_status_parameters, Sink(), None))
                async with asyncio.timeout(3):
                    await ready.wait()
                    session.put('test/command_status', b'not-json')
                    for request_id in range(4):
                        for status in ('STARTED', 'COMPLETED'):
                            session.put('test/command_status', msgspec.json.encode({'request_id': request_id, 'command': 'set_arm_tip', 'status': status}))
                    while len(received) < 8:
                        await asyncio.sleep(.01)
                task.cancel()
            assert all(sub._closed.is_set() for sub in subscriptions)
        assert [(v['request_id'], v['status']) for v in received] == [(i, s) for i in range(4) for s in ('STARTED', 'COMPLETED')]

    asyncio.run(check())
