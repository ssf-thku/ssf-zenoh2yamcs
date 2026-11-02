import asyncio
from datetime import UTC, datetime
from unittest.mock import Mock

import asyncio_for_robotics as afor
import msgspec
import pytest
import zenoh
from asyncio_for_robotics.zenoh import Sub, auto_context
from test_telemetry import imu_payload
from yamcs.client import YamcsClient
from yamcs.protobuf.processing import processing_pb2

from ssf_zenoh2yamcs import bridge
from ssf_zenoh2yamcs.telemetry import imu_parameters
from ssf_zenoh2yamcs.yamcs import ParameterPublisher


def test_publisher_uses_real_sdk_and_scope_closes_client(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = YamcsClient("localhost:8090")
    post = Mock()
    close = Mock()
    monkeypatch.setattr(client.ctx, "post_proto", post)
    monkeypatch.setattr(client, "close", close)
    monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.YamcsClient", lambda address: client)

    async def check() -> None:
        async with afor.Scope():
            publisher = ParameterPublisher()
            publisher.publish(
                imu_parameters(imu_payload()),
                generation_time=datetime(2026, 1, 1, tzinfo=UTC),
            )
            assert close.call_count == 0
        assert close.call_count == 1
        publisher.close()
        assert close.call_count == 1
        with pytest.raises(RuntimeError, match="closed"):
            publisher.publish({"/example": 1})

    asyncio.run(check())
    assert post.call_args.args == (
        "/processors/ssf-tohoku/realtime/parameters:batchSet",
    )
    request = processing_pb2.BatchSetParameterValuesRequest.FromString(
        post.call_args.kwargs["data"]
    )
    assert len(request.request) == 3
    acceleration = request.request[0]
    assert acceleration.id.name == "/Rover/cdh/imu_accelerometer"
    assert list(acceleration.value.aggregateValue.name) == ["ax", "ay", "az"]
    assert acceleration.generationTime.ToDatetime(tzinfo=UTC) == datetime(
        2026, 1, 1, tzinfo=UTC
    )
    client.ctx.close()


def test_real_zenoh_to_yamcs_sdk_and_cancellation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Real Zenoh in-process routing, without external sockets or services.
    config = zenoh.Config.from_json5("""{
        mode: "peer", listen: {endpoints: []},
        scouting: {multicast: {enabled: false}, gossip: {enabled: false}}
    }""")
    client = YamcsClient("localhost:8090")
    monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.YamcsClient", lambda address: client)

    async def check() -> None:
        ready = asyncio.Event()
        delivered = asyncio.Event()
        subscriptions: list[Sub] = []
        requests: list[processing_pb2.BatchSetParameterValuesRequest] = []

        def subscribe(topic: str) -> Sub:
            sub = Sub(topic)
            subscriptions.append(sub)
            ready.set()
            return sub

        def post(url: str, *, data: bytes) -> None:
            requests.append(
                processing_pb2.BatchSetParameterValuesRequest.FromString(data)
            )
            delivered.set()

        monkeypatch.setattr(bridge, "Sub", subscribe)
        monkeypatch.setattr(client.ctx, "post_proto", post)
        with zenoh.open(config) as session, auto_context(session):
            async with afor.Scope() as scope:
                sink = ParameterPublisher()
                task = scope.task_group.create_task(
                    bridge.forward("test/imu", imu_parameters, sink, 100)
                )
                async with asyncio.timeout(3):
                    await ready.wait()
                    # A malformed sample must not kill the forwarding loop.
                    session.put("test/imu", b"not JSON")
                    await asyncio.sleep(0.02)
                    session.put("test/imu", imu_payload())
                    while not delivered.is_set():
                        await asyncio.sleep(0.01)
                assert len(requests[0].request) == 3
                task.cancel()
            assert subscriptions[0]._closed.is_set()
            assert not session.is_closed()  # borrowed session remains caller-owned

    asyncio.run(check())


def test_yamcs_failure_propagates(monkeypatch: pytest.MonkeyPatch) -> None:
    client = YamcsClient("localhost:8090")
    monkeypatch.setattr(
        client.ctx, "post_proto", Mock(side_effect=OSError("server unavailable"))
    )
    monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.YamcsClient", lambda address: client)
    publisher = ParameterPublisher(scope=None)
    try:
        with pytest.raises(OSError, match="server unavailable"):
            publisher.publish({"/example": 1})
    finally:
        publisher.close()


def test_run_stays_alive_forwards_both_topics_and_cleans_up(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = zenoh.Config.from_json5("""{
        mode: "peer", listen: {endpoints: []},
        scouting: {multicast: {enabled: false}, gossip: {enabled: false}}
    }""")
    client = YamcsClient("localhost:8090")
    close = Mock(wraps=client.close)
    monkeypatch.setattr(client, "close", close)
    monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.YamcsClient", lambda address: client)

    async def check() -> None:
        ready = asyncio.Event()
        delivered = asyncio.Event()
        subscriptions: list[Sub] = []
        parameters: set[str] = set()

        def subscribe(topic: str) -> Sub:
            sub = Sub(topic)
            subscriptions.append(sub)
            if len(subscriptions) == 2:
                ready.set()
            return sub

        def post(url: str, *, data: bytes) -> None:
            request = processing_pb2.BatchSetParameterValuesRequest.FromString(data)
            parameters.update(item.id.name for item in request.request)
            if len(parameters) == 4:
                delivered.set()

        monkeypatch.setattr(bridge, "Sub", subscribe)
        monkeypatch.setattr(client.ctx, "post_proto", post)
        with zenoh.open(config) as session:
            monkeypatch.setattr(bridge, "auto_context", lambda: auto_context(session))
            task = asyncio.create_task(
                bridge.run(bridge.Config(hz=100, commands=False))
            )
            try:
                async with asyncio.timeout(3):
                    await ready.wait()
                    assert not task.done()
                    session.put("OmniLRS/pioneer/imu", imu_payload())
                    session.put(
                        "OmniLRS/pioneer/joint_telemetry",
                        msgspec.msgpack.encode(
                            [
                                123,
                                0,
                                [["wheel_1", 0.0], ["joint_1", 1.0]],
                            ]
                        ),
                    )
                    while not delivered.is_set():
                        await asyncio.sleep(0.01)
                assert parameters == {
                    "/Rover/cdh/imu_accelerometer",
                    "/Rover/cdh/imu_gyroscope",
                    "/Rover/cdh/imu_orientation",
                    "/Rover/motor/joint_telemetry",
                }
                assert not task.done()
            finally:
                task.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await task
            assert all(sub._closed.is_set() for sub in subscriptions)
            assert close.call_count == 1
            assert not session.is_closed()

    asyncio.run(check())
