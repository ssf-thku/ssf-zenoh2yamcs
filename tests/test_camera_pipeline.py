"""Optional cross-repository test; add the runtime src directory to PYTHONPATH."""

import asyncio
from pathlib import Path
from unittest.mock import AsyncMock, Mock

import pytest
import zenoh
from asyncio_for_robotics.zenoh import Sub, auto_context
from PIL import Image
from test_commands import MDB
from yamcs.client import YamcsClient
from yamcs.protobuf.processing import processing_pb2

from ssf_zenoh2yamcs import bridge, cameras


def test_yamcs_command_runtime_group_capture_and_yamcs_images(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    runtime = pytest.importorskip("ssf_isaac_runtime.camera_capture")
    np = pytest.importorskip("numpy")
    config = zenoh.Config.from_json5("""{
        mode: "peer", listen: {endpoints: []},
        scouting: {multicast: {enabled: false}, gossip: {enabled: false}}
    }""")
    client = YamcsClient("localhost:8090")
    monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.YamcsClient", lambda address: client)
    frames = {
        "/rover/nav_left/Camera": np.full((2, 3, 4), 64, dtype=np.uint8),
        "/rover/nav_right/Camera": np.full((2, 3, 4), 128, dtype=np.uint8),
    }
    capture = AsyncMock(return_value=frames)
    monkeypatch.setattr(runtime, "get_camera_images", capture)
    monkeypatch.setattr(runtime, "_scene_cameras", lambda: list(frames))
    monkeypatch.setattr(runtime, "camera_names", lambda paths: {
        "nav_left": "/rover/nav_left/Camera", "nav_right": "/rover/nav_right/Camera",
    })

    async def check() -> None:
        incoming: asyncio.Queue[bytes] = asyncio.Queue()
        ready = asyncio.Event()
        runtime_ready = asyncio.Event()
        images_ready = asyncio.Event()
        done = asyncio.Event()
        sock = Mock()
        uploaded: dict[str, bytes] = {}
        metadata: list[processing_pb2.BatchSetParameterValuesRequest] = []

        def runtime_sub(key: str) -> Sub:
            sub = Sub(key)
            runtime_ready.set()
            return sub

        def images_sub(key: str) -> Sub:
            sub = Sub(key)
            images_ready.set()
            return sub

        async def recvfrom(
            udp_socket: object, size: int
        ) -> tuple[bytes, tuple[str, int]]:
            ready.set()
            return await incoming.get(), ("127.0.0.1", 45000)

        def upload(*, path: str, method: str, files: dict) -> None:
            assert path.startswith("/storage/buckets/images_oncommand/objects/")
            assert method == "post"
            for name, (_filename, file, mime) in files.items():
                assert mime == "image/png"
                uploaded[name] = file.read()

        def post(url: str, *, data: bytes) -> None:
            request = processing_pb2.BatchSetParameterValuesRequest.FromString(data)
            metadata.append(request)
            assert len(uploaded) == 2 * len(
                metadata
            )  # archive and latest precede metadata
            if len(metadata) == 2:
                done.set()

        monkeypatch.setattr(runtime.afor_zenoh, "Sub", runtime_sub)
        monkeypatch.setattr(cameras, "Sub", images_sub)
        monkeypatch.setattr(client.ctx, "request", upload)
        monkeypatch.setattr(client.ctx, "post_proto", post)
        monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.socket.socket", lambda *args: sock)
        monkeypatch.setattr(asyncio.get_running_loop(), "sock_recvfrom", recvfrom)
        with zenoh.open(config) as session, auto_context(session):
            monkeypatch.setattr(bridge, "auto_context", lambda: auto_context(session))
            runtime_task = asyncio.create_task(
                runtime.zenoh_capture_cameras(output_dir=tmp_path)
            )
            bridge_task = asyncio.create_task(
                bridge.run(
                    bridge.Config(
                        imu=False,
                        encoders=False,
                        mdb=MDB,
                    )
                )
            )
            try:
                async with asyncio.timeout(5):
                    await ready.wait()
                    await runtime_ready.wait()
                    await images_ready.wait()
                    incoming.put_nowait(
                        b"camera_capture_group" + b"nav_left,nav_right\0"
                    )
                    # This sandbox blocks sends on asyncio's wakeup socket.
                    # Poll so queued background Zenoh callbacks can run here.
                    while not done.is_set():
                        await asyncio.sleep(0.01)
                capture.assert_awaited_once_with(list(frames), resolution=(1280, 720))
                assert len(uploaded) == 4
                assert len(list(tmp_path.rglob("*.png"))) == 4
                for local in tmp_path.rglob("*.png"):
                    with Image.open(local) as image:
                        assert image.size == (3, 2)
                assert not runtime_task.done()
                assert not bridge_task.done()
            finally:
                for task in (bridge_task, runtime_task):
                    task.cancel()
                for task in (bridge_task, runtime_task):
                    with pytest.raises(asyncio.CancelledError):
                        await task
            assert not session.is_closed()

    asyncio.run(check())
