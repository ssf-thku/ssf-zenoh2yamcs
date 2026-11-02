from io import BytesIO
from unittest.mock import Mock

import msgspec
import pytest
from PIL import Image
from test_commands import MDB
from yamcs.client import YamcsClient

from ssf_zenoh2yamcs.cameras import WireNDArray, image_png
from ssf_zenoh2yamcs.commands import load_commands, translate
from ssf_zenoh2yamcs.yamcs import ParameterPublisher


@pytest.mark.parametrize("channels", [3, 4])
def test_omnilrs_ndarray_becomes_lossless_png(channels: int) -> None:
    pixels = bytes(range(2 * 3 * channels))
    payload = msgspec.msgpack.encode(WireNDArray("uint8", (2, 3, channels), pixels))
    with Image.open(BytesIO(image_png(payload))) as image:
        assert image.size == (3, 2)
        assert image.tobytes() == pixels


@pytest.mark.parametrize(
    "frame",
    [
        ["float32", [2, 3, 3], bytes(72)],
        ["uint8", [2, 3], bytes(6)],
        ["uint8", [2, 3, 1], bytes(6)],
        ["uint8", [2, 3, 3], bytes(17)],
        ["uint8", [0, 3, 3], b""],
    ],
)
def test_malformed_frames_are_rejected(frame: list) -> None:
    with pytest.raises(ValueError):
        image_png(msgspec.msgpack.encode(frame))


def test_camera_single_dropdown_and_group_commands() -> None:
    commands = load_commands(MDB)
    assert translate(b"camera_capture_high\x06", commands) == (
        "cameras/capture",
        ["nav_left"],
    )
    assert translate(b"camera_capture_group" + b"nav_left\0", commands) == (
        "cameras/capture",
        ["nav_left"],
    )
    assert translate(
        b"camera_capture_group" + b"nav_left, nav_right,nav_left\0", commands
    ) == (
        "cameras/capture",
        ["nav_left", "nav_right"],
    )
    with pytest.raises(ValueError):
        translate(b"camera_capture_group" + b"nav_left\0trailing", commands)
    with pytest.raises(ValueError):
        translate(b"camera_capture_group" + b"nav_left", commands)


def test_upload_completes_before_metadata_and_failure_does_not_publish(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = YamcsClient("localhost:8090")
    monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.YamcsClient", lambda address: client)
    calls: list[str] = []

    def upload(
        bucket: str, filename: str, image: BytesIO, *, content_type: str
    ) -> None:
        assert bucket == "images_oncommand"
        assert filename.startswith("nav_left_")
        assert content_type == "image/png"
        assert image.read() == b"png"
        calls.append("upload")

    storage = Mock()
    storage.upload_object.side_effect = upload
    monkeypatch.setattr(client, "get_storage_client", lambda: storage)
    publisher = ParameterPublisher(scope=None, public_url="https://example.test/yamcs")
    metadata: list[dict] = []

    def publish(values: dict) -> None:
        calls.append("metadata")
        metadata.append(values)

    monkeypatch.setattr(publisher, "publish", publish)
    try:
        publisher.publish_image("nav_left", b"png")
        assert calls == ["upload", "upload", "metadata"]
        assert metadata[0]["/Rover/camera/images_oncommand/number"] == 1
        camera_base = "/Rover/camera/images_oncommand/nav_left"
        assert metadata[0][f"{camera_base}/number"] == 1
        assert metadata[0][f"{camera_base}/latest_name"] == "nav_left_latest.png"
        assert metadata[0][f"{camera_base}/latest_url_full"] == (
            "http://localhost:8090/api/storage/buckets/images_oncommand/objects/nav_left_latest.png"
        )
        assert metadata[0]["/Rover/camera/images_oncommand/url_full_nginx"].startswith(
            "https://example.test/yamcs/api/"
        )
        storage.upload_object.side_effect = OSError("upload failed")
        with pytest.raises(OSError, match="upload failed"):
            publisher.publish_image("nav_left", b"png")
        assert len(metadata) == 1
        storage.upload_object.side_effect = [None, OSError("latest failed")]
        with pytest.raises(OSError, match="latest failed"):
            publisher.publish_image("nav_left", b"png")
        assert len(metadata) == 1
    finally:
        publisher.close()


def test_latest_replaced_archives_preserved_and_camera_counters_independent(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = YamcsClient("localhost:8090")
    monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.YamcsClient", lambda address: client)
    ticks = iter((1, 2, 3))
    monkeypatch.setattr("ssf_zenoh2yamcs.yamcs.time_ns", lambda: next(ticks))
    objects: dict[str, bytes] = {}
    metadata: list[dict] = []

    def upload(bucket: str, filename: str, file: BytesIO, **kwargs: object) -> None:
        objects[filename] = file.read()

    storage = Mock()
    storage.upload_object.side_effect = upload
    monkeypatch.setattr(client, "get_storage_client", lambda: storage)
    publisher = ParameterPublisher(scope=None)
    monkeypatch.setattr(publisher, "publish", metadata.append)
    try:
        publisher.publish_image("nav_left", b"first")
        publisher.publish_image("nav_right", b"right")
        publisher.publish_image("nav_left", b"newest")
        assert objects == {
            "nav_left_1.png": b"first",
            "nav_left_3.png": b"newest",
            "nav_right_2.png": b"right",
            "nav_left_latest.png": b"newest",
            "nav_right_latest.png": b"right",
        }
        base = "/Rover/camera/images_oncommand"
        assert metadata[0][f"{base}/nav_left/number"] == 1
        assert metadata[1][f"{base}/nav_right/number"] == 1
        assert metadata[2][f"{base}/nav_left/number"] == 2
        assert metadata[2][f"{base}/number"] == 3
    finally:
        publisher.close()
