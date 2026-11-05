"""Small afor-compatible interfaces for YAMCS telemetry and UDP commands."""

import asyncio
import socket
from collections.abc import Mapping
from datetime import datetime
from io import BytesIO
from time import time_ns
from typing import TypeAlias

import asyncio_for_robotics as afor
from yamcs.client import YamcsClient
from yamcs.client.core.exceptions import YamcsError
from yamcs.client.tmtc.client import ProcessorClient

Value: TypeAlias = bool | int | float | str | list["Value"] | dict[str, "Value"]


class ParameterPublisher:
    """Own a YAMCS client, with automatic cleanup in the current afor scope.

    Publishing is synchronous, just like Zenoh's ``put``. Failures propagate
    to the caller. Outside a scope, the caller must call ``close()``.
    """

    def __init__(
        self,
        address: str = "localhost:8090",
        instance: str = "ssf-tohoku",
        processor: str = "realtime",
        *,
        scope: afor.Scope | None = afor.AUTO_SCOPE,
        public_url: str | None = None,
    ) -> None:
        self._closed = False
        self._client = YamcsClient(address)
        self._address = address
        self._public_url = (public_url or f"http://{address}").rstrip("/")
        self._image_error_message = "Image upload failed. Check Yamcs storage."
        self._image_count = 0
        self._camera_image_counts: dict[str, int] = {}
        self._processor: ProcessorClient = self._client.get_processor(
            instance, processor
        )
        if scope is afor.AUTO_SCOPE:
            scope = afor.Scope.current(default=None)
        if scope is not None:
            scope.exit_stack.callback(self.close)

    def publish(
        self,
        values: Mapping[str, Value],
        *,
        generation_time: datetime | None = None,
    ) -> None:
        """Write native Python values to existing LOCAL parameters in the MDB."""
        if self._closed:
            raise RuntimeError("YAMCS publisher is closed")
        if values:
            self._processor.set_parameter_values(
                values, generation_time=generation_time
            )

    def image_error(self, error: Exception) -> str:
        try:
            bucket = self._client.get_storage_client().get_bucket("images_oncommand")
            full = bucket.size >= bucket.max_size * .9 or bucket.object_count >= bucket.max_object_count
            status = "Image bucket full" if full else f"Image upload failed: {error}"
            self._image_error_message = (
                f'{status} at "{bucket.directory}" ({bucket.max_size / 1048576:g} MiB Yamcs limit). '
                f'Delete its contents with `rm -r -- {bucket.directory}/*` or move them to continue.'
            )
        except (YamcsError, OSError):
            pass
        return self._image_error_message

    def publish_image(self, camera_name: str, png: bytes) -> None:
        """Store the snapshot and latest copy before publishing camera metadata."""
        if self._closed:
            raise RuntimeError("YAMCS publisher is closed")
        bucket = "images_oncommand"
        filename = f"{camera_name}_{time_ns()}.png"
        latest_filename = f"{camera_name}_latest.png"
        storage = self._client.get_storage_client()
        for object_name in (filename, latest_filename):
            with BytesIO(png) as image:
                storage.upload_object(
                    bucket, object_name, image, content_type="image/png"
                )
        url_storage = f"/storage/buckets/{bucket}/objects/{filename}"
        latest_url_storage = f"/storage/buckets/{bucket}/objects/{latest_filename}"
        base = "/Rover/camera/images_oncommand"
        metadata: dict[str, Value] = {
            "number": self._image_count + 1,
            "name": filename,
            "url_storage": url_storage,
            "url_full": f"http://{self._address}/api{url_storage}",
            "url_full_nginx": f"{self._public_url}/api{url_storage}",
        }
        # Preserve the existing shared preview, and give each camera its own
        # archive/latest URLs and independent capture counter.
        values = {f"{base}/{field}": value for field, value in metadata.items()}
        camera_count = self._camera_image_counts.get(camera_name, 0) + 1
        metadata.update(
            {
                "number": camera_count,
                "latest_name": latest_filename,
                "latest_url_storage": latest_url_storage,
                "latest_url_full": f"http://{self._address}/api{latest_url_storage}",
                "latest_url_full_nginx": f"{self._public_url}/api{latest_url_storage}",
            }
        )
        values.update(
            {
                f"{base}/{camera_name}/{field}": value
                for field, value in metadata.items()
            }
        )
        values[f"{base}/error"] = ""
        self.publish(values)
        self._image_count += 1
        self._camera_image_counts[camera_name] = camera_count

    def close(self) -> None:
        """Close the owned client once."""
        if not self._closed:
            self._closed = True
            self._client.close()


class CommandReceiver:
    """Own the UDP socket used by YAMCS's UdpTcDataLink.

    Inherits the current afor scope for cleanup. Outside a scope, call close().
    Binding is exclusive: another command receiver on this port is an error.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 10025,
        *,
        scope: afor.Scope | None = afor.AUTO_SCOPE,
    ) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind((host, port))
            self._socket.setblocking(False)
        except BaseException:
            self._socket.close()
            raise
        if scope is afor.AUTO_SCOPE:
            scope = afor.Scope.current(default=None)
        if scope is not None:
            scope.exit_stack.callback(self.close)

    async def receive(self) -> bytes:
        """Wait for one complete UDP command datagram."""
        data, _sender = await asyncio.get_running_loop().sock_recvfrom(
            self._socket, 65536
        )
        return data

    def close(self) -> None:
        """Close the socket; safe to call repeatedly."""
        self._socket.close()
