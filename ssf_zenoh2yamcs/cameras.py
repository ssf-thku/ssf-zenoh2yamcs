"""Forward OmniLRS ndarray camera frames to the YAMCS image bucket."""

import logging
from io import BytesIO

import asyncio_for_robotics as afor
import msgspec
from asyncio_for_robotics.zenoh import Sub
from PIL import Image
from yamcs.client.core.exceptions import YamcsError

from .yamcs import ParameterPublisher

logger = logging.getLogger(__name__)
IMAGE_PREFIX = "cameras/one_shot"


class WireNDArray(msgspec.Struct, array_like=True):
    dtype: str
    shape: tuple[int, ...]
    data: bytes


_DECODER = msgspec.msgpack.Decoder(WireNDArray)


def image_png(payload: bytes) -> bytes:
    frame = _DECODER.decode(payload)
    if frame.dtype != "uint8" or len(frame.shape) != 3 or frame.shape[2] not in (3, 4):
        raise ValueError("Expected uint8 RGB or RGBA camera image")
    height, width, channels = frame.shape
    if height <= 0 or width <= 0 or len(frame.data) != height * width * channels:
        raise ValueError("Camera image shape and byte count do not match")
    image = Image.frombytes(
        "RGB" if channels == 3 else "RGBA", (width, height), frame.data
    )
    with BytesIO() as output:
        image.save(output, format="PNG")
        return output.getvalue()


@afor.scoped
async def forward_camera_images(publisher: ParameterPublisher) -> None:
    sub = Sub(f"{IMAGE_PREFIX}/*")
    logger.info("Subscribed to %s/*", IMAGE_PREFIX)
    published: set[str] = set()
    async for sample in sub.listen_reliable(queue_size=0):
        camera_name = str(sample.key_expr).rsplit("/", 1)[1]
        try:
            png = image_png(sample.payload.to_bytes())
        except (msgspec.DecodeError, ValueError) as exc:
            logger.warning("Rejected image from %s: %s", camera_name, exc)
            continue
        try:
            publisher.publish_image(camera_name, png)
        except (YamcsError, OSError) as exc:
            message = publisher.image_error(exc)
            logger.error("%s", message)
            try:
                publisher.publish({"/Rover/camera/images_oncommand/error": message})
            except (YamcsError, OSError):
                pass
            continue
        if camera_name not in published:
            logger.info("Forwarding camera %s to YAMCS", camera_name)
            published.add(camera_name)
