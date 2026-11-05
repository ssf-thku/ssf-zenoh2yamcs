"""Bidirectional SSF Zenoh/YAMCS bridge."""

import asyncio
import logging
import math
from collections.abc import Callable
from dataclasses import dataclass
from functools import partial
from pathlib import Path

import asyncio_for_robotics as afor
import msgspec
from asyncio_for_robotics.zenoh import Sub, auto_context

from .cameras import forward_camera_images
from .commands import DEFAULT_MDB, forward_commands
from .telemetry import command_status_parameters, imu_parameters, joint_parameters
from .yamcs import ParameterPublisher, Value

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Config:
    address: str = "localhost:8090"
    instance: str = "ssf-tohoku"
    processor: str = "realtime"
    base_keyexpr: str = "OmniLRS/pioneer"
    imu_prefix: str = "/Rover/cdh"
    hz: float = 5.0
    imu: bool = True
    encoders: bool = True
    commands: bool = True
    cameras: bool = True
    command_feedback: bool = True
    public_url: str | None = None
    command_host: str = "127.0.0.1"
    command_port: int = 10025
    mdb: Path = DEFAULT_MDB


@afor.scoped
async def forward(
    topic: str,
    convert: Callable[[bytes], dict[str, Value]],
    publisher: ParameterPublisher,
    hz: float | None,
) -> None:
    sub = Sub(topic)
    # Command status is event data: preserve STARTED and terminal messages.
    samples = sub.listen_reliable() if hz is None else sub.listen()
    logger.info("Subscribed to %s", topic)
    published = False
    try:
        async for sample in samples:
            try:
                values = convert(sample.payload.to_bytes())
            except (msgspec.DecodeError, ValueError) as exc:
                logger.warning("Skipping invalid telemetry on %s: %s", topic, exc)
            else:
                if values:
                    publisher.publish(values)
                    if not published:
                        logger.info("Forwarding telemetry from %s to YAMCS", topic)
                        published = True
            # The subscriber retains only the latest message during this wait.
            # Avoid queuing 50 Hz feedback behind synchronous YAMCS requests.
            if hz is not None:
                await asyncio.sleep(1.0 / hz)
    finally:
        await samples.aclose()


async def run(config: Config) -> None:
    if not math.isfinite(config.hz) or config.hz <= 0:
        raise ValueError("hz must be finite and positive")
    if (
        not config.imu
        and not config.encoders
        and not config.commands
        and not config.cameras
        and not config.command_feedback
    ):
        raise ValueError("Enable at least one bridge direction")
    # The scope closes subscribers and cancels forwarding tasks before the
    # shared Zenoh session closes. ParameterPublisher inherits the scope.
    with auto_context():
        async with afor.Scope() as scope:
            if config.commands:
                scope.task_group.create_task(
                    forward_commands(
                        config.mdb,
                        config.command_host,
                        config.command_port,
                        config.base_keyexpr,
                    )
                )
            if config.imu or config.encoders or config.cameras or config.command_feedback:
                publisher = ParameterPublisher(
                    config.address,
                    config.instance,
                    config.processor,
                    public_url=config.public_url,
                )
            if config.command_feedback:
                scope.task_group.create_task(
                    forward(
                        f"{config.base_keyexpr}/command_status",
                        command_status_parameters,
                        publisher,
                        None,
                    )
                )
            if config.cameras:
                scope.task_group.create_task(forward_camera_images(publisher))
            if config.imu:
                scope.task_group.create_task(
                    forward(
                        f"{config.base_keyexpr}/imu",
                        partial(imu_parameters, prefix=config.imu_prefix),
                        publisher,
                        config.hz,
                    )
                )
            if config.encoders:
                scope.task_group.create_task(
                    forward(
                        f"{config.base_keyexpr}/joint_telemetry",
                        joint_parameters,
                        publisher,
                        config.hz,
                    )
                )
            # Leaving an afor scope cancels its tasks. Keep the application
            # alive until cancellation or a forwarding task fails.
            await asyncio.Event().wait()
