import argparse
import asyncio
import logging
from contextlib import suppress
from pathlib import Path

from .bridge import Config, run


def main() -> None:
    defaults = Config()
    parser = argparse.ArgumentParser(
        description="Bridge SSF Zenoh telemetry and YAMCS rover commands"
    )
    parser.add_argument("--address", default=defaults.address)
    parser.add_argument("--instance", default=defaults.instance)
    parser.add_argument("--processor", default=defaults.processor)
    parser.add_argument("--base-keyexpr", default=defaults.base_keyexpr)
    parser.add_argument("--imu-prefix", default=defaults.imu_prefix)
    parser.add_argument("--hz", type=float, default=defaults.hz)
    parser.add_argument("--no-imu", action="store_true")
    parser.add_argument("--no-encoders", action="store_true")
    parser.add_argument("--no-commands", action="store_true")
    parser.add_argument("--no-cameras", action="store_true")
    parser.add_argument("--public-url", help="Public YAMCS URL used in image links")
    parser.add_argument("--command-host", default=defaults.command_host)
    parser.add_argument("--command-port", type=int, default=defaults.command_port)
    parser.add_argument("--mdb", type=Path, default=defaults.mdb)
    args = parser.parse_args()
    config = Config(
        address=args.address,
        instance=args.instance,
        processor=args.processor,
        base_keyexpr=args.base_keyexpr.rstrip("/"),
        imu_prefix=args.imu_prefix.rstrip("/"),
        hz=args.hz,
        imu=not args.no_imu,
        encoders=not args.no_encoders,
        commands=not args.no_commands,
        cameras=not args.no_cameras,
        public_url=args.public_url,
        command_host=args.command_host,
        command_port=args.command_port,
        mdb=args.mdb,
    )
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    with suppress(KeyboardInterrupt):
        asyncio.run(run(config))


if __name__ == "__main__":
    main()
