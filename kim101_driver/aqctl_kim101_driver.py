#!/usr/bin/env python3

import argparse
import asyncio
import sys

from sipyco import common_args
from sipyco.pc_rpc import simple_server_loop

from kim101_driver.kim101_driver import KIM101
from kim101_driver.kim101_driver_sim import KIM101Sim

def get_argparser():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", default=None, help="Serial device name")
    parser.add_argument(
        "--simulation",
        action="store_true",
        help="Put the driver in simulation mode, even if " "--device is used.",
    )
    common_args.simple_network_args(parser, 3283)
    common_args.verbosity_args(parser)
    return parser


def main():
    args = get_argparser().parse_args()
    common_args.init_logger_from_args(args)

    if not args.simulation and args.device is None:
        print(
            "You need to specify either --simulation or -d/--device "
            "argument. Use --help for more information."
        )
        sys.exit(1)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        if args.simulation:
            dev = KIM101Sim()
        else:
            dev = KIM101(args.device)
        try:
            simple_server_loop(
                {"kim101": dev},
                common_args.bind_address_from_args(args),
                args.port,
                loop=loop,
            )
        finally:
            dev.close()
    finally:
        loop.close()


if __name__ == "__main__":
    main()