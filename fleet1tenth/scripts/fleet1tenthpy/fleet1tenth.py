#! /usr/bin/env python3
import argparse


class Fleet1tenth:
    def __init__(self,config_file_path=None, parser_=[],args=None):
        # build parser for command line arguments
        parser=argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                        parents=[parser_])

        parser.add_argument("--sim", help="Run scripts in simulation.", action="store_true")
        if isinstance(args, str):
            args = args.split()
        args, _ = parser.parse_known_args(args)

        if args.sim:
            from .simulation import Fleet
            self.fleet=Fleet(config_file_path)
        else:
            from . import Fleet
            self.fleet=Fleet(config_file_path)



        

# TODO: crazyswarm args, check to add!
"""

    parser.add_argument("--sim", help="Run using simulation.", action="store_true")

    group = parser.add_argument_group("Simulation-only", "")
    group.add_argument("--vis", help="Visualization backend.", choices=['mpl', 'vispy', 'null'], default="mpl")
    group.add_argument("--dt", help="Duration of seconds between rendered visualization frames.", type=float, default=0.1)
    group.add_argument("--writecsv", help="Enable CSV output.", action="store_true")
    group.add_argument("--disturbance", help="Simulate Gaussian-distributed disturbance when using cmdVelocityWorld.", type=float, default=0.0)
    group.add_argument("--maxvel", help="Limit simulated velocity (meters/sec).", type=float, default=np.inf)
    group.add_argument("--video", help="Video output path.", type=str)
"""