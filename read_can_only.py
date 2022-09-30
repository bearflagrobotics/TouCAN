"""
Copyright 2022 Bear Flag Robotics

    read_can_only.py

        Script to read the CAN data (no writing), and log to file.

        Specify the uC port, and the format to log (CAN Parse .asc, or normal .log).

        Specify verbose/color/debug level for terminal prints

        eg. `py read_can_only.py -p /dev/ttyACM0 -f -c -d`
            Uses uC on port ttyACM0
            Uses CAN format, saves to `<timestamp>.asc`
            Colors output of text to terminal
            Logging level set to Debug (includes echo of all CAN data to terminal as well as data log)

    Author: Austin Chun
    Date:   Oct 2022
"""

# pylint: disable=C0103
from time import time, sleep
import logging
import argparse
from datetime import datetime

from utils import open_serial_port_blocking

from CanInterface import CanInterface


def main():
    """ TODO """

    ###############################################################################################
    ## Parse CLI input
    # Argparser
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', '-p',
                        help="Path to the device (eg. /dev/ttyACM0 (Unix), or COM7 (Windows)",
                        type=str, default=None)
    parser.add_argument('--fmt_canparse', '-f', action=argparse.BooleanOptionalAction,
                        help="Log as CAN Parse format (.asc)")
    parser.add_argument('--verbose', '-v', action=argparse.BooleanOptionalAction)
    parser.add_argument('--color', '-c', action=argparse.BooleanOptionalAction)
    parser.add_argument('--debug', '-d', action=argparse.BooleanOptionalAction)
    args = parser.parse_args()
    # Condition Args
    port_path = args.path
    canparse_fmt = args.fmt_canparse
    verbose = args.verbose
    color = args.color
    logger_lvl = logging.DEBUG if args.debug else logging.INFO

    ###############################################################################################
    ## Setup the logger
    try:
        from CustomLogger import CustomLogger
        # Init Logger
        logger = CustomLogger(
            "read_can_only.py",
            level=logger_lvl,
            color=color
        )
    except ModuleNotFoundError as e:
        print(f"  Failed to load CustomLogger: {e}")
        logger = logging.getLogger()

    ###############################################################################################

    # Optional logging of raw can data to
    datetime_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if canparse_fmt:
        datalogname = datetime_str + ".asc"
    else:
        datalogname = datetime_str + ".log"

    # Establish serial connection
    ser = open_serial_port_blocking(port_path=port_path)

    # Initialize SerialInterface object (for threaded serial read process)
    can_int = CanInterface(ser, logger=logger, datalogname=datalogname)

    try:
        while True:
            ## Read from CanInterface, and print
            rx_msg = can_int.get_rx_msg()
            if rx_msg is None: # Deal with empty queue
                sleep(0.001)
            else:
                can_int.print_can_msg_data(rx_msg, verbose=verbose)
                can_int.log_can_data(rx_msg, canparse_fmt=canparse_fmt)


    except KeyboardInterrupt:
        if canparse_fmt:
            can_int.data_log.info("End TriggerBlock")

        logger.warning("User exited w/ Ctrl+C.")

# ==================================================================================================

if __name__ == '__main__':
    main()
