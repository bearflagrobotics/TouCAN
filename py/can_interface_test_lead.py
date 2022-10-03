"""
Copyright 2022 Bear Flag Robotics

    lead_can_interface_test.py

        Test CAN Interface (via serial), this is the lead node

        Author: Austin Chun
        Date:   Sep 2022

"""

# pylint: disable=C0103
from time import time, sleep
import logging
import argparse

from src.Utils import open_serial_port_blocking
from src.UtilsCan import CanMsg

from src.CanInterface import CanInterface

# ==================================================================================================

def main():
    """ TODO """

    ###############################################################################################
    ## Parse CLI input
    # Argparser
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', '-p',
                        help="Path to the device (eg. /dev/ttyACM0 (Unix), or COM7 (Windows)",
                        type=str, default=None)
    parser.add_argument('--verbose', '-v', action=argparse.BooleanOptionalAction)
    parser.add_argument('--color', '-c', action=argparse.BooleanOptionalAction)
    parser.add_argument('--debug', '-d', action=argparse.BooleanOptionalAction)
    args = parser.parse_args()
    # Condition Args
    port_path = args.path
    verbose = args.verbose
    color = args.color
    logger_lvl = logging.DEBUG if args.debug else logging.INFO
    ###############################################################################################

    ## Setup the logger
    try:
        from src.CustomLogger import CustomLogger
        # Init Logger
        logger = CustomLogger(
            "CanInterface.py",
            level=logger_lvl,
            verbose=verbose,
            color=color
        )
    except ModuleNotFoundError as e:
        print(f"  Failed to load CustomLogger: {e}")
        logger = logging.getLogger()
    ###############################################################################################

    # # Optional logging of raw can data to
    # datetime_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # datalogname = datetime_str + ".asc"

    # Establish serial connection
    ser = open_serial_port_blocking(port_path=port_path)

    # Initialize SerialInterface object (for threaded serial read process)
    can_int = CanInterface(ser, logger=logger, datalogname=None)

    msg_0123 = CanMsg(0x1A0123FF, [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77], bus=0)
    msg_9876 = CanMsg(0x1B9876FF, [0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22], bus=1)

    try:
        print_t1 = time()
        print_t2 = time()
        while True:
            ## Read from CanInterface, and print
            rx_msg = can_int.get_rx_msg()
            if rx_msg is None: # Deal with empty queue
                sleep(0.001)
            else:
                # logger.info("RX: %s", rx_msg.info_string)
                can_int.print_can_msg_data(rx_msg, verbose=False)

            # Write Data periodically
            if time() - print_t1 > 1.0:
                print_t1 = time()
                can_int.write_can_msg(msg_0123)
                logger.debug("Transmit: %s", msg_0123.log_str())
            if time() - print_t2 > 2.0:
                print_t2 = time()
                can_int.write_can_msg(msg_9876)
                logger.debug("Transmit: %s", msg_9876.log_str())


    except KeyboardInterrupt:
        logger.warning("User exited w/ Ctrl+C.")

# ==================================================================================================

if __name__ == '__main__':
    main()
