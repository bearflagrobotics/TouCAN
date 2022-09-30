"""
Copyright 2022 Bear Flag Robotics

    follow_can_interface_test.py

        Test CAN Interface (via serial), this is the follow node

        Author: Austin Chun
        Date:   Sep 2022

"""

# pylint: disable=C0103
from time import time, sleep
import logging
import argparse

from utils import open_serial_port_blocking
from can_utils import CanMsg

from CanInterface import CanInterface

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
        from CustomLogger import CustomLogger
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

    # datetime_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # datalogname = datetime_str + ".asc"

    # Establish serial connection
    ser = open_serial_port_blocking(port_path=port_path)

    # Initialize SerialInterface object (for threaded serial read process)
    can_int = CanInterface(ser, logger=logger, datalogname=None)

    msg_1 = CanMsg(0x1A4321FF, [0x44, 0x33, 0x22, 0x11, 0x43, 0x21, 0x43, 0x21], bus=0)
    msg_2 = CanMsg(0x1BABCDFF, [0xAA, 0xBB, 0xCC, 0xDD, 0xAB, 0xCD, 0xAB, 0xCD], bus=1)

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
                can_int.write_can_msg(msg_1)
                logger.debug("Transmit: %s", msg_1.log_str)
            if time() - print_t2 > 2.0:
                print_t2 = time()
                can_int.write_can_msg(msg_2)
                logger.debug("Transmit: %s", msg_2.log_str)


    except KeyboardInterrupt:
        logger.warning("User exited w/ Ctrl+C.")

# ==================================================================================================

if __name__ == '__main__':
    main()
