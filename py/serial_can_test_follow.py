"""
Copyright 2022 Bear Flag Robotics

    follow_serial_can_test.py

        test CAN serial, this is the follow node

        Author: Austin Chun
        Date:   Aug 2022

"""

# pylint: disable=C0103
from time import time, sleep
import logging
import argparse

from src.Utils import open_serial_port_blocking

from src.SerialInterface import SerialInterface


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
            "SerialInterface.py",
            level=logger_lvl,
            verbose=verbose,
            color=color
        )
    except ModuleNotFoundError as e:
        print(f"  Failed to load CustomLogger: {e}")
        logger = logging.getLogger()
    ###############################################################################################


    # Establish serial connection
    ser = open_serial_port_blocking(port_path=port_path)

    # Initialize SerialInterface object (for threaded serial read process)
    ser_int = SerialInterface(ser, logger=logger)

    # Start a thread to read serial data
    ser_int.read_threaded()


    mock_can_data_tx = [
        0x01, # Bus ID
        0x00, 0x03, 0xF0, 0x0C, # ID, LSB first
        # 0xF0, 0xFF, 0x94, 0x90, 0x1A, 0xFF, 0xFF, 0xFF # data
        0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88 # data
    ]

    # Mock: Simulate processing the data from the queue
    try:
        print_last_t = time()
        while True:
            ## Read from SerialInterface, and print
            data_msg = ser_int.get_data()
            if data_msg is None: # Deal with empty queue
                sleep(0.001)
            else:
                logger.info("RX: (%02X %02X) %s", data_msg.type, data_msg.index,
                            bytearr_to_hexstr(data_msg.data))

            # Write mock CAN data
            if time() - print_last_t > 1.0:
                print_last_t = time()
                ser_int.logger.debug("TX: %s", bytearr_to_hexstr(mock_can_data_tx))
                ser_int.write_can_msg(mock_can_data_tx)

    except KeyboardInterrupt:
        logger.warning("User exited w/ Ctrl+C.")

# ==================================================================================================

if __name__ == '__main__':
    main()
