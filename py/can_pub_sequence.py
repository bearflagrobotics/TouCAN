"""
Copyright 2022 Bear Flag Robotics

    can_pub_sequencye.py

        Publish sequence of CAN msgs based on txt file, and log received CAN data

        eg. `py can_pub_sequence.py sample_can_tx_msg_sequence.txt -p /dev/ttyACM0 -c -v -d`

    Author: Austin Chun
    Date:   Oct 2022

"""

# pylint: disable=C0103
from time import time, sleep
import logging
import argparse
import struct

from src.Utils import open_serial_port_blocking, bytearr_to_hexstr
from src.UtilsCan import CanMsg

from src.CanInterface import CanInterface


class CanMsgSequence:
    def __init__(self, startt, freq, dur, bus, ID, data):
        self.startt = startt
        self.freq = freq
        self.dur = dur
        self.msg = CanMsg(ID, data, bus=bus)
        self.last_tx_t = 0

    def __str__(self):
        return f"Time: {self.startt:04.3f}, {self.dur} s @ {self.freq} hz\n" + \
               f"  ID:   0x{self.msg.ID:08X}\n" + \
               f"  Data: {bytearr_to_hexstr(self.msg.data)}"


COMMENT_CHAR = '#'
LINE_ELEMENT_LEN = 6

def can_tx_sequence_file(filename):

    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    can_msg_sequences = []

    # Loop through lines
    for line in lines:
        # Ignore comment rows
        if line[0] == COMMENT_CHAR:
            continue

        arr = line.split()

        if len(arr) != LINE_ELEMENT_LEN:
            print("actualy len: ", len(arr))
            continue

        try:
            can_msg_seq = CanMsgSequence(
                float(arr[0]),    # start time
                float(arr[1]),    # freq
                float(arr[2]),    # duration
                int(arr[3]),      # bus
                int(arr[4], 16),  # ID
                struct.pack('>Q', int(arr[5], 16)), # data
            )
            can_msg_sequences += [can_msg_seq]
        except TypeError as e:
            print(" Type error: ", e)

    return can_msg_sequences

# ==================================================================================================

def main():
    """ TODO """

    ###############################################################################################
    ## Parse CLI input
    # Argparser
    parser = argparse.ArgumentParser()
    parser.add_argument('infile', type=str, help="Path to input file")
    parser.add_argument('--path', '-p',
                        help="Path to the device (eg. /dev/ttyACM0 (Unix), or COM7 (Windows)",
                        type=str, default=None)
    parser.add_argument('--verbose', '-v', action=argparse.BooleanOptionalAction)
    parser.add_argument('--color', '-c', action=argparse.BooleanOptionalAction)
    parser.add_argument('--debug', '-d', action=argparse.BooleanOptionalAction)
    args = parser.parse_args()
    # Condition Args
    infile = args.infile
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

    ## Load sequence file
    can_msg_sequence = can_tx_sequence_file(infile)

    print("can_msg_sequence")
    for msg_seq in can_msg_sequence:
        print(msg_seq)

    # # Optional logging of raw can data to
    # datetime_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # datalogname = datetime_str + ".asc"
    datalogname = None

    # Establish serial connection
    ser = open_serial_port_blocking(port_path=port_path)

    # Initialize SerialInterface object (for threaded serial read process)
    can_int = CanInterface(ser, logger=logger, datalogname=datalogname)

    start_time = time() # Log now. Zero time to this

    active_msgs = [] # Store which msgs are active and should be sent
    window_end_i = 0 # Track how far along sequence list to check
                     # (the next msg in line to be active)

    try:
        while True:
            ## Read from CanInterface, and print
            rx_msg = can_int.get_rx_msg()
            if rx_msg is None: # Deal with empty queue
                sleep(0.001)
            else:
                # logger.info("RX: %s", rx_msg.info_string)
                can_int.print_can_msg_data(rx_msg, verbose=False)


            # Latch current time for single use
            cur_t = time() - start_time

            # Append newly active
            for i in range(window_end_i, len(can_msg_sequence)):
                if can_msg_sequence[i].startt < cur_t:
                    active_msgs += [can_msg_sequence[i]]
                else:
                    window_end_i = i
                    break

            # Pop expired
            expired_inds = []
            for msg_i, msg in enumerate(active_msgs):
                if cur_t > (msg.startt + msg.dur):
                    expired_inds += [msg_i]
            if expired_inds:
                active_msgs = [active_msgs[i] for i in range(len(active_msgs)) if i not in expired_inds]

            # Publish messages
            for msg in active_msgs:
                if cur_t - msg.last_tx_t > 1.0/msg.freq:
                    msg.last_tx_t = cur_t
                    msg.msg.timestamp = cur_t
                    can_int.write_can_msg(msg.msg)
                    logger.debug("Transmit: %s", msg.msg.log_str())



    except KeyboardInterrupt:
        logger.warning("User exited w/ Ctrl+C.")

# ==================================================================================================

if __name__ == '__main__':
    main()
