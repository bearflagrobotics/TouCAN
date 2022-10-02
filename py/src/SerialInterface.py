"""
Copyright 2022 Bear Flag Robotics

    SerialInterface.py

        Author: Austin Chun
        Date:   Aug 2022

"""

# pylint: disable=C0103

from time import time, sleep
from enum import IntEnum

import logging
import queue
import threading
import struct

from src.Utils import bytearr_to_hexstr

######################################################################
###                     Serial Interface Class                     ###
######################################################################
class SerialInterface(object):
    """ Library to read packets of data from USB Serial port Teensy"""

    ## Formatted Message Structure
    #
    #   See README
    #
    #

    # Class "constants"
    START_BYTES = bytearray([ 0x00, 0x55 ])
    START_BYTES_LEN = len(START_BYTES)
    SER_BUFFER_MAX_LEN = 100
    DATA_BUFF_MAX_LEN = 255
    CHKSM_LEN = 2
    MAX_QUEUE_SIZE = 2000

    DATA_MSG_TYPE = 0x01
    STRING_MSG_TYPE = 0x02
    CAN_MSG_TYPE = 0x03

    class ReadState(IntEnum):
        """ Map Read states to corresponding ints """
        READ_START = 0
        READ_TYPE = 1
        READ_IDX = 2
        READ_LEN = 3
        READ_DATA = 4
        READ_CHKSM = 5
        READ_STRING = 6

    class MsgPacket():
        """ Simple Class to store message data and meta data"""

        def __init__(self, type, index, len, data, chksm):
            self.type = type
            self.index = index
            self.data_len = len
            self.data = data
            self.chksm = chksm



    def __init__(self, serial_object, logger=None):

        # Input Arguments
        self.ser = serial_object # This is the already initialized serial port
        # Logging
        if logger is None:
            self.logger = logging.getLogger(__package__)
        else:
            self.logger = logger

        # Output
        self.data_queue = queue.Queue(maxsize=self.MAX_QUEUE_SIZE) # Store all received msgs in this queue

        ### Initialize Class Variables
        self._ser_buffer = bytearray(self.SER_BUFFER_MAX_LEN) # The generic string to batch read all serial data
        self._read_state = self.ReadState.READ_START

        self._start_bytes_i = 0 # Index the start sequence

        self._rx_msg_type = 0 # What msg type was received

        self._msg_index = None # Store the message number index
        self._data_len_exp = 0 # Store the expected data length

        self._data_buff = bytearray(self.DATA_BUFF_MAX_LEN) # Accumulate the data packet here
        self._data_buff_i = 0 # Index the data sqeuence

        self._chksm = bytearray(self.CHKSM_LEN) # Store the 2-byte checksum
        self._chksm_i = 0 # Index the checksum array

        self._string_buffer = ""

        self.tx_msg_idx = 0

        # Diagnostics
        self._dropped_packets_count = 0 # number of packets dropped
        self._bad_chksm_count = 0       # number of bad checksums

        self.logger.info("SerialInterface: Initialized.")

    ###############################################
    ###                 Reading                 ###
    ###############################################

    def read_threaded(self, timeout_ms=10):
        """ Read the serial port in a separate thread """
        # Start the reading in a separate thread
        t = threading.Thread(target = self.read, daemon=True)
        t.start()

    def read(self, timeout_ms=10):
        """ Read the serial bus until empty or timeout """
        while True:
            self._read_serial(timeout_ms=timeout_ms)
            # sleep(0.001)

    def get_data(self):
        """ TODO """

        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None


    def _read_serial(self, timeout_ms=10):
        """ All serial processing happens here """

        start_t = time()

        # Read number of bytes in buffer
        bytes_to_read = self.ser.in_waiting

        # if bytes_to_read:
        #     self.logger.debug("bytes_to_read: %d", bytes_to_read)

        # Read all bytes at once (convert from str to bytearray)
        self._ser_buffer += bytearray(self.ser.read(bytes_to_read))

        # If buffer empty, nothing to do
        if not self._ser_buffer:
            # logerror("_ser_buffer is not")
            return False

        # Parse every char in the buffer
        i = 0
        while i < len(self._ser_buffer):

            if i > 1000:
                # self.logwarn("i > 1000.")
                break

            # Manual Timeout (eg. might trigger if large backlog of data in ser_buffer)
            if timeout_ms is not None:
                if (time() - start_t) > timeout_ms/1000.0:
                    self.logger.warning("read_serial timed-out.")
                    break

            cur_c = self._ser_buffer[i]
            # print("cur_c: {:02X}".format(cur_c))
            i += 1

            # self.logger.debug("ReadState: %s, CurC: 0x%02X", self._read_state, cur_c)

            # Read the Start Byte Sequence (eg. [0x00, 0x55])
            if self._read_state == self.ReadState.READ_START:
                # Check against start_byte sequence
                if cur_c == self.START_BYTES[self._start_bytes_i]:
                    self._start_bytes_i += 1
                else: # Incorrect, so restart
                    self._start_bytes_i = 0
                    if cur_c == self.START_BYTES[0]:
                        self._start_bytes_i = 1
                # Next State
                if self._start_bytes_i == self.START_BYTES_LEN: # Successful reads
                    self._start_bytes_i = 0                 # Restart the counter
                    self._read_state = self.ReadState.READ_TYPE # Move to next state

            # Read the message type
            elif self._read_state == self.ReadState.READ_TYPE:
                self._rx_msg_type = cur_c
                # Next State
                if self._rx_msg_type in [self.DATA_MSG_TYPE, self.CAN_MSG_TYPE]:
                    self._read_state = self.ReadState.READ_IDX
                elif self._rx_msg_type == self.STRING_MSG_TYPE:
                    self._read_state = self.ReadState.READ_STRING
                    self._string_buffer = "" # Reset buffer
                    self._data_buff_i = 0
                else:
                    self.logger.warning("Unknown MsgType: %02X", self._rx_msg_type)
                    self._read_state = self.ReadState.READ_START


            # Read the message index number (one byte)
            elif self._read_state == self.ReadState.READ_IDX:
                if (self._msg_index is not None) and (cur_c != (self._msg_index + 1)%256):
                    self._dropped_packets_count += (cur_c - (self._msg_index + 1)%256)
                    self.logger.warning("Missed %d msgs (total missed = %d)",
                                        cur_c - (self._msg_index + 1)%256,
                                        self._dropped_packets_count)
                # Next State
                self._msg_index = cur_c
                self._read_state = self.ReadState.READ_LEN # Move to next state

            # Read the data length (one byte)
            elif self._read_state == self.ReadState.READ_LEN:
                self._data_len_exp = cur_c
                # Next State
                # self._data_buff = bytearray(self.DATA_BUFF_MAX_LEN)  # Reset the data buffer
                self._data_buff_i = 0 # Reset the data buffer (just reset index)
                self._read_state = self.ReadState.READ_DATA

            # Read the Data into a buffer (as defined by CAN_MSG struct)
            elif self._read_state == self.ReadState.READ_DATA:
                self._data_buff[self._data_buff_i] = cur_c
                self._data_buff_i += 1
                # Next State
                if self._data_buff_i >= self._data_len_exp:
                    # self._chksm = bytearray(self.CHKSM_LEN)
                    self._chksm_i = 0
                    self._read_state = self.ReadState.READ_CHKSM

            # Read the Checksum to verify transmission (Fletcher 16)
            elif self._read_state == self.ReadState.READ_CHKSM:
                self._chksm[self._chksm_i] = cur_c
                self._chksm_i += 1
                # Next State
                if self._chksm_i >= self.CHKSM_LEN:
                    if self.verify_checksum(self._data_buff[:self._data_len_exp], self._chksm):
                        msg = self.MsgPacket(self._rx_msg_type, self._msg_index, self._data_len_exp,
                                             self._data_buff[:self._data_len_exp], self._chksm)
                        # Add data to queue (make room if needed)
                        try:
                            self.data_queue.put_nowait(msg) # Store the data
                            # print(" Put here, Qsize: {}".format(self.data_queue.qsize()))
                            # self.data_queue.put_nowait(self._data_buff[:self._data_len_exp]) # Store the data
                        except queue.Full:
                            # print(" Q ful")
                            self.logger.warning("Queue Full. Making space")
                            self.data_queue.get_nowait() # Pop an item
                            self.data_queue.put_nowait(msg) # Store the data
                            # self.data_queue.put_nowait(self._data_buff[:self._data_len_exp]) # Store the data
                    else:
                        self._bad_chksm_count += 1
                        self.logger.warning("Invalid chksm. (total invalid chksms: %d)", self._bad_chksm_count)

                        self.logger.debug("RX: %s", bytearr_to_hexstr(self._data_buff[:self._data_len_exp]))

                    self._read_state = self.ReadState.READ_START

            # Read string (endline terminated)
            elif self._read_state == self.ReadState.READ_STRING:
                if chr(cur_c) != '\n':
                    self._string_buffer += chr(cur_c)
                    self._data_buff[self._data_buff_i] = cur_c
                    self._data_buff_i += 1
                else:
                    self.logger.warning("RX: %s", self._string_buffer)
                    self._read_state = self.ReadState.READ_START


        # Remove all the processed bytes
        self._ser_buffer = self._ser_buffer[i:]

    def print_raw_rx_data(self):
        """ Print the raw RX data msg"""
        self.logger.debug("RX: (%s) %s (%s)",
            bytearr_to_hexstr(self.START_BYTES + [self._rx_msg_type, self._msg_index, self._data_len_exp]),
            bytearr_to_hexstr(self._data_buff[:self._data_len_exp]),
            bytearr_to_hexstr(self._chksm))

    ###############################################
    ###                 Writing                 ###
    ###############################################

    def write_data(self, data):
        """ TODO """
        self.write_to_serial(self.DATA_MSG_TYPE, data)

    def write_string(self, s):
        """ TODO """
        self.write_to_serial(self.STRING_MSG_TYPE, s)

    def write_can_msg(self, data):
        """ TODO """
        self.write_to_serial(self.CAN_MSG_TYPE, data)

    def write_to_serial(self, msg_type, data):
        """ TODO """

        self.ser.write(self.START_BYTES)
        self.ser.write(bytearray([msg_type]))

        # Data, CAN
        if msg_type in [self.DATA_MSG_TYPE, self.CAN_MSG_TYPE]:
            self.ser.write(bytearray([
                self.tx_msg_idx,
                len(data)
            ]))
            self.tx_msg_idx = (self.tx_msg_idx + 1) % 256
            self.ser.write(bytearray(data))
            chksm = self.calc_checksum(data)
            # self.logger.debug("  chksm: %02X", chksm)
            self.ser.write(struct.pack('H', chksm))

        # String
        elif msg_type == self.STRING_MSG_TYPE:
            self.ser.print(data)
            if data[-1] != '\n':
                self.ser.print('\n')

    ###############################################
    ###                 Helpers                 ###
    ###############################################

    def verify_checksum(self, data, chksm):
        """ Verify the Fletcher16 checksum """

        calc_chksm = self.calc_checksum(data)
        rcvd_chksm = (chksm[1] << 8) | chksm[0]  # Convert bytearray to uint16_t

        res = (calc_chksm == rcvd_chksm)

        if not res:
            self.logger.warning("Invalid chksm. Rcvd: %04X, Calc: %04X", rcvd_chksm, calc_chksm)

        return res

    def calc_checksum(self, data):
        """ Calculate the Fletcher16 checksum """
        sum1 = 0
        sum2 = 0

        for index in range(len(data)): # pylint: disable=C0200 # Make it C-like for code sharing
            sum1 = (sum1 + data[index]) % 255
            sum2 = (sum2 + sum1) % 255

        return (sum2 << 8) | sum1


# ==================================================================================================

def main():
    """ TODO """

    ###############################################################################################
    ## Parse CLI input
    import argparse
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


    # Establish serial connection
    from src.Utils import open_serial_port_blocking
    ser = open_serial_port_blocking(port_path=port_path)

    # Initialize SerialInterface object (for threaded serial read process)
    ser_int = SerialInterface(ser, logger=logger)

    ###########################################################
    # # Non-threaded run
    # try:
    #     ser_int.read()
    # except KeyboardInterrupt:
    #     logger.warning("User exited w/ Ctrl+C.")
    ###########################################################

    # Start a thread to read serial data
    ser_int.read_threaded()

    mock_can_data_tx = [
        0x00, 0x04, 0x0F, 0x0C, # ID, LSB
        0xF0, 0xFF, 0x94, 0x90, 0x1A, 0xFF, 0xFF, 0xFF # data
        # 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 # data
    ]

    # Mock: Simulate processing the data from the queue
    try:
        print_last_t = time()
        while True:
            data_msg = ser_int.get_data()
            # Deal with empty queue
            if data_msg is None:
                sleep(0.001)

                # continue
            else:
                # logger.info("RX: %s", bytearr_to_hexstr(data_msg))
                logger.info("RX: %s", bytearr_to_hexstr(data_msg.data))


            # if time() - print_last_t > 1.0:
            #     print_last_t = time()
            #     # Write Data
            #     ser_int.logger.debug("TX: %s", bytearr_to_hexstr(test_tx_data))
            #     ser_int.write_data(test_tx_data)
            #     test_tx_data[0] = (test_tx_data[0] + 1) % 256
            #     # # Write String
            #     # tx_str = "Hello World2!"
            #     # ser_int.logger.debug("TX: %s", tx_str)
            #     # ser_int.write_string(tx_str)


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
