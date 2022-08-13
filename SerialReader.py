"""
Copyright 2022 Bear Flag Robotics

    SerialReader.py

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
import serial
import serial.tools.list_ports

##########################################################################
###                     CAN Reader Interface Class                     ###
##########################################################################
class SerialReader(object):
    """ Library to read packets of data from """

    ## Formatted Message Structure
    ##
    ##          START_BYTES  INDEX  DATA_LEN   DATA                        CHKSM
    ## eg.      00 55        00     08         00 11 22 33 44 55 66 77     9D 4F <made up
    ## eg.      00 55        01     04         00 11 22 33                 C2 35 <made up
    ## eg.      00 55        02     06         00 11 22 33 44 55           4D EA <made up

    # Class "constants"
    START_BYTES = bytearray([ 0x00, 0x55 ])
    START_BYTES_LEN = len(START_BYTES)
    SER_BUFFER_MAX_LEN = 100
    DATA_BUFF_MAX_LEN = 255
    CHKSM_LEN = 2
    MAX_QUEUE_SIZE = 2000

    DATA_MSG_TYPE = 0x01
    STRING_MSG_TYPE = 0x02

    class ReadState(IntEnum):
        """ Map Read states to corresponding ints """
        READ_START = 0
        READ_TYPE = 1
        READ_IDX = 2
        READ_LEN = 3
        READ_DATA = 4
        READ_CHKSM = 5
        READ_STRING = 6


    def __init__(self, serial_object, verbose=False, logger=None):

        # Input Arguments
        self.ser = serial_object # This is the already initialized serial port
        self.verbose = verbose
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

        self.logger.info("SerialReader: Initialized.")

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
                # Next State
                if cur_c == self.DATA_MSG_TYPE:
                    self._read_state = self.ReadState.READ_IDX
                elif cur_c == self.STRING_MSG_TYPE:
                    self._read_state = self.ReadState.READ_STRING
                    self._string_buffer = "" # Reset buffer
                    self._data_buff_i = 0
                else:
                    self.logger.warning("Unknown MsgType: %02X", cur_c)
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
                        # Add data to queue (make room if needed)
                        try:
                            # print(" Put here, Qsize: {}".format(self.data_queue.qsize()))
                            self.data_queue.put_nowait(self._data_buff[:self._data_len_exp]) # Store the data
                        except queue.Full:
                            # print(" Q ful")
                            self.logger.warning("Queue Full. Making space")
                            self.data_queue.get_nowait() # Pop an item
                            self.data_queue.put_nowait(self._data_buff[:self._data_len_exp]) # Store the data
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

    def write_data(self, data):
        """ TODO """
        self.ser.write(self.START_BYTES)
        self.ser.write(bytearray([
            self.DATA_MSG_TYPE,
            self.tx_msg_idx,
            len(data)
        ]))
        self.tx_msg_idx += 1
        self.ser.write(bytearray(data))
        chksm = self.calc_checksum(data)
        # self.logger.debug("  chksm: %02X", chksm)
        self.ser.write(struct.pack('H', chksm))

    def write_string(self, s):
        """ TODO """
        self.ser.write(self.START_BYTES)
        self.ser.write(bytearray([self.STRING_MSG_TYPE]))
        self.ser.print(s)
        if s[-1] != '\n':
            self.ser.print('\n')



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


##############################################
############ External Functions ##############
##############################################

def open_serial_port_blocking(serial_num=None, port_path=None):
    """ Try open serial port, and wait until successful """
    last_waiting_t = time()
    while True:
        ser = open_serial_port(serial_num=serial_num, port_path=port_path)
        if ser is not None:
            break
        sleep(0.1)
        if time() - last_waiting_t > 5.0:
            last_waiting_t = time()
            print("No Serial port found. Waiting...")
    return ser


def open_serial_port(serial_num=None, port_path=None):
    """ Simple wrapper function for handling opening serial port. Returns the opened serial port"""

    if serial_num is not None:
        print(f"Trying to connect to uC with serial#='{serial_num}'")
        # Get port number and verify opened. If unable to open, print error and quit node.
        port_path = get_port_number(serial_num) # returns None if not found.
        if port_path is None:
            # print("uC not found with serial#='{}'".format(serial_num))
            # print("Shutting down.")
            return None
        else:
            print(f"uC connected on port_path='{port_path}'")
            ser = serial.Serial(port_path, 1, timeout=0, write_timeout=0)
            sleep(0.5)
            ser.flush()
            sleep(0.5)
            return ser

    elif port_path is not None:
        print(f"uC connected on port_path='{port_path}'")
        ser = serial.Serial(port_path, 1, timeout=0, write_timeout=0)
        sleep(0.5)
        ser.flush()
        sleep(0.5)
        return ser

    else: # Look for Teensy PID/VID
        TEENSY_PID = 0x0483
        TEENSY_VID = 0x16C0
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if (port.pid == TEENSY_PID) and (port.vid == TEENSY_VID):
                port_path = port.device
                print(f"uC connected on port_path='{port_path}'")
                ser = serial.Serial(port_path, 1, timeout=0, write_timeout=0)
                sleep(0.5)
                ser.flush()
                sleep(0.5)
                return ser
        # print("No Teensy found...")
        # print("Shutting down.")
        return None


def get_port_number(serial_number):
    """ return port number for serial_number or None if not found. """
    ports = list(serial.tools.list_ports.grep(serial_number))
    if len(ports) > 0:
        return ports[0].device
    return None

def bytearr_to_hexstr(arr, delimiter=' '):
    """ Convert a byte array to a hex string """
    return f'{delimiter}'.join(format(x, '02X') for x in arr)


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
        from CustomLogger import CustomLogger
        # Init Logger
        logger = CustomLogger(
            "SerialReader.py",
            level=logger_lvl,
            verbose=verbose,
            color=color
        )
    except ModuleNotFoundError as e:
        print(f"  Failed to load CustomLogger: {e}")
        logger = logging.getLogger()


    # Establish serial connection
    ser = open_serial_port_blocking(port_path=port_path)

    # Initialize SerialReader object (for threaded serial read process)
    reader = SerialReader(ser, verbose=True, logger=logger)

    ###########################################################
    # # Non-threaded run
    # try:
    #     reader.read()
    # except KeyboardInterrupt:
    #     logger.warning("User exited w/ Ctrl+C.")
    ###########################################################

    # Start a thread to read serial data
    reader.read_threaded()


    print_last_t = time()

    test_tx_data = [0xF0, 0x02, 0x03, 0x04, 0x05]

    mock_can_data_tx = [
        0x00, 0x04, 0x0F, 0x0C, # ID, LSB
        0xF0, 0xFF, 0x94, 0x90, 0x1A, 0xFF, 0xFF, 0xFF # data
        # 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 # data
    ]


    # Mock: Simulate processing the data from the queue
    try:
        while True:
            data_msg = reader.get_data()
            # Deal with empty queue
            if data_msg is None:
                sleep(0.001)

                # continue
            else:
                logger.info("RX: %s", bytearr_to_hexstr(data_msg))


            # if time() - print_last_t > 1.0:
            #     print_last_t = time()
            #     # Write Data
            #     reader.logger.debug("TX: %s", bytearr_to_hexstr(test_tx_data))
            #     reader.write_data(test_tx_data)
            #     test_tx_data[0] = (test_tx_data[0] + 1) % 256
            #     # # Write String
            #     # tx_str = "Hello World2!"
            #     # reader.logger.debug("TX: %s", tx_str)
            #     # reader.write_string(tx_str)


            # Write mock CAN data
            if time() - print_last_t > 1.0:
                print_last_t = time()
                reader.logger.debug("TX: %s", bytearr_to_hexstr(mock_can_data_tx))
                reader.write_data(mock_can_data_tx)


    except KeyboardInterrupt:
        logger.warning("User exited w/ Ctrl+C.")

# ==================================================================================================

if __name__ == '__main__':
    main()
