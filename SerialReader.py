### """### !/usr/bin/env python"""

"""
Copyright 2022 Bear Flag Robotics

    Teensy_Serial_Interface.py

        Author: Austin Chun
        Date:   Aug 2022

"""

# from __future__ import print_function
# Disable 'pylint: warning C0326 - Exactly one space required after comma'
# pylint: disable=C0326
# pylint: disable=C0103

from time import time, sleep
from enum import IntEnum

import logging
import queue
import threading
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
    DATA_BUFF_MAX_LEN = 256
    CHKSM_LEN = 2
    MAX_QUEUE_SIZE = 2000

    class ReadState(IntEnum):
        """ Map Read states to corresponding ints """
        READ_START = 0
        READ_IDX = 1
        READ_LEN = 2
        READ_DATA = 3
        READ_CHKSM = 4


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

        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None


    def _read_serial(self, timeout_ms=10):
        """ All serial processing happens here """

        start_t = time()

        # Read number of bytes in buffer
        bytes_to_read = self.ser.in_waiting

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

            # Read the Start Byte Sequence (eg. [0x00, 0x55])
            if self._read_state == self.ReadState.READ_START:
                # Check against start_byte sequence
                if cur_c == self.START_BYTES[self._start_bytes_i]:
                    self._start_bytes_i += 1
                else: # Incorrect, so restart
                    self._start_bytes_i = 0
                # Next State
                if self._start_bytes_i == self.START_BYTES_LEN: # Successful reads
                    self._start_bytes_i = 0                 # Restart the counter
                    self._read_state = self.ReadState.READ_IDX # Move to next state

            # Read the message index number (one byte)
            elif self._read_state == self.ReadState.READ_IDX:
                if (self._msg_index is not None) and (cur_c != (self._msg_index + 1)%256):
                    self._dropped_packets_count += (cur_c - (self._msg_index + 1)%256)
                    self.logger.warning("Missed {} msgs (total missed = {})".format(cur_c - (self._msg_index + 1)%256, self._dropped_packets_count))
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
                        self.logger.warning("Invalid chksm. (total invalid chksms: {})".format(self._bad_chksm_count))

                    self._read_state = self.ReadState.READ_START

        # Remove all the processed bytes
        self._ser_buffer = self._ser_buffer[i:]

    def verify_checksum(self, data, chksm):
        """ Verify the Fletcher16 checksum """
        sum1 = 0
        sum2 = 0

        for index in range(len(data)):
            sum1 = (sum1 + data[index]) % 255
            sum2 = (sum2 + sum1) % 255

        calc_chksm = (sum2 << 8) | sum1     # Calculated chksm
        chksm = (chksm[1] << 8) | chksm[0]  # Received chksm

        res = (calc_chksm == chksm)

        if not res:
            self.logger.warning("Invalid chksm. Rcvd: {:X}, Calc: {:X}".format(chksm, calc_chksm))

        return calc_chksm == chksm


##############################################
############ External Functions ##############
##############################################

def open_serial_port_blocking(serial_num=None, port_path=None):
    """ Try open serial port, and wait until successful """
    last_waiting_t = time()
    while True:
        ser = open_serial_port()
        if ser is not None:
            break
        sleep(0.1)
        if time() - last_waiting_t > 5.0:
            last_waiting_t = time()
            print("No Serial port found. Waiting...")
    return ser

def open_serial_port(serial_num=None, port_path=None):
    """ Simple wrapper function for handling opening a serial port. Returns the opened serial port"""

    if serial_num is not None:
        print("Trying to connect to uC with serial#='{}'".format(serial_num))
        # Get port number and verify opened. If unable to open, print error and quit node.
        port_path = get_port_number(serial_num) # returns None if not found.
        if port_path is None:
            # print("uC not found with serial#='{}'".format(serial_num))
            # print("Shutting down.")
            return None
        else:
            print("uC connected on port_path='{}'".format(port_path))
            ser = serial.Serial(port_path, 1, timeout=0)
            sleep(0.5)
            ser.flush()
            sleep(0.5)
            return ser

    elif port_path is not None:
        print("uC connected on port_path='{}'".format(port_path))
        ser = serial.Serial(port_path, 1, timeout=0)
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
                print("uC connected on port_path='{}'".format(port_path))
                ser = serial.Serial(port_path, 1, timeout=0)
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
    else:
        return None

def bytearr_to_hexstr(arr, delimiter=' '):
    """ Convert a byte array to a hex string """
    return '{}'.format(delimiter).join(format(x, '02X') for x in arr)


# ==================================================================================================

def main():

    from CustomLogger import CustomLogger

    logger = CustomLogger("SerialReader.py",
        level=logging.DEBUG,
        verbose=True, color=True
    )

    # Establish serial connection
    ser = open_serial_port_blocking()

    # Initialize SerialReader object (for threaded serial read process)
    reader = SerialReader(ser, verbose=True, logger=logger)

    # Start a thread to read serial data
    reader.read_threaded()

    # Mock: Simulate processing the data from the queue
    try:
        while True:
            data_msg = reader.get_data()
            # Deal with empty queue
            if data_msg is None:
                sleep(0.001)
                continue
            logger.info("RX: {}".format(bytearr_to_hexstr(data_msg)))

    except KeyboardInterrupt:
        logger.warning("User exited w/ Ctrl+C.")

# ==================================================================================================

if __name__ == '__main__':
    main()
