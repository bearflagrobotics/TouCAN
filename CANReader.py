
"""
Copyright 2022 Bear Flag Robotics

    CANReader.py

        Usage:
            Plug in TouCAN USB
                Verify permissions on serial port
                (eg. chmod a+rw /dev/ttyACM<NUMBER>)
            Run this python script (> python CANReader.py)
            Prints to console info/warn/errors, but logs data to separate file (timestamped)


        Dependencies: pyserial

        Author: Austin Chun
        Date:   Aug 2022

"""

# from __future__ import print_function
# Disable 'pylint: warning C0326 - Exactly one space required after comma'
# pylint: disable=C0326
# pylint: disable=C0103

from collections import namedtuple, OrderedDict

from time import time, sleep
import logging
import struct
import serial
import serial.tools.list_ports
from datetime import datetime

import os
if os.name == 'nt': # Windows
    os.system('color')

from SerialReader import SerialReader


# Define namedtuple for bit mapping field (for readability)
Field = namedtuple('Field', 'name width')

CAN_MSG_BIT_MAPPING = (
    Field('index',               16),
    Field('bus_id',               8),
    Field(' ',                    8),  # Bit padding, for 32-bit words
    Field('tstamp',              32),  # micros since start
    Field('id',                  32),
    Field('ext',                  8),
    Field('len',                  8),
    Field('timeout',             16),
    Field('data',                64),
)
# Status msg length (in bytes)
CAN_MSG_LEN = sum(fld.width for fld in CAN_MSG_BIT_MAPPING) // 8

# Construct the string format for unpacking the can msg (using the struct library)
#   See details here: https://docs.python.org/2/library/struct.html
#   Ex: "<BBHHIB" => 2 uint8, 2 uint16, 1 uint32, 1 uint8
CAN_MSG_FMT = '<' # Start with Little Endian

# Construct accompanying array, mapping the byte separations (borders) in bit positions
#   Ex: "<BBHHIB" => [0, 8, 16, 32, 48, 80]
CAN_MSG_FMT_BITS = [0] # Keep track of what bit corresponds to byte packing

bit_count = 0 # temporary variable, to count the bit count, determine where to split bytes
# Loop through mapping, in order
for fld in CAN_MSG_BIT_MAPPING:
    # Accumulate bits until reach a byte size
    bit_count += fld.width
    # When the byte size matches
    if bit_count % 8 == 0:
        # Generate bit count fmt
        CAN_MSG_FMT_BITS += [CAN_MSG_FMT_BITS[-1] + bit_count]
        # Determine what data type to use (uint8, uint16, uint32)
        size_bytes = bit_count // 8 # Integer division (though not technically neccessary)
        bit_count = 0
        if size_bytes == 1:
            CAN_MSG_FMT += 'B' # uint8_t
        elif size_bytes == 2:
            CAN_MSG_FMT += 'H' # uint16_t
        elif size_bytes == 4:
            CAN_MSG_FMT += 'I' # uint32_t
        elif size_bytes == 8:
            CAN_MSG_FMT += 'Q' # uint64_t
        else:
            raise ValueError("Invalid size of bytes, must be 1, 2, 4, or 8 bytes")



class CANReader(object):


    def __init__(self, port_path=None, logger=None, datalogname=None):

        # Logging
        if logger is None:
            logging.basicConfig(level=logging.DEBUG)
            self.logger = logging.getLogger(__name__)
        else:
            self.logger = logger

        # Log data to file?
        self.data_log = None
        if datalogname:
            self.logger.info("Logging data to '{}'".format(datalogname))
            self.data_log = logging.getLogger("data")
            self.data_log.setLevel(logging.DEBUG)
            file_handler = logging.FileHandler(datalogname)
            self.data_log.addHandler(file_handler)

            # Add Header (match CAN Parse)
            header_str = \
                "date Mon Jan 1 00:00:00 AM 2000\n" +\
                "base hex timestamps absolute\n" +\
                "no internal events logged\n" +\
                "// version 11.0.0\n" +\
                "Begin TriggerBlock Mon Jan 1 00:00:00 AM 2000\n" +\
                "   0.000000 Start of measurement"
            self.data_log.info(header_str)


        # Open the Serial port
        ser = open_serial_port_blocking(port_path=port_path)
        # Initialize the SerialReader (handles the serial bytes and checksums and stuff)
        self.reader = SerialReader(ser, verbose=True, logger=logger)


        # Create the formatting of the CAN msg format
        self.canmsg = create_status_dict_from_mapping(CAN_MSG_BIT_MAPPING)
        self.canmsg['index'] = 0
        self.last_rcv_ind = self.canmsg['index']

        self.start_msg_t = None

        self.logger.info("CANReader: Initialized.")

        # Some diagnostics
        self.msg_count = 0
        self.status_print_t = time()
        self.STATUS_PRINT_PERIOD = 5.0 # seconds


    def run(self):

        # Start a thread to read serial data
        self.reader.read_threaded()
        try:
            while True:
                data_msg = self.reader.get_data()
                # Deal with empty queue
                if data_msg is None:
                    sleep(0.001)
                    continue

                ## Process data into CAN msg
                # Unpack the data
                data_unpacked = struct.unpack(CAN_MSG_FMT, data_msg)

                # Parse the data into the dictionary
                for j, field in enumerate(CAN_MSG_BIT_MAPPING):
                    self.canmsg[field.name] = data_unpacked[j]

                # Log first time
                if self.start_msg_t is None:
                    self.start_msg_t = self.canmsg['tstamp']/1E6

                # Count messages
                self.msg_count += 1
                if time() - self.status_print_t > self.STATUS_PRINT_PERIOD:
                    self.status_print_t = time()
                    self.logger.info("Received {} msgs in last {} secs".format(self.msg_count, self.STATUS_PRINT_PERIOD))
                    self.msg_count = 0

                # Print the data
                self.print_can_msg_data(verbose=False)
                # Log data to file
                self.log_can_data(canparse_fmt=True)

        except KeyboardInterrupt:
            self.data_log.info("End TriggerBlock")
            raise KeyboardInterrupt


    def print_can_msg_data(self, verbose=False):
        """ Print CAN msg data to logger (terminal/file) """
        byte_data = struct.pack('<Q', self.canmsg['data'])
        if verbose:
            self.logger.debug("Index: {}".format(self.canmsg['index']))
            self.logger.debug("  time: {}".format(self.canmsg['tstamp']))
            self.logger.debug("  bus_id: {}".format(self.canmsg['bus_id']))
            self.logger.debug("  id: 0x{:08X}".format(self.canmsg['id']))
            self.logger.debug("  ext: {}".format(self.canmsg['ext']))
            self.logger.debug("  len: {}".format(self.canmsg['len']))
            self.logger.debug("  timeout: {}".format(self.canmsg['timeout']))
            self.logger.debug("  data: {}".format(bytearr_to_hexstr(byte_data)))
        else:
            self.logger.debug("({:010.3f}) can{:d} {:08X}#{:s}".format(
                self.canmsg['tstamp']/1E6,
                self.canmsg['bus_id'],
                self.canmsg['id'],
                bytearr_to_hexstr(byte_data, delimiter=''),
            ))

    def log_can_data(self, canparse_fmt=True):
        """ Log data to log file """
        if self.data_log is not None:
            byte_data = struct.pack('<Q', self.canmsg['data'])
            if canparse_fmt:
                self.data_log.info("{:s} {:d} {:08X}x  Rx d {:d} {:s} ".format(
                    "{:.6f}".format(self.canmsg['tstamp']/1E6-self.start_msg_t).rjust(11, ' '),
                    self.canmsg['bus_id']+1,
                    self.canmsg['id'],
                    self.canmsg['len'],
                    bytearr_to_hexstr(byte_data, delimiter=' '),
                ))
            else:
                self.data_log.info("({:010.3f}) can{:d} {:08X}#{:s}".format(
                    self.canmsg['tstamp']/1E6,
                    self.canmsg['bus_id'],
                    self.canmsg['id'],
                    bytearr_to_hexstr(byte_data, delimiter=''),
                ))

##############################################
############ External Functions ##############
##############################################

def create_status_dict_from_mapping(mapping):
    """ Convert bit mapping to dict, for variable names """
    status = OrderedDict()
    for field in mapping:
        status[field.name] = None
    return status

def open_serial_port_blocking(serial_num=None, port_path=None):
    """ Try open serial port, and wait until successful """
    last_waiting_t = time()
    while True:
        ser = open_serial_port(port_path=port_path)
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

    from datetime import datetime
    from CustomLogger import CustomLogger

    logger = CustomLogger(
        "CANReader.py",
        level=logging.INFO,
        # level=logging.DEBUG, # Uncomment this to see traffic
        verbose=True, color=True
    )

    datetime_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    datalogname = datetime_str + ".asc"
    cr = CANReader(logger=logger, datalogname=datalogname, port_path="COM7")

    try:
        cr.run()
    except KeyboardInterrupt:
        cr.logger.warning("User exited w/ Ctrl+C")

# ==================================================================================================

if __name__ == '__main__':
    main()
