"""
Copyright 2022 Bear Flag Robotics

    CanInterface.py

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

# pylint: disable=C0103

from collections import namedtuple, OrderedDict

from time import time, sleep
from datetime import datetime
import logging
import struct

from utils import open_serial_port_blocking, bytearr_to_hexstr
from can_utils import CanMsg


from SerialInterface import SerialInterface


class CanInterface(object):
    """ TODO """


    # Define the CAN msg bit mapping
    #   This must be in sync with the TouCAN formatting
    #   This is later parsed to get relevant parsing attributes (msg length, unpacking string, etc)
    Field = namedtuple('Field', 'name width') # namedtuple for bit mapping field (for readability)
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


    def __init__(self, serial_object, logger=None, datalogname=None):

        ##############################
        ### Handle Input Arguments ###
        self.ser = serial_object # This is the already initialized serial port

        # Logging
        if logger is None:
            self.logger = logging.getLogger(__package__)
        else:
            self.logger = logger

        # Log data to file?
        self.data_log = None
        if datalogname:
            self.logger.info("Logging data to '%s'", datalogname)
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

        # Initialize the SerialInterface (handles the serial bytes and checksums and stuff)
        self.ser_int = SerialInterface(self.ser, logger=logger)

        # Start a thread to read serial data
        self.ser_int.read_threaded()

        ### Setup CAN
        ## Create the formatting of the CAN msg format
        #   CAN_MSG_LEN: The len (bytes) of the CAN msg
        #   CAN_MSG_FMT: struct.pack string formatter
        #                eg: "<BBHHIB" => LittleEndian, 2 uint8, 2 uint16, 1 uint32, 1 uint8
        #   rx_msg_dict: dictionary for storing the received canmsg. will dynamically update
        self.CAN_MSG_LEN, self.CAN_MSG_FMT, _ = \
            CanInterface.parse_can_msg_bit_mapping(self.CAN_MSG_BIT_MAPPING)
        self.rx_last_ind = 0
        self.rx_first_msg_t = None # Time of the first received msg (for better log timing)

        # Some diagnostics
        self.msg_count = 0
        self.status_print_t = time()
        self.STATUS_PRINT_PERIOD = 5.0 # seconds
        self.buses_seen = set() # Log bus activity

        self.logger.info("CanInterface: Initialized.")


    @staticmethod
    def parse_can_msg_bit_mapping(mapping):
        """ Do some parsing magic, to allow simple msg defintion, then extract useful formatting data """
        # Status msg length (in bytes)
        can_msg_len = sum(fld.width for fld in mapping) // 8

        # Construct the string format for unpacking the can msg (using the struct library)
        #   See details here: https://docs.python.org/2/library/struct.html
        #   Eg: "<BBHHIB" => 2 uint8, 2 uint16, 1 uint32, 1 uint8
        can_msg_fmt = '<' # Start with Little Endian

        # Construct accompanying array, mapping the byte separations (borders) in bit positions
        #   Eg: "<BBHHIB" => [0, 8, 16, 32, 48, 80]
        _can_msg_fmt_bits = [0] # Keep track of what bit corresponds to byte packing

        _bit_count = 0 # temporary variable, to count the bit count, determine where to split bytes
        # Loop through mapping, in order
        for fld in mapping:
            # Accumulate bits until reach a byte size
            _bit_count += fld.width
            # When the byte size matches
            if _bit_count % 8 == 0:
                # Generate bit count fmt
                _can_msg_fmt_bits += [_can_msg_fmt_bits[-1] + _bit_count]
                # Determine what data type to use (uint8, uint16, uint32)
                _size_bytes = _bit_count // 8 # Integer division (though not technically neccessary)
                _bit_count = 0
                if _size_bytes == 1:
                    can_msg_fmt += 'B' # uint8_t
                elif _size_bytes == 2:
                    can_msg_fmt += 'H' # uint16_t
                elif _size_bytes == 4:
                    can_msg_fmt += 'I' # uint32_t
                elif _size_bytes == 8:
                    can_msg_fmt += 'Q' # uint64_t
                else:
                    raise ValueError("Invalid size of bytes, must be 1, 2, 4, or 8 bytes")

        ## Convert bit mapping to dict, for variable names
        can_msg_dict = OrderedDict()
        for fld in mapping:
            can_msg_dict[fld.name] = None

        return can_msg_len, can_msg_fmt, can_msg_dict

    ###############################################
    ###                 Reading                 ###
    ###############################################

    def get_rx_msg(self):
        """ Pop a data packet from the Serial rx queue, and process as CAN data """

        # Get data packet from queue (from SerialInterface)
        data_msg = self.ser_int.get_data()

        # Deal with empty queue
        if data_msg is None:
            return None

        # Check if CAN msg
        if data_msg.type == self.ser_int.CAN_MSG_TYPE:
            ## Process data into CAN msg
            # Unpack the data
            data_unpacked = struct.unpack(self.CAN_MSG_FMT, data_msg.data)

            # Parse the data into the dictionary
            rx_dict = {}
            for j, field in enumerate(self.CAN_MSG_BIT_MAPPING):
                rx_dict[field.name] = data_unpacked[j]
            # Parse fields into CAN msg
            byte_data = struct.pack('<Q', rx_dict['data'])
            rx_msg = CanMsg(rx_dict['id'], byte_data,
                            timestamp=rx_dict['tstamp']/1E6, bus=rx_dict['bus_id'])

            # Log first time
            if self.rx_first_msg_t is None:
                self.rx_first_msg_t = rx_msg.timestamp
            # Log first data on buses
            if rx_msg.bus not in self.buses_seen:
                self.buses_seen.add(rx_msg.bus)
                self.logger.info("  Received first msg on bus %d", rx_msg.bus)

            # Count messages
            self.msg_count += 1
            if time() - self.status_print_t > self.STATUS_PRINT_PERIOD:
                self.status_print_t = time()
                self.logger.info("Received %d msgs in last %.1f secs",
                                 self.msg_count, self.STATUS_PRINT_PERIOD)
                self.msg_count = 0

            return rx_msg

        # Don't care about other msg types
        return None

    ###############################################
    ###                 Writing                 ###
    ###############################################

    def write_can_msg(self, msg):
        """ TODO """
        id_bytes = struct.pack('<I', msg.ID)
        msg_byte_data = bytearray([msg.bus]) + id_bytes + msg.data
        self.ser_int.write_can_msg(msg_byte_data)

    ###############################################
    ###                 Helpers                 ###
    ###############################################

    def print_can_msg_data(self, msg, verbose=False):
        """ Print CAN msg data to logger (terminal/file) """
        if verbose:
            # self.logger.debug("Index: %d", msg.index)
            self.logger.debug("Msg")
            self.logger.debug("  time: %f", msg.timestamp)
            self.logger.debug("  bus_id: %d", msg.bus)
            self.logger.debug("  id: 0x%08X", msg.ID)
            # self.logger.debug("  ext: %d", msg.ext)
            self.logger.debug("  len: %d", msg.data_len)
            # self.logger.debug("  timeout: %d", msg.timeout)
            self.logger.debug("  data: %s", bytearr_to_hexstr(msg.data))
        else:
            self.logger.debug("Receive:  %s", msg.log_str)

    def log_can_data(self, msg, canparse_fmt=True):
        """ Log data to log file """
        if self.data_log is not None:
            if canparse_fmt:
                self.data_log.info("%s %d %08Xx  Rx d %d %s",
                    f"{(msg.timestamp-self.rx_first_msg_t):.6f}".rjust(11, ' '),
                    msg.bus+1,
                    msg.ID,
                    msg.data_len,
                    bytearr_to_hexstr(msg.data, delimiter=' '),
                )
            else:
                self.data_log.info(msg.log_str)

# ==================================================================================================

def main():
    """ Simple example of usage """

    # Specify path to uC
    port_path = '/dev/ttyACM0'

    # Establish serial connection
    ser = open_serial_port_blocking(port_path=port_path)

    # Initialize SerialInterface object (for threaded serial read process)
    can_int = CanInterface(ser, logger=None, datalogname=None)


    try:
        # Sample Message to transmit
        msg_1 = CanMsg(0x1A0123FF, [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77], bus=0)
        tx_t1 = time() # Time the transmit

        while True:
            ## Read from CanInterface, and print
            rx_msg = can_int.get_rx_msg()
            if rx_msg is None: # Deal with empty queue
                sleep(0.001)
            else:
                can_int.print_can_msg_data(rx_msg, verbose=False)

            ## Write Data periodically
            if time() - tx_t1 > 1.0:
                tx_t1 = time()
                can_int.write_can_msg(msg_1)
                can_int.logger.info("Transmit: %s", msg_1.log_str)

    except KeyboardInterrupt:
        can_int.logger.warning("User exited w/ Ctrl+C.")

# ==================================================================================================

if __name__ == '__main__':
    main()
