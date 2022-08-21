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

# pylint: disable=C0103

from collections import namedtuple, OrderedDict

from time import time, sleep
from datetime import datetime
import logging
import struct
import serial
import serial.tools.list_ports
from SerialReader import SerialReader

from utils import open_serial_port_blocking, bytearr_to_hexstr

class CANReader(object):
    """ TODO """

    Field = namedtuple('Field', 'name width') # namedtuple for bit mapping field (for readability)

    # Define the CAN msg bit mapping
    #   This must be in sync with the TouCAN formatting
    #   This is later parsed to get relevant parsing attributes (msg length, unpacking string, etc)
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


    def __init__(self, port_path=None, logger=None, datalogname=None):

        ##############################
        ### Handle Input Arguments ###
        # Logging
        if logger is None:
            logging.basicConfig(level=logging.DEBUG)
            self.logger = logging.getLogger(__name__)
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

        # Open the Serial port
        ser = open_serial_port_blocking(port_path=port_path)
        # Initialize the SerialReader (handles the serial bytes and checksums and stuff)
        self.reader = SerialReader(ser, verbose=True, logger=logger)

        ### Setup CAN
        ## Create the formatting of the CAN msg format
        #   CAN_MSG_LEN: The len (bytes) of the CAN msg
        #   CAN_MSG_FMT: struct.pack string formatter
        #                eg: "<BBHHIB" => LittleEndian, 2 uint8, 2 uint16, 1 uint32, 1 uint8
        #   rxmsg: dictionary for storing the received canmsg
        #           will dynamically update
        self.CAN_MSG_LEN, self.CAN_MSG_FMT, self.rx_msg = \
            CANReader.parse_can_msg_bit_mapping(self.CAN_MSG_BIT_MAPPING)
        self.rx_msg['index'] = 0
        self.rx_last_ind = self.rx_msg['index']
        self.rx_first_msg_t = None # Time of the first received msg (for better log timing)

        # Some diagnostics
        self.msg_count = 0
        self.status_print_t = time()
        self.STATUS_PRINT_PERIOD = 5.0 # seconds
        self.buses_seen = set() # Log bus activity

        self.logger.info("CANReader: Initialized.")


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


    def run(self):
        """ TODO """

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
                data_unpacked = struct.unpack(self.CAN_MSG_FMT, data_msg)

                # Parse the data into the dictionary
                for j, field in enumerate(self.CAN_MSG_BIT_MAPPING):
                    self.rx_msg[field.name] = data_unpacked[j]

                # Log first time
                if self.rx_first_msg_t is None:
                    self.rx_first_msg_t = self.rx_msg['tstamp']/1E6

                # Log first data on buses
                if self.rx_msg['bus_id'] not in self.buses_seen:
                    self.buses_seen.add(self.rx_msg['bus_id'])
                    self.logger.info("  Received first msg on bus %d", self.rx_msg['bus_id'])

                # Count messages
                self.msg_count += 1
                if time() - self.status_print_t > self.STATUS_PRINT_PERIOD:
                    self.status_print_t = time()
                    self.logger.info("Received %d msgs in last %.1f secs",
                                     self.msg_count, self.STATUS_PRINT_PERIOD)
                    self.msg_count = 0

                # Print the data
                self.print_can_msg_data(verbose=False)
                # Log data to file
                self.log_can_data(canparse_fmt=True)

        except KeyboardInterrupt as exc:
            self.data_log.info("End TriggerBlock")
            raise exc


    def print_can_msg_data(self, verbose=False):
        """ Print CAN msg data to logger (terminal/file) """
        byte_data = struct.pack('<Q', self.rx_msg['data'])
        if verbose:
            self.logger.debug("Index: %d", self.rx_msg['index'])
            self.logger.debug("  time: %f", self.rx_msg['tstamp'])
            self.logger.debug("  bus_id: %d", self.rx_msg['bus_id'])
            self.logger.debug("  id: 0x%08X", self.rx_msg['id'])
            self.logger.debug("  ext: %d", self.rx_msg['ext'])
            self.logger.debug("  len: %d", self.rx_msg['len'])
            self.logger.debug("  timeout: %d", self.rx_msg['timeout'])
            self.logger.debug("  data: %s", bytearr_to_hexstr(byte_data))
        else:
            self.logger.debug("(%010.6f) can%d %08X#%s",
                self.rx_msg['tstamp']/1E6,
                self.rx_msg['bus_id'],
                self.rx_msg['id'],
                bytearr_to_hexstr(byte_data, delimiter='')
            )

    def log_can_data(self, canparse_fmt=True):
        """ Log data to log file """
        if self.data_log is not None:
            byte_data = struct.pack('<Q', self.rx_msg['data'])
            if canparse_fmt:
                self.data_log.info("%s %d %08Xx  Rx d %d %s",
                    f"{(self.rx_msg['tstamp']/1E6-self.rx_first_msg_t):.6f}".rjust(11, ' '),
                    self.rx_msg['bus_id']+1,
                    self.rx_msg['id'],
                    self.rx_msg['len'],
                    bytearr_to_hexstr(byte_data, delimiter=' '),
                )
            else:
                self.data_log.info("(%010.3f) can%d %08X#%s",
                    self.rx_msg['tstamp']/1E6,
                    self.rx_msg['bus_id'],
                    self.rx_msg['id'],
                    bytearr_to_hexstr(byte_data, delimiter=''),
                )



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
            "CANReader.py",
            level=logger_lvl,
            verbose=verbose,
            color=color
        )
    except ModuleNotFoundError as e:
        print(f"  Failed to load CustomLogger: {e}")
        logger = logging.getLogger()

    datetime_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    datalogname = datetime_str + ".asc"
    cr = CANReader(logger=logger, datalogname=datalogname, port_path=port_path)

    try:
        cr.run()
    except KeyboardInterrupt:
        cr.logger.warning("User exited w/ Ctrl+C")

# ==================================================================================================

if __name__ == '__main__':
    main()
