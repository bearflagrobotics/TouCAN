"""
Copyright 2021 Bear Flag Robotics

    utils.py

    Store useful/common functiosn for log files.

    Author: Austin Chun
    Date:   Oct 2021
"""

# pylint: disable=C0326, C0103, C0111, W0612, C0301, C0305
# pylint: disable=C0305

import math
import yaml
from time import time, sleep
import serial

class CAN_Log(object):
    """
        Simple class to load a log file

        Assumes following format
        (<time_in_seconds_float>) <can_bus> <CAN_ID_HEX>#<DATA_HEX>

        e.g.
            (0.02) can0 18EF0006#CD4FFFFFFFFFFFF
            (0.23) can0 8FF6505#42000FAF00
            (0.35) can0 4EF0006#641510F10048D

    """

    def __init__(self, filename):


        # Array of CAN msg objects
        self.msgs = []

        # Initialize variables (optional to calculate)
        self.time_array = None
        self.ID_array = None
        self.data_array = None


        # Load the file
        self.load_file(filename)



    def load_file(self, filename, verbose=False):

        # Read in the file
        with open(filename, 'r') as f:
            lines = f.readlines()

        # Loop through lines
        for line_i, line in enumerate(lines):
            try:
                # Parse the line
                arr = line.strip("\n").split(" ")
                t = float(arr[0].strip("()"))
                bus = arr[1]

                IDdata = arr[2].split("#") # eg. "18B1FF07#1F08CC48DF72B6B3"
                ID = int(IDdata[0], 16) # Parse ID from Hex

                # Parse each byte into data array
                data = [int(IDdata[1][i:i+2], 16) for i in range(0, len(IDdata[1]), 2)]

                # Create and append the CAN msg
                self.msgs += [CAN_Msg(ID, data, t, bus)]


            except ValueError as e:
                if verbose:
                    print("ValueError, line {}: {}".format(line_i, e))

            except IndexError as e:
                if verbose:
                    print("IndexError, line {}: {}".format(line_i, e))


    def get_time_array(self):
        if not self.time_array:
            self.time_array = [msg.t for msg in self.msgs]
        return self.time_array

    def get_ID_array(self):
        if not self.ID_array:
            self.ID_array = [msg.ID for msg in self.msgs]
        return self.ID_array

    def get_data_array(self):
        if not self.data_array:
            self.data_array = [msg.data for msg in self.msgs]
        return self.data_array



class CAN_Msg(object):

    def __init__(self, ID, data, t=None, bus=None):
        """ """
        self.ID = ID
        self.data = data # Array for each byte (max 8 bytes)
        self.data_len = len(data) # Number of bytes of data

        self.t = t
        self.bus = bus


        self.PGN = None               # Parameter Group Number (18-bti)
        self.priority = None          # 0 = highest, 7 = lowest (3-bit)
        self.pdu_format = None        # "Message Format", dictates addressable or broadcast (8-bit)
        self.pdu_specific = None      # Either Destination (addressable), or Group extension
        self.source = None            # (8-bit)
        self.destination  = None      # Only for addressable messages

        # Parsing
        self.PGN, self.priority, self.pdu_format, self.pdu_specific, \
            self.source, self.destination = parseID(ID)


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
            ser = serial.Serial(port_path, 1, timeout=0)
            sleep(0.5)
            ser.flush()
            sleep(0.5)
            return ser

    elif port_path is not None:
        print(f"uC connected on port_path='{port_path}'")
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
                print(f"uC connected on port_path='{port_path}'")
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
    return None

def bytearr_to_hexstr(arr, delimiter=' '):
    """ Convert a byte array to a hex string """
    return f'{delimiter}'.join(format(x, '02X') for x in arr)



#####################################################
###               CAN ID Parsing                  ###
#####################################################

def parseID(ID):
    # Bit Unpacking
    priority        = ID >> 26 & 0b111
    # reserved      = ID >> 25 & 0b1
    # data_page     = ID >> 24 & 0b1
    PF              = ID >> 16 & 0xFF
    PS              = ID >>  8 & 0xFF
    src             = ID >>  0 & 0xFF

    # Addressed
    if PF < 240:
        PGN = ID >> 8 & 0x3FF00
        dest = ID >> 8 & 0xFF
    # Broadcast
    else:
        PGN = ID >> 8 & 0x3FFFF
        dest = None

    return PGN, priority, PF, PS, src, dest


def IDgetPGN(ID):
    return parseID(ID)[0]
def IDgetPRI(ID):
    return parseID(ID)[1]
def IDgetSRC(ID):
    return parseID(ID)[4]
def IDgetDEST(ID):
    return parseID(ID)[5]

##############################################################
###               CAN msg array functions                  ###
##############################################################

def get_time(can_msg_arr):
    if not can_msg_array_check(can_msg_arr):
        print("Cannot operate. only use on CAN_Msg arrays")
    return [msg.t for msg in can_msg_arr]

def get_data(can_msg_arr):
    if not can_msg_array_check(can_msg_arr):
        print("Cannot operate. only use on CAN_Msg arrays")
    return [msg.data for msg in can_msg_arr]

def cat_bytes(data_arr, indexes):
    """
        Input is an array of byte-arrayget_

        indexs is order of bytes, LSB first
    """

    result = [0] * len(data_arr)
    for i, d in enumerate(data_arr):
        for j, ind in enumerate(indexes):
            result[i] += d[ind] << (8 * j)

    return result

def splice_byte(data_arr, index_start, bit_len):
    """
        Input is an array of byte array

        Indexes are in Byte-bit format, index at 1 for both bytes and bits,
        and inclusive

        e.g 3rd byte, 1st bit to 4th bit ==> 3.1 and 4

        Note: Does not support spanning bytes

    """

    byte_ind = int(math.floor(index_start-1))

    bit_start = int(((index_start%1)*10)-1)

    print("byte_ind:", byte_ind)
    print("bit_start:", bit_start)

    result = [0] * len(data_arr)
    for i, d in enumerate(data_arr):
        result[i] = (d[byte_ind] >> bit_start) & ((0b1 << bit_len)-1)

    return result


def can_msg_array_check(can_msg_arr):
    if not can_msg_arr:
        print("Empty array")
        raise ValueError

    if not isinstance(can_msg_arr[0], CAN_Msg):
        print("Array is not of CAN_Msgs.")
        raise ValueError

    return True


def print_byte_array(arr, hex_fmt=False):
    for x in arr:
        if not hex_fmt:
            print("{} ".format(x), end="")
        if hex_fmt:
            print("{:2X} ".format(x), end="")


##########################################################
###               Load YAML Databases                  ###
##########################################################

def load_PGN_db():
    with open("yamls/J1939-PGNs.yaml", 'r') as f:
        pgn_db = yaml.safe_load(f)
    return pgn_db

def load_JD_src_db():
    with open("yamls/JD-Source-Mapping.yaml", 'r') as f:
        src_db = yaml.safe_load(f)
    return src_db
