"""
Copyright 2022 Bear Flag Robotics

    UtilsCan.py

        Useful stuff for CAN data

    Author: Austin Chun
    Date:   Sep 2022
"""


class CanMsg(object):
    """ Simple CAN Bus message data structure"""

    def __init__(self, ID, data, timestamp=0.0, bus=0):
        """ Define message packet """
        self.ID = ID
        self.data = bytearray(data) # byte array (max 8 bytes)
        self.data_len = len(data) # Number of bytes of data

        # Unused params, added to match ROS can_msgs/Frame
        self.is_rtr = False
        self.is_extended = True
        self.is_error = False
        self.dlc = 0 # uint8

        # Metadata
        self.timestamp = timestamp
        self.bus = bus # Which CAN bus

        # Extra (redundant) variables, splitting up ID into component pieces
        self.pgn = None               # Parameter Group Number (18-bit)
        self.priority = None          # 0 = highest, 7 = lowest (3-bit)
        self.pdu_format = None        # "Message Format", dictates addressable or broadcast (8-bit)
        self.pdu_specific = None      # Either Destination (addressable), or Group extension
        self.source = None            # (8-bit)
        self.destination  = None      # Only for addressable messages
        # Parse the ID
        self.pgn, self.priority, self.pdu_format, self.pdu_specific, \
            self.source, self.destination = parse_can_id(ID)

    # Helpful strings
    def data_str(self):
        return ''.join(format(x, '02X') for x in self.data)
    def log_str(self):
        return "({:011.6f}) can{:d} {:08X}#{:s}".format(
                self.timestamp,
                self.bus,
                self.ID,
                self.data_str()
        )

#####################################################
###               CAN ID Parsing                  ###
#####################################################

def parse_can_id(ID):
    # Bit Unpacking
    priority        = ID >> 26 & 0b111
    # reserved      = ID >> 25 & 0b1
    # data_page     = ID >> 24 & 0b1
    pf              = ID >> 16 & 0xFF
    ps              = ID >>  8 & 0xFF
    src             = ID >>  0 & 0xFF

    # Addressed
    if pf < 240:
        pgn = ID >> 8 & 0x3FF00
        dest = ID >> 8 & 0xFF
    # Broadcast
    else:
        pgn = ID >> 8 & 0x3FFFF
        dest = None

    return pgn, priority, pf, ps, src, dest

def get_pgn_from_can_id(ID):
    return parse_can_id(ID)[0]
def get_pri_from_can_id(ID):
    return parse_can_id(ID)[1]
def get_src_from_can_id(ID):
    return parse_can_id(ID)[4]
def get_dest_from_can_id(ID):
    return parse_can_id(ID)[5]

