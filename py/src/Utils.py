"""
Copyright 2022 Bear Flag Robotics

    Utils.py

        Store useful/common functiosn for log files.

    Author: Austin Chun
    Date:   Oct 2022
"""

# pylint: disable=C0326, C0103, C0111, W0612, C0301, C0305

from time import time, sleep
import serial
import serial.tools.list_ports


#####################################################
###              Serial Port Helpers              ###
#####################################################

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
        try:
            ser = serial.Serial(port_path, 1, timeout=0)
            print(f"uC connected on port_path='{port_path}'")
            sleep(0.5)
            ser.flush()
            sleep(0.5)
            return ser
        except serial.serialutil.SerialException:
            return None

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

#####################################################
###                 Misc Helpers                  ###
#####################################################

def bytearr_to_hexstr(arr, delimiter=' '):
    """ Convert a byte array to a hex string """
    return f'{delimiter}'.join(format(x, '02X') for x in arr)

def print_byte_array(arr, hex_fmt=False):
    """ Print byte array (optionally as hex) """
    for x in arr:
        if not hex_fmt:
            print(f"{x} ", end="")
        if hex_fmt:
            print(f"{x:2X} ", end="")
