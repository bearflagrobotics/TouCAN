"""
    utils.py



        Author: Austin Chun <austin@bearflagrobotics.com>
        Date:   Jan 2021
"""
# Disable some pylint warnings
# pylint: disable=C0326, C0103

from __future__ import print_function

import os
import sys

import re
import subprocess
from subprocess import Popen, PIPE
import checksumdir
import yaml
import serial.tools.list_ports

def calculate_dir_hash():
    """ Calculate the directory hash of the embedded folder """
    full_path = sys.path[0]
    dir_path, _ = os.path.split(full_path)
    dir_sha1 = checksumdir.dirhash(dir_path, 'sha1', ignore_hidden=True,
                                   excluded_files=['git_info.h'])
    return dir_sha1


def read_yaml_config_file(file_path):
    """ Read a YAML config file """
    with open(file_path, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
            return data
        except yaml.YAMLError as exc:
            print(exc)

# Define Teensy Vendor ID and Product ID(different based on mode)
TEENSY_VID          = 0x16C0
TEENSY_SERIAL_PID   = 0x0483
TEENSY_BOOT_PID     = 0x0478
TEENSY_HID_PID      = 0x0486


def check_port_for_teensy(upload_port):
    """ Check the specified upload port for a valid target device """
    print("  UPLOAD_PORT defined in env as {}".format(upload_port))
    all_ports = list(serial.tools.list_ports.comports())

    des_uc_id = None

    # Find the serial number for the port (bit of a hack to keep rest of code the same)
    for p in all_ports:
        # Look for Teensy at specified port
        if p.device == upload_port:
            # Make sure it is a Teensy (VID and PID according to udev)
            # if (hex(p.vid) in ['0x16c0']) and (re.match("04[789B]?", hex(p.pid)[2:].zfill(4))):
            if (p.vid == TEENSY_VID) and (p.pid == TEENSY_SERIAL_PID):
                des_uc_id = p.serial_number
                break
            else:
                raise ValueError("Found device at port {},".format(upload_port) +\
                                 " but it is not a Teensy. Failed to Flash.")
    # If no device at port, raise error
    if des_uc_id is None:
        raise ValueError("No device found at {}. Failed to Flash.".format(upload_port))

    return des_uc_id


def get_port_number(serial_number):
    """ return port number for serial_number or None if not found. """
    ports = list(serial.tools.list_ports.grep(serial_number))
    if ports:
        return ports[0].device
    return None


def find_teensy_ports():
    """ Check all USB devices for Teensys (by VID and PID) """
    return find_device_ports([TEENSY_VID], [TEENSY_SERIAL_PID, TEENSY_HID_PID, TEENSY_BOOT_PID])


def find_device_ports(VIDs, PIDs):
    """ Check USB ports for given VIDs / PIDs """

    # Call 'lsusb' and parse into array of dictionaries
    devices = get_lsusb()

    # Store the bus locations (for rebinding), and the hardware id
    uc_locations = []
    uc_ids = []
    uc_paths = []

    # Loop through all devices on from lsusb
    for _, dev in enumerate(devices):
        # Extract Vendor ID and Product ID
        vid, pid = [int(x, 16) for x in dev['id'].split(":")]
        # print("VID: {:04X}".format(vid))
        # print("PID: {:04X}".format(pid))

        if (vid in VIDs) and (pid in PIDs):

            # Call 'udevadm' to get complete information of the device
            cmd = "udevadm info {}".format(dev['device'])
            p = Popen([cmd], stdin=PIPE, stdout=PIPE, shell=True)
            stdout, _ = p.communicate()

            # Parse the output (string parsing, splits)
            for line in stdout.decode("utf-8").split('\n'):
                # Get the Bus location
                if "DEVPATH" in line:
                    bus = line.split("/")[-1]
                    uc_locations += [bus]
                    # print("Bus: ", bus)
                # Get the hardware ID
                if "ID_SERIAL_SHORT" in line:
                    hwid = line.split("=")[-1]
                    uc_ids += [hwid]
                    # print("Hwid: ", hwid)

            # Find corresponding path (e.g. /dev/ttyACM0)
            uc_paths += [get_port_number(uc_ids[-1])]


    return uc_ids, uc_locations, uc_paths


def get_lsusb():
    """ Call lsusb and parse output """

    # Get output of 'lsusb'
    df = subprocess.check_output("lsusb").decode("utf-8")

    # Regex Formula
    device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)

    # Loop through output, and parse out devices
    # Device info:
    #   'device': the location on the usb bus drive, e.g. Bus 002 Device 002 (format as '/dev/bus/usb/002/002')
    #   'id':     the Vendor ID and Product ID (format: 'VID:PID')
    #   'tag':    description of the device, e.g. 'Intel Corp'
    devices = [] # Array to populate w/ found devices
    for i in df.split('\n'):
        if i:
            info = device_re.match(i)
            if info:
                dinfo = info.groupdict()
                dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
                devices.append(dinfo)
    return devices
