#!/usr/bin/env python

"""
Copyright 2022 Bear Flag Robotics

    unbind_rebind_teensys.py

        Unfortunately, the Teensy toolchain does not support flashing when multiple Teensys
        are connected. Therefore, this script works around that flaw by unbinding and rebinding
        the corresponding ports of non-target Teensys to prevent errors.

        If an upload_port is specified (say with the --upload-port CLI tag), then this script
        will target that port.

        Otherwise, if the src code warrants a specific Teensy, this script will pull the Teensy ID
        from a config file (/var/tractor_config.yaml), and search for the target Teensy that way.

        Once the target Teensy is found, all other Teensy's on the bus are identified, and unbinded
        from the usb (via subprocess calls).

        After the target Teensy is flashed, the other Teensy's are rebinded.

        In order for the subprocess calls to succeed, the unbind/bind files must have proper
        permissions, which requires the following commands:
            sudo chown root:plugdev /sys/bus/usb/drivers/usb/{bind,unbind}
            sudo chmod g+w /sys/bus/usb/drivers/usb/{bind,unbind}
        These change the group to plugDev, and add write permissions to the files for plugDev.

        Expected output of >> ll /sys/bus/usb/drivers/usb/
                ...
            --w--w---- 1 root plugdev 4096 Apr  9 21:48 bind
            --w------- 1 root root    4096 Apr  9 13:29 uevent
            --w--w---- 1 root plugdev 4096 Apr  9 21:43 unbind
                ...



    Author: Austin Chun
    Date:   Apr 2020
"""
# Disable some pylint warnings
# pylint: disable=C0326, C0103

# from __future__ import print_function

from time import sleep
from subprocess import Popen, PIPE
import utils # BFR Embedded Utils

CFG_FILENAME = "/var/tractor_config.yaml" # <-- Edit this file for setting the teensy ID


Import("env") # Import the PlatformIO environment to get/set variables


# Global Variables
non_target_ids, non_target_locs = [], []

#####################################################
###                 Before Upload                 ###
#####################################################
def before_upload(source, target, env):
    """ Main function """

    global non_target_ids, non_target_locs

    print("------------------------------------------")
    print("--- Running 'unbind_rebind_teensys.py' ...")

    des_uc_id = None   # ID when Teensy in Serial
    des_boot_id = None # ID when Teensy in Bootloader
    des_loc = None     # USB Port Location of Target Teensy



    if 'UPLOAD_PORT' in env:
        des_uc_id = utils.check_port_for_teensy(env['UPLOAD_PORT'])
    else:
        try:
            des_uc_id, des_boot_id = read_config_file(env)
        except FileNotFoundError:
            pass

    # Get all Teensy ids, and corresponding locations
    all_ids, all_locs, all_paths = utils.find_teensy_ports()
    count_ucs = len(all_ids)

    # Check for Target uC
    for i, uc_id in enumerate(all_ids):
        # Found target uC
        if (uc_id == des_uc_id) or (uc_id == des_boot_id):
            des_loc = all_locs[i]
            print("  Found Target uC (ID={}) at loc={}".format(uc_id, des_loc))

            # Label all other IDs and Locs as non-target
            non_target_ids = all_ids[:i] + all_ids[i+1:]
            non_target_locs = all_locs[:i] + all_locs[i+1:]
            break


    # Deal with unspecified flash
    if des_uc_id is None:
        if count_ucs == 1:
            # No upload port specified, and not uc id specified, and only a single uc on connected
            # Do nothing, to just flash that uC
            print("  No ID nor upload port specified, and only 1 uC.")
            print("  Flashing to uC ID: {}".format(all_ids[0]))
            # Clear arrays, to make sure after_upload doesn't rebind them
            non_target_ids = []
            non_target_locs = []
            return
        elif count_ucs > 1:
            raise ValueError("No ID nor upload port specified, but there are multiple" + \
                             " possible targets. Please specify an ID or port. Failed to Flash.")


    # Found Teensy port, so unbind all other Teensys
    if des_loc is not None:
        # Unbind all other teensy's from usb
        for i, loc in enumerate(non_target_locs):
            print("    Unbinding port {} (ID={})".format(loc, non_target_ids[i]))
            unbind_port(loc)

    # If Teensy not found, raise Error
    else:
        # If no teensy devices found
        if count_ucs == 0:
            raise ValueError("No Teensys detected on USB. Failed to Flash.")
        # Single teensy found, but wrong ID
        elif count_ucs == 1:
            raise ValueError("No Teensy w/ ID={} found. Only 1 Teensy w/ ID={} on USB. Failed to Flash"
                             .format(des_uc_id, all_ids[0]))
        # If there are multiple teensy devices, it's ambiguous, and can lead to undesired flashing
        # so instead raise an error, and force user to either specify the port, or specify the ID
        else:
            print("  ERROR: No matching teensy ID found, and multiple teensy devices found, so quitting due to ambiguty.")
            raise ValueError("The Teensy ID={}".format(des_uc_id) + \
                             " was not found on the ports, and there are multiple Teensys" + \
                             " connected. Please check ID or upload port.")
    sleep(1)


####################################################
###                 After Upload                 ###
####################################################
def after_upload(source, target, env):
    """ Script to run after upload """
    sleep(1)
    # Rebind all other teensy's from usb
    for i, loc in enumerate(non_target_locs):
        print("    Rebinding port {} (ID={})".format(loc, non_target_ids[i]))
        bind_port(loc)


#########################################################
###                 Unbind/Bind Ports                 ###
#########################################################
def unbind_port(loc):
    """ Call subprocess to unbind a specified USB location"""
    bus = loc
    cmd = 'echo -n "{}" > /sys/bus/usb/drivers/usb/unbind'.format(bus)
    p = Popen([cmd], stdin=PIPE, stdout=PIPE, stderr=PIPE, shell=True)
    stdout, stderr = p.communicate()

    stdout = stdout.decode("utf-8")
    stderr = stderr.decode("utf-8")

    # Check for output
    if stdout:
        print("Stdout: ", end="")
        print(stdout)
    if stderr:
        print("Stderr: ", end="")
        print(stderr)
    # Check for permissions error
    if "Permission denied" in stderr:
        print("Failed to unbind port. Check permissions on file")
        print("Make sure the group is plugDev, and plugDev has write permissions.")
        print("May need to run: $ sudo chown root:plugdev /sys/bus/usb/drivers/usb/{bind,unbind}")
        print("                 $ sudo chmod g+w /sys/bus/usb/drivers/usb/{bind,unbind}")
        raise ValueError("Failed to unbind port. Check permissions.")


def bind_port(loc):
    """ Call subprocess to rebind a specified USB location"""
    arr = loc.split(":")
    bus = arr[0]
    cmd = 'echo -n "{}" > /sys/bus/usb/drivers/usb/bind'.format(bus)
    Popen([cmd], stdin=PIPE, stdout=PIPE, stderr=PIPE, shell=True)


########################################################
###                 Helper Functions                 ###
########################################################

def read_config_file(env):
    """ Parse the YAML config fiel for the desired IDs """

    # Open the YAML config file, to read in User settings
    data = utils.read_yaml_config_file(CFG_FILENAME)
    print("  Read cfg_file: {}".format(CFG_FILENAME))

    # Pull the ID from the yaml dict, based on which environment is being called
    pioenv = env['PIOENV'] # check the PIO environment
    if pioenv == '8RT_main':
        des_uc_id = data['Embedded']['MainuC']['port_id']
        des_boot_id = data['Embedded']['MainuC']['boot_id']
    elif pioenv == 'CANReader':
        des_uc_id = data['CANReader']['port_id']
        des_boot_id = data['CANReader']['boot_id']
    elif pioenv == 'TestbenchDUT':
        des_uc_id   = data['Embedded']['DUT']['port_id']
        des_boot_id = data['Embedded']['DUT']['boot_id']
    elif pioenv == 'TestbenchBCH':
        des_uc_id   = data['Embedded']['BCH']['port_id']
        des_boot_id = data['Embedded']['BCH']['boot_id']
    else:
        des_uc_id = None # No specified uC
        des_boot_id = None

    print("  Looking for uC id={} (boot_id={})".format(des_uc_id, des_boot_id))

    return des_uc_id, des_boot_id



# Run the function
env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)
