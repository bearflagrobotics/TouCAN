# TouCAN Interface

## Hello, I am a TouCAN.
Why you ask? Because I have two CAN buses!!! Mwahahah

## Directory Structure
This repo includes the embedded code (runs on TouCAN Teensy), as well as Python files (runs on
Laptop).

### Embedded
This is a standard PlatformIO directory structure. The libraries are in `lib/`, with the
implementation scripts in `src/`. The "targets" are defined in `platformio.ini`.

The primary script here is `src/can_relay.cpp`, though there are others scripts in `src/tests/` that illustrated components of the libraries.

### Python Scripts
There are a handful of scripts, and a handful of libraries/modules.

The main modules are

1) **SerialInterface.py**: handle bytes, basic comm protocl to Teensy, and threaded reading
2) **CanInterface.py**: parse the data from the Serial into CAN msgs/data

The Can Interface uses the Serial Interface to communicate with the TouCAN. There are scripts that
use these modules, for different use cases, commonly a "lead" and "follow"

#### Serial Interface
Interface

#### Can Interface
Interface

## TouCAN Usage

- TouCAN connections:
    - J1939 connector   (to Diagnostic port on tractor)
    - USB               (to computer)

- <details> <summary> Verify permissions on USB port </summary>

    - `ll /dev/tty*` and make sure rw permissions
        - `chmod a+rw /dev/ttyACM<number>`
  </details>

- <details> <summary> Flash multiple Teensys permissions (optional) </summary>

    - Check permissions on file
        - `ll /sys/bus/usb/drivers/usb/*bind`
        - Good permissions (assigned plugDev group, and has write permissions):
            ```
            austin@PC27SKGN:~$ ll /sys/bus/usb/drivers/usb/*bind
            --w--w---- 1 root plugdev 4096 Sep 29 12:17 /sys/bus/usb/drivers/usb/bind
            --w--w---- 1 root plugdev 4096 Sep 29 12:17 /sys/bus/usb/drivers/usb/unbind
            ```
        - Bad permissions:
            ```
            austin@PC27SKGN:~$ ll /sys/bus/usb/drivers/usb/*bind
            --w------- 1 root root 4096 Sep 29 12:12 /sys/bus/usb/drivers/usb/bind
            --w------- 1 root root 4096 Sep 29 12:10 /sys/bus/usb/drivers/usb/unbind
            ```
    - Fix the permissions, run these commands
        ```
        sudo chown root:plugdev /sys/bus/usb/drivers/usb/{bind,unbind}
        sudo chmod g+w /sys/bus/usb/drivers/usb/{bind,unbind}
        ```
    - Recheck permissions (see above)
  </details>

- Run **CANReader.py** (only reading CAN)
    - `python CANReader.py`
    - Output
        - Logging info prints to consol
        - CAN data logs to a timestamped *.asc file
            - Matches the CAN Parse format, and can be directly dropped in
    - Options:
        - `-p <port>` (say if multiple Teensys plugged in)
            - `python CANReader.py -p /dev/ttyACM0`
        - `-v` verbose for timestamps and filename on log lines
            - Normal: `INFO: CANReader: Initialized.`
            - Verbose: `2022-08-08 18:00:31,455 CANReader.py INFO: CANReader: Initialized.`
        - `-c` color for colored log output


- Teensy as **CAN Relay** (read and write to CAN)
    - Purpose: Teensy will read CAN bus, and publish over Serial, and the Teensy can relay (write)
        messages from USB serial to the CAN bus
    - Flash Teensy with `CanRelayTest`
    - Run python script [TBD]

    - To test with two TouCANs for testing
        - Flash both with `CanRelay`
            - `cd ~/TouCAN/embedded/`
            - `pio run -e CanRelay -t upload --upload-port /dev/ttyACM0`
            - `pio run -e CanRelay -t upload --upload-port /dev/ttyACM1`
        - Run 'leader' python script with port specified
            - `py lead_serial_can_test.py -p /dev/ttyACM0 -c -d -v`
        - Run  'follower' python script with port specified
            - `py follow_serial_can_test.py -p /dev/ttyACM1 -c -d -v`
        - Should see traffic on both sides now (TX and RX)

- Two TouCANs, with CanInterface
    - Flash both Teensy's with `CanRelayTest`
    - Run two lead/follow of can_interface_test
        - `py lead_can_interface_test.py -p /dev/ttyACM0 -v -c -d`
        - `py follow_can_interface_test.py -p /dev/ttyACM1 -c -d -v`

TODO:
- (todo)

## Requirements
- PlatformIO Core
- `pip install checksumdir`
-
