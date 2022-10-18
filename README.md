# TouCAN Interface

## Hello, I am a TouCAN.
Why you ask? Because I have two CAN buses!!! Mwahahah

## Directory Structure
This repo includes the embedded code (runs on TouCAN Teensy), as well as Python files (runs on
Laptop).

For both sides (embedded and python), the main breakdown just has 2 modules: Serial, and CAN.

The Serial driver manages the communication protocol (think bytes processing, checksums, message types, etc). This should be a generalized interface for communication between laptop and Teensy.

The CAN driver actually manages the data (read from Serial) and parses it into CAN data (bus, ID, PGN, timestamp, etc).



### Embedded
This is a standard PlatformIO directory structure. The libraries are in `lib/`, with the
implementation scripts in `src/`. The "targets" are defined in `platformio.ini`.

The primary script here is `src/can_relay.cpp`, though there are others scripts in `src/tests/` that illustrated components of the libraries.

### Python
The core libraries/modules live in `py/src/`, while the scripts that use the modules just live in `py/`.
Ideally, for other use cases, the src file is sufficient for portability.

The main modules are

1) **SerialInterface.py**: handle bytes, basic comm protocl to Teensy, and threaded reading
2) **CanInterface.py**: parse the data from the Serial into CAN msgs/data

The Can Interface uses the Serial Interface to communicate with the TouCAN. Some of the test scripts use a "lead/follow" relationship, because it was tested with two TouCANs talking to each other (The only differences are the messages and timings, otherwise mostly copy-pasta)

## Communication Protocol
<details> <summary> details </summary>

**StartBytes**: All msgs start (eg. 0x00 0x55)

**MessageType**: Which defines the following format
- DataMsgType = 0x01      Used for raw data bytes passing (generic)
- StringMsgType = 0x02    Used for freeform string printing
- CanMsgType = 0x03       Used for CAN specific data

**DataMsgType**:
- Index   (1 byte)        Index the number of messages sent, for tracking any drops
- DataLen (1 byte)        Length of the data packet (# of bytes)
- Data    (DataLen bytes) The actual data
- Checksum (2 bytes)      Fletcher16 for the Data

**StringMsgType**
- Only criteria, is newline terminated

**CanMsgType**
- Index   (1 byte)                 Index the number of messages sent, for tracking any drops
- DataLen (1 byte)                 Length of the data packet (# of bytes)
- Data    (DataLen bytes)          The actual data
    - bus_id      (1 byte)                Which CAN bus
    - id          (4 bytes)               29-bit Extended CAN ID
    - data        (DataLen-5 bytes)       At most 8-bytes
- Checksum (2 bytes)      Fletcher16 for the Data

**eg Data Msg packets**
```
    START_BYTES MSG_TYPE INDEX  DATA_LEN   DATA                        CHKSM
    00 55       01       00     08         00 11 22 33 44 55 66 77     9D 4F <made up
    00 55       01       01     04         00 11 22 33                 C2 35 <made up
    00 55       01       02     06         00 11 22 33 44 55           4D EA <made
```

**eg String Msg packets**
```
   START_BYTES MSG_TYPE  StringMsg       Endline
   00 55       02        "hello world"   '\n'
   00 55       02        "test test"     '\n'
```

**eg CAN Msg packet**
```
   START_BYTES MSG_TYPE INDEX  DATA_LEN   BUS  EXT_ID       DATA                     CHKSM
                                               0x0CF00400
   00 55       01       00     08         00   00 04 0F 0C  F0 FF 94 90 1A FF FF FF  FA BC <made up
```

</details>


## TouCAN Usage

- TouCAN connections:
    - J1939 connector   (to Diagnostic port on tractor)
    - USB               (to computer)

- <details> <summary> Verify permissions on USB port </summary>
    - `ll /dev/tty*` and make sure rw permissions
        - `chmod a+rw /dev/ttyACM<number>`
    - udev rules (for Teensy, via PlatformIO)


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

- **Receive and Transmit CAN data!!!**
    - Flash Teensy with `CanRelayTest`
    - Run a script such as `can_interface_test_lead.py`
        - (This shows an example of how to transmit and receive CAN msgs, using the CanInterface (and SerialInterface) modules)
- <details> <summary> Testing CanInterface w/ Mock Tractor (uses 2 Teensys) </summary>

    - Use 2 Teensys (1 to mock the tractor, and 1 to actually be a TouCAN)
        - Flash one Teeny with `MockTractorCan`
        - Flash the other Teensy with `CanRelayTest`
    - Run a/the python script (such as `can_interface_test_lead.py`) that receives and transmits CAN messages as desired (make sure to connect to the proper Teensy, the one with `CanRelayTest`)
    - (Note: The other Teensy just needs power. Then it will transmit two messages periodically, and echo any received msgs with a different source address (eg. 0x12))

  </details>


### Python Scripts

- **read_can_only.py**
    - Purpose: Log CAN data to log file. (Uses CanInterface module)
    - Teensy Code: `CanRelayTest`

- **convert_asc_to_log.py** and  **convert_log_to_asc.py**
    - CLI Scripts for converting CAN log files from asc format (aka CAN Parse) to log files (aka can dump)

- **serial_can_test_lead/follow.py**
    - Purpose: Uses only the SerialInterface module to test functionality. (Happens to send CAN msg types)
    - Teensy Code: Two Teensys with `CanRelayTest`

- **can_interface_test_lead/follow.py**
    - Purpose: Full test of the CanInterface module, with both transmit and receive.
    - Teensy Code: Two Teensys with `CanRelayTest`


- <details> <summary> Some other things </summary>

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

  </details>

TODO:
- (todo)

## Requirements
- PlatformIO Core
- `pip install checksumdir pyserial`
-
