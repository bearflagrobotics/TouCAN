# TouCAN Interface

## Hello, I am a TouCAN.
Why you ask? Because I have two CAN buses!!! Mwahahah

## TouCAN Usage

- TouCAN connections:
    - J1939 connector   (to Diagnostic port on tractor)
    - USB               (to computer)

- Verify permissions on USB port
    - `ll /dev/tty*` and make sure rw permissions
        - `chmod a+rw /dev/ttyACM<number>`

- Run CANReader.py
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

