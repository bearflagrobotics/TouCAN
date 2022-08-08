"""
Copyright 2022 Bear Flag Robotics

    convert_log_to_asc.py

    Author: Austin Chun
    Date:   Aug 2022
"""

import sys, os
import argparse

### Input definitions
##
LOG_DIR = "logs/"
LOG_FILENAME = "5R_AutoTracEngaged_TouCAN.log" # <== Edit this file!

## Define relative paths
ASC_FILENAME = "".join(LOG_FILENAME.split(".")[:-1]) + ".asc"
LOG_FILENAME = LOG_DIR + LOG_FILENAME
ASC_FILENAME = LOG_DIR + ASC_FILENAME

def main():

    ## Parse CLI input
    if not len(sys.argv) > 1:
        print("  Error: Needs an input file")
        print("  Usage: python convert_log_to_asc.py logs/test.log")
        print("         (Saves output file with same name/dir but different extension)")
        return
    # Argparser
    parser = argparse.ArgumentParser()
    parser.add_argument('logfile', help="Input log file name", type=str)
    args = parser.parse_args()
    # Verify args
    if os.path.splitext(args.logfile)[1] != '.log':
        print(f"  Error: Input file is not an .log extension. Rcvd: {args.logfile}")
        return
    # Condition Args
    log_filename = args.logfile
    asc_filename = os.path.splitext(log_filename)[0] + '.asc'

    # Load input
    print(f"  Reading '{log_filename}'...")
    with open(log_filename, 'r', encoding='utf-8') as infile:
        lines = infile.readlines()

    # Trim beginning and end lines
    lines = lines[:-1]
    # lines = lines[:10]

    # Create output
    print(f"  Writing '{asc_filename}'...")
    with open(asc_filename, 'w', encoding='utf-8') as outfile:

        # Write the header (random dates)
        header_str = \
            "date Mon Jan 1 00:00:00 AM 2000\n" +\
            "base hex timestamps absolute\n" +\
            "no internal events logged\n" +\
            "// version 11.0.0\n" +\
            "Begin TriggerBlock Mon Jan 1 00:00:00 AM 2000\n" +\
            "   0.000000 Start of measurement\n"
        outfile.write(header_str)

        # Write the data
        tstart = None
        for line in lines:
            words = line.strip().split(" ")

            # Handle timestamp
            tcur = float(words[0][1:-1])
            if tstart is None:
                tstart = tcur

            trel = tcur - tstart # Adjust to relative time

            # Get strings
            tstamp = f"{trel:.6f}".rjust(11, ' ')
            bus = int(words[1][-1] )+1
            ID, data = words[2].split("#")
            data_len = int(len(data)/2)

            # Add spaces between bytes
            data_split = " ".join(data[i:i+2] for i in range(0, len(data), 2))

            # eg. 0.103862 1 18EFFF07x  Rx d 8 F0 0D FF FF FF FF FF 0F
            outline = f"{tstamp} {bus} {ID}x Rx d {data_len} {data_split}\n"

            outfile.write(outline)

        # Write the footer
        outfile.write("End TriggerBlock\n")

    return

# ==================================================================================================

if __name__ == '__main__':
    main()
