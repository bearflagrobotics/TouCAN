"""
Copyright 2022 Bear Flag Robotics

    convert_asc_to_log.py

    Author: Austin Chun
    Date:   Aug 2022
"""

import sys, os
import argparse

def main():

    ## Parse CLI input
    if not len(sys.argv) > 1:
        print("  Error: Needs an input file")
        print("  Usage: python convert_asc_to_log.py logs/test.asc")
        print("         (Saves output file with same name/dir but different extension)")
        return
    # Argparser
    parser = argparse.ArgumentParser()
    parser.add_argument('ascfile', help="Input asc file name", type=str)
    args = parser.parse_args()
    # Verify args
    if os.path.splitext(args.ascfile)[1] != '.asc':
        print(f"  Error: Input file is not an .asc extension. Rcvd: {args.ascfile}")
        return
    # Condition Args
    asc_filename = args.ascfile
    log_filename = os.path.splitext(asc_filename)[0] + '.log'

    # Load input
    print(f"  Reading '{asc_filename}'...")
    with open(asc_filename, 'r', encoding='utf-8') as infile:
        lines = infile.readlines()

    # Trim beginning and end lines
    lines = lines[6:-2]

    # Create output
    print(f"  Writing '{log_filename}'...")
    with open(log_filename, 'w', encoding='utf-8') as outfile:
        # Loop through each line
        for line in lines:
            words = line.strip().split(" ")
            # eg. (001266.497) can1 18FFFFF0#2700FCFFFFFFFFFF
            outline = f"({words[0]}) can{words[1]} {words[2][:-1]}#{''.join(words[6:])}\n"
            outfile.write(outline)

    return

# ==================================================================================================

if __name__ == '__main__':
    main()
