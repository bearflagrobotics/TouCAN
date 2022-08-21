/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  serial_driver.h
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 */

#pragma once

#include <Arduino.h>  // Core library

class SerialDriver : public usb_serial_class {
 public:

    // Comm Protocol Message Structure
    //
    // All msgs start with StartBytes (eg. 0x00 0x55)
    // Next is the Message Type, which defines the following format
    //     DataMsgType = 0x01      Used for raw data bytes passing (generic)
    //     StringMsgType = 0x02    Used for freeform string printing
    //     CanMsgType = 0x03       Used for CAN specific data

    // DataMsgType
    //     Index   (1 byte)        Index the number of messages sent, for tracking any drops
    //     DataLen (1 byte)        Length of the data packet (# of bytes)
    //     Data    (DataLen bytes) The actual data
    //     Checksum (2 bytes)      Fletcher16 for the Data
    // StringMsgType
    //     Only criteria, is newline terminated
    // CanMsgType
    //     Index   (1 byte)                 Index the number of messages sent, for tracking any drops
    //     DataLen (1 byte)                 Length of the data packet (# of bytes)
    //     Data    (DataLen bytes)          The actual data
    //          bus_id      (1 byte)                Which CAN bus
    //          id          (4 bytes)               29-bit Extended CAN ID
    //          data        (DataLen-5 bytes)       At most 8-bytes
    //     Checksum (2 bytes)      Fletcher16 for the Data

    // eg Data Msg packets
    //    START_BYTES MSG_TYPE INDEX  DATA_LEN   DATA                        CHKSM
    //    00 55       01       00     08         00 11 22 33 44 55 66 77     9D 4F <made up
    //    00 55       01       01     04         00 11 22 33                 C2 35 <made up
    //    00 55       01       02     06         00 11 22 33 44 55           4D EA <made

    // eg String Msg packets
    //    START_BYTES MSG_TYPE  StringMsg       Endline
    //    00 55       02        "hello world"   '\n'
    //    00 55       02        "test test"     '\n'

    // eg CAN Msg packet
    //    START_BYTES MSG_TYPE INDEX  DATA_LEN   BUS  EXT_ID       DATA                     CHKSM
    //                                                0x0CF00400
    //    00 55       01       00     08         00   00 04 0F 0C  F0 FF 94 90 1A FF FF FF  FA BC <made up


    ///////////////////////////
    ///      Functions      ///
    ///////////////////////////

    bool WriteData(const uint8_t* data, const uint8_t data_len);

    bool IsUp();

    bool Read();

    uint16_t Fletcher16(const uint8_t* data, int32_t count);

    bool VerifyChecksum(const uint8_t* data, uint8_t data_len, uint16_t chksm_rcvd);

    bool PrintStringHeader();

    ///////////////////////////
    ///      Constants      ///
    ///////////////////////////
    const uint8_t kStartBytes[2] = {0x00, 0x55};  // (Nod to LIN protocol)
    static const uint8_t kNumStartBytes = sizeof(kStartBytes);


    const uint8_t kDataMsgType = 0x01;
    const uint8_t kStringMsgType = 0x02;
    const uint8_t kCanMsgType = 0x03;

    ///////////////////////////
    ///      Variables      ///
    ///////////////////////////
    uint8_t msg_idx;  // Track

    bool is_up;  // Flag


    enum ReadState {
        READ_START,
        READ_TYPE,
        READ_IDX,
        READ_LEN,
        READ_DATA,
        READ_CHKSM,
        READ_STRING
    } read_state;                        // Simple state machine for serial parsing

    uint8_t start_bytes_i = 0;

    uint8_t msg_type;

    static const uint8_t kMaxDataLen = 255;  // Max data msg frame length
    uint8_t data_buff[kMaxDataLen];
    uint8_t data_buff_i = 0;

    uint8_t read_msg_index;
    uint8_t data_len_exp;

    static const uint8_t kChksmLen = 2;
    uint8_t chksm_i;
    uint8_t chksm[kChksmLen];

    static const uint8_t kStringBufferMAxLen = 100;
    uint8_t string_buffer[kStringBufferMAxLen];
    uint8_t string_buffer_i = 0;



//  private:
};
