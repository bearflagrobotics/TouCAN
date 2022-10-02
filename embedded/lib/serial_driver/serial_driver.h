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
    //  See README

    ///////////////////////////
    ///      Functions      ///
    ///////////////////////////

    bool WriteData(const uint8_t* data, const uint8_t data_len, bool is_can = false);

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
