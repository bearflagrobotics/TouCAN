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
    ///////////////////////////
    ///      Functions      ///
    ///////////////////////////

    bool WriteData(const uint8_t* data, const uint8_t data_len);

    bool IsUp();


    ///////////////////////////
    ///      Variables      ///
    ///////////////////////////
    uint8_t msg_idx;  // Track

    bool is_up;  // Flag

//  private:
    ///////////////////////////
    ///      Constants      ///
    ///////////////////////////
    const char kSTART_BYTE = 0x00;
    const char kSYNCH_BYTE = 0x55;

    ///////////////////////////
    ///      Variables      ///
    ///////////////////////////

    ///////////////////////////
    ///      Functions      ///
    ///////////////////////////

    uint16_t Fletcher16(const uint8_t* data, int32_t count);

};
