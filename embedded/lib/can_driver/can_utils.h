/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  can_utils.h
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 */

#pragma once

#include <Arduino.h>  // Core library

#include <FlexCAN.h>

// Simple Union to allow easy read/write of CAN msg struct
union CanMsg {
    // The CAN Message struct
    struct Fields {
        uint16_t ind;       // Track msg number (recevied by uC)
        uint8_t bus_id;     // Which bus is this on
        uint8_t  : 8;       // Byte padding, for 32-bit words
        uint32_t tstamp;    // microseconds since launch
        CAN_message_t msg;  // CAN_message_t struct for reference
                            // typedef struct CAN_message_t {
                            //   uint32_t id; // can identifier
                            //   uint8_t ext; // identifier is extended
                            //   uint8_t len; // length of data
                            //   uint16_t timeout; // milliseconds, zero will disable waiting
                            //   uint8_t buf[8];
                            // } CAN_message_t;
    } fields;
    // Raw Data
    uint8_t raw[];
};


void PrintStructHex(const CanMsg& can_msg);
void PrintCanData(const CanMsg& can_msg);
uint32_t IdToPgn(uint32_t id);
