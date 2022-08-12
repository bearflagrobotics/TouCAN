/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  can_driver.h
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 */

#pragma once

#include <Arduino.h>  // Core library

#include <FlexCAN.h>
#include <can_utils.h>

class CanDriver {

 public:
    ///////////////////////////
    ///      Functions      ///
    ///////////////////////////
    CanDriver(uint32_t baud, uint8_t id, uint8_t tx_alt = 0, uint8_t rx_alt = 0);

    bool ReadCan();
    void WriteCan(const CAN_message_t& can_msg);

    ///////////////////////////
    ///      Variables      ///
    ///////////////////////////

    FlexCAN can_;

    uint8_t msg_idx_;

    CanMsg rx_msg_;
    CanMsg tx_msg_;

    uint16_t chksm_;

    uint8_t bus_id_;

    uint32_t last_can_msg_t;

    ///////////////////////////
    ///      Constants      ///
    ///////////////////////////

 private:

};
