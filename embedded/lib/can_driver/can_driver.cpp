/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  can_driver.cpp
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 */

#include <can_driver.h>

#include <Arduino.h>

#include <FlexCAN.h>
#include <can_utils.h>


CanDriver::CanDriver(uint32_t baud, uint8_t id, uint8_t tx_alt, uint8_t rx_alt)
    : can_(baud, id, tx_alt, rx_alt), bus_id_(id)
{
    can_.begin();
}


bool CanDriver::ReadCan() {
    if (can_.available()) {
        // Read in the CAN msg
        can_.read(rx_msg_.fields.msg);
        // Populate metadata
        rx_msg_.fields.ind++;  // Increment count
        rx_msg_.fields.bus_id = bus_id_;
        rx_msg_.fields.tstamp = micros();  // timestamp msg receipt
        last_can_msg_t = millis();  // Log last time, to check for timeouts
        return true;
    }
    return false;
}

void CanDriver::WriteCan(const CAN_message_t& can_msg) {
    can_.write(can_msg);
}
