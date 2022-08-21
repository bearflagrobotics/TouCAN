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

////////////////////////////
// Other useful functions //

void PrintStructHex(const CanMsg& can_msg) {
    Serial.print("Raw: ");
    for (uint i = 0; i < sizeof(can_msg.fields); ++i) {
        // Line below triggers compilation warning, but should be fine, since Union
        Serial.print(can_msg.raw[i], HEX); Serial.print(" ");
    }
    Serial.println();
}
void PrintCanData(const CanMsg& can_msg) {
    Serial.print("ID: "); Serial.print(can_msg.fields.msg.id, HEX);
    Serial.print(", Data: ");
    for (int i = 0; i < can_msg.fields.msg.len; ++i) {
        Serial.print(can_msg.fields.msg.buf[i], HEX); Serial.print(" ");
    }
    Serial.println();
}
uint32_t IdToPgn(uint32_t id) {
    // For Addressable messages, last 8 bits are used for destination address, thus not
    // included as PGN
    uint32_t pgn;
    uint8_t pf = (id >> 16) & 0xFF;  // PDU Format byte
    if (pf < 240) {
        pgn = (id >> 8) & 0x3FF00;
    } else {
        pgn = (id >> 8) & 0x3FFFF;
    }
    return pgn;
}
