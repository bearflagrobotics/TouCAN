/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  mock_tractor_can.cpp
 *
 *      Simulate some random CAN traffic, to test as the Tractor CAN
 *
 *      Periodic transmit of specific message, and echo back any received CAN msgs with separate
 *      source address (say 0x12)
 *
 *      Intended to be run headless (w/o Serial connection)
 *
 *      Author: Austin Chun
 *      Date:   Oct 2022
 */

#include <Arduino.h>  // Core library

#include <can_driver.h>

bool serial_up = false;  // Flag used for some LED diagnostics

// Create CAN Bus object
CanDriver can0 = CanDriver(500000, 0);  // Standard Baud of Vehicle Bus
CanDriver can1 = CanDriver(250000, 1);  // Standard Baud of Implement Bus (ISOBUS)


uint32_t last_tx_t;      // Log when last transmitted (for periodic transmit)
CAN_message_t tx_msg_0 {
    0x01234567,  // ID
    1,           // Ext
    8,           // Len
    0,           // Timeout
    {0x11, 0x22, 0x33, 0x44, 0x12, 0x34, 0x43, 0x21}  // Data
};

// FEF3: GPS Location
//      1.138000 2 0CFEF31Cx Rx d 8 69 23 8C 93 37 0B 6B 34
CAN_message_t tx_msg_1 { // GPS Location
    0x0CFEF31C,  // ID
    1,           // Ext
    8,           // Len
    0,           // Timeout
    {0x69, 0x23, 0x8C, 0x93, 0x37, 0x0B, 0x6B, 0x34}  // Data
};


CAN_message_t rx_echo_msg;  // Echoed CAN msgs with changed source address
const uint8_t CAN_SOURCE_ADDRESS = 0x12;
const uint8_t MAX_READ_LOOP = 50;  // Max number of loops to read CAN bus

// LED Variables
const uint8_t LED_PIN = LED_BUILTIN;  // Built in on Teensy (Pin 13 for T3.6)
bool led_state = 1;
uint32_t led_last_t;  // Log time of last LED toggle
uint32_t led_wait_time;  // How long between LED toggles (changes based on USB connection)


////////////////////////////////////////////////////////////////////////////
///                         Setup/Loop Functions                         ///
////////////////////////////////////////////////////////////////////////////

void setup() {
    pinMode(LED_PIN, OUTPUT);  // Setup LED
    digitalWriteFast(LED_PIN, led_state);

    // Setup Serial
    Serial.begin(1);
    if (Serial) {
        serial_up = true;
        Serial.println("Running 'mock_tractor_can.cpp'");
    }
}


void loop() {

    // Periodically transmit msgs
    if (millis() - last_tx_t > 1000) {
        last_tx_t = millis();
        can0.WriteCan(tx_msg_0);
        can1.WriteCan(tx_msg_1);
        tx_msg_0.buf[0]++;
    }

    // Read CAN bus, re-transmit with changed source address
    for (int i = 0; i < MAX_READ_LOOP; ++i) {
        if (can0.ReadCan()) {
            // Copy all data
            memcpy(&rx_echo_msg, &can0.rx_msg_.fields.msg, sizeof(CAN_message_t));
            // Change source address
            rx_echo_msg.id = (rx_echo_msg.id & 0x1FFFFF00) | CAN_SOURCE_ADDRESS;
            can0.WriteCan(rx_echo_msg);
        }
        if (can1.ReadCan()) {
            // Copy all data
            memcpy(&rx_echo_msg, &can1.rx_msg_.fields.msg, sizeof(CAN_message_t));
            // Change source address
            rx_echo_msg.id = (rx_echo_msg.id & 0x1FFFFF00) | CAN_SOURCE_ADDRESS;
            can1.WriteCan(rx_echo_msg);
        }
    }

    // Blink the LED (slow when serial down, fast when serial up)
    led_wait_time = (Serial) ? 50 : 200;
    if (millis() - led_last_t > led_wait_time) {
        led_last_t = millis();
        digitalWriteFast(LED_PIN, led_state);  // Toggle LED
        led_state = !led_state;
    }
}
