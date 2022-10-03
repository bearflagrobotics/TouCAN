/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  can_writer_test.cpp
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 *
 */

#include <Arduino.h>

#include <can_driver.cpp>


CanDriver can0 = CanDriver(500000, 0);
CanDriver can1 = CanDriver(250000, 1);

CAN_message_t EEC1_msg {
    0x0CF00400,  // ID
    1,           // Ext
    8,           // Len
    0,           // Timeout
    {0xF0, 0xFF, 0x94, 0x90, 0x1A, 0xFF, 0xFF, 0xFF}  // Data
};

CAN_message_t EEC2_msg {
    0x0CF00300,  // ID
    1,           // Ext
    8,           // Len
    0,           // Timeout
    {0xFF, 0xFE, 0x1C, 0xFF, 0xFF, 0xFF, 0xC3, 0xFF}  // Data
};

uint32_t last_EEC2_msg_t;

const uint8_t LED_PIN = 13;
bool led_state = 0;
uint32_t last_led_t;  // Log time of last LED toggle
uint32_t led_wait_time;  // How long between LED toggles (changes based on USB connection)

bool serial_up = false;

////////////////////////////////////////////////////////////////////////////
///                         Setup/Loop Functions                         ///
////////////////////////////////////////////////////////////////////////////

void setup() {
    // can0.begin();  // No mask, let everything through
    // can1.begin();  // No mask, let everything through
    pinMode(LED_PIN, OUTPUT);
}


void loop() {

    // Serial just closed
    if (serial_up && !Serial) {
        Serial.begin(1);
        serial_up = false;
    }
    if (Serial) serial_up = true;


    // Periodically write some CAN msgs to CAN bus
    if (millis() - last_EEC2_msg_t > 100) {
        last_EEC2_msg_t = millis();
        can0.WriteCan(EEC1_msg);
        can1.WriteCan(EEC2_msg);
    }

    // Blink the LED (slow when serial down, fast when serial up)
    led_wait_time = serial_up ? 100 : 500;
    if (millis() - last_led_t > led_wait_time) {
        last_led_t = millis();
        digitalWriteFast(LED_PIN, led_state);
        led_state = !led_state;
    }

}
