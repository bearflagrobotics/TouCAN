/**
 *  Copyright 2020 Bear Flag Robotics
 *
 *  CAN_master_mock.cpp
 *
 *      Spoof some CAN messages, to simulate 6130 diagnostics port.
 *      This uses the FlexCAN library for the Teensy 3.6
 *
 *      Author: Austin Chun
 *      Date:   Apr 2020
 *
 */

#include <Arduino.h>

#include <FlexCAN.h>


#define CAN_BAUD    500000
#define CAN_CH      0
#define CAN_RX_ALT  0
#define CAN_TX_ALT  0


// FlexCAN can = FlexCAN(CAN_BAUD, CAN_CH, CAN_TX_ALT, CAN_RX_ALT);
FlexCAN can0 = FlexCAN(500000, 0);
FlexCAN can1 = FlexCAN(250000, 1);

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
bool LED_STATE = 0;
uint32_t last_led_toggle_t;


////////////////////////////////////////////////////////////////////////////
///                         Setup/Loop Functions                         ///
////////////////////////////////////////////////////////////////////////////

void setup() {
    // Serial.begin(1);
    // // delay(1000);
    // while (!Serial) {}
    // Serial.println("Running 'CAN_master_mock'");

    can0.begin();  // No mask, let everything through
    can1.begin();  // No mask, let everything through

    pinMode(LED_PIN, OUTPUT);

    // Serial.println("Initialization Complete.");
}


void loop() {

    // can.write(EEC1_msg);

    if (millis() - last_EEC2_msg_t > 100) {
        last_EEC2_msg_t = millis();
        can0.write(EEC1_msg);
        // can0.write(EEC2_msg);
        // Serial.println("Wrote EEC2.");
    // } else {
        // delay(2);
        // can1.write(EEC1_msg);
        can1.write(EEC2_msg);
        // last_EEC2_msg_t = millis();

        // Serial.println("Wrote EEC1.");
    }

    if (millis() - last_led_toggle_t > 500) {
        last_led_toggle_t = millis();
        digitalWriteFast(LED_PIN, LED_STATE);
        LED_STATE = !LED_STATE;
        Serial.println("LED");
    }



    // delay(10);
    // while (can1.available()) {
    //     can1.read(rx_msg);
    //     Serial.print("ID: "); Serial.println(rx_msg.id);
    // }

}
