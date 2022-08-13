/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  can_driver_test.cpp
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 */

#include <Arduino.h>  // Core library

#include <serial_driver.h>
#include <can_driver.h>

SerialDriver serial = SerialDriver();

// Create CAN Bus object
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
bool led_state = 1;
uint32_t led_last_t;  // Log time of last LED toggle
uint32_t led_wait_time;  // How long between LED toggles (changes based on USB connection)

bool serial_up = false;

////////////////////////////////////////////////////////////////////////////
///                         Setup/Loop Functions                         ///
////////////////////////////////////////////////////////////////////////////

void setup() {
    pinMode(LED_PIN, OUTPUT);
}


void loop() {

    // Periodically write some CAN msgs to CAN bus
    if (millis() - last_EEC2_msg_t > 500) {
        last_EEC2_msg_t = millis();
        can0.WriteCan(EEC1_msg);
        can1.WriteCan(EEC2_msg);
    }

    // Read from CAN buses
    if (can0.ReadCan()) {
        serial.WriteData(can0.rx_msg_.raw, sizeof(can0.rx_msg_.fields));
    }
    if (can1.ReadCan()) {
        serial.WriteData(can1.rx_msg_.raw, sizeof(can1.rx_msg_.fields));
    }

    // Read data from Serial
    if (serial.Read()) {
        // Echo back the bytes for verification
        // serial.WriteData(serial.data_buff, serial.data_len_exp);

        if (serial.IsUp()) {
            serial.PrintStringHeader();
            Serial.print("uC Rx: ");
            for (int i = 0; i < serial.data_len_exp; ++i) {
                Serial.print(serial.data_buff[i], HEX);
                Serial.print(" ");
            }
            // Serial.print("Hello back!");
            // Serial.print(test_data[0]);
            Serial.println();
        }

    }

    // // // Write (Relay) CAN msgs (from Serial to CAN)
    // // if (serial.available()) {
    // //     // TODO: Develop SerialParser, so can send CAN smgs from Python script
    // // }

    // Blink the LED (slow when serial down, fast when serial up)
    led_wait_time = serial.IsUp() ? 100 : 500;
    if (millis() - led_last_t > led_wait_time) {
        led_last_t = millis();
        digitalWriteFast(LED_PIN, led_state);  // Toggle LED
        led_state = !led_state;
    }

}
