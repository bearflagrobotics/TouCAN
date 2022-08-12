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

const uint8_t LED_PIN = 13;
bool led_state = 0;
uint32_t led_last_t;  // Log time of last LED toggle
uint32_t led_wait_time;  // How long between LED toggles (changes based on USB connection)

bool serial_up = false;

////////////////////////////////////////////////////////////////////////////
///                         Setup/Loop Functions                         ///
////////////////////////////////////////////////////////////////////////////

void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Setup Serial
    serial.begin(1);
    while (!serial) {  // Blocking wait for Serial port to open
        if (millis() - led_last_t > 2000) {
            led_last_t = millis();
            digitalWriteFast(LED_PIN, led_state);
            led_state = !led_state;  // Toggle LED
        }
    }
    serial_up = true;
    serial.println("Running 'can_driver_test.cpp'");

    serial.println("Initialization Complete.");
}


void loop() {

    // Read from CAN buses
    if (can0.ReadCan()) {
        serial.WriteData(can0.rx_msg_.raw, sizeof(can0.rx_msg_.fields));
    }
    if (can1.ReadCan()) {
        serial.WriteData(can1.rx_msg_.raw, sizeof(can1.rx_msg_.fields));
    }

    // // Write (Relay) CAN msgs (from Serial to CAN)
    // if (serial.available()) {
    //     // TODO: Develop SerialParser, so can send CAN smgs from Python script
    // }

    // Blink the LED (slow when serial down, fast when serial up)
    led_wait_time = serial.IsUp() ? 100 : 500;
    if (millis() - led_last_t > led_wait_time) {
        led_last_t = millis();
        digitalWriteFast(LED_PIN, led_state);  // Toggle LED
        led_state = !led_state;
    }

}
