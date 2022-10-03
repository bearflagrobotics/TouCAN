/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  serial_driver_test.cpp
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 *
 */

#include <Arduino.h>

#include <serial_driver.h>

SerialDriver serial = SerialDriver();

const uint8_t LED_PIN = 13;
bool led_state = 0;
uint32_t last_led_t;  // Log time of last LED toggle
uint32_t led_wait_time;  // How long between LED toggles (changes based on USB connection)


uint8_t test_data[] = {0x00, 0x12, 0x34, 0x56};
uint32_t print_last_t;

////////////////////////////////////////////////////////////////////////////
///                         Setup/Loop Functions                         ///
////////////////////////////////////////////////////////////////////////////

void setup() {
    while (!Serial) {}
    serial.begin(1);

    pinMode(LED_PIN, OUTPUT);

    serial.PrintStringHeader();
    serial.println("Running serial_driver_test.cpp");
}

void loop() {
    // Write data periodically
    if (millis() - print_last_t > 1000) {
        test_data[0]++;
        serial.WriteData(test_data, sizeof(test_data));
        print_last_t = millis();

        // Print Strings
        // WARNING: Calling Serial.print() when Serial not up will freeze the uC
        if (serial.IsUp()) {
            serial.PrintStringHeader();
            serial.print("uC Tx: Hello World! ");
            serial.print(test_data[0]);
            serial.println();
        }
    }

    // Read data from Serial
    if (serial.Read()) {
        // // Echo back the bytes for verification
        // serial.WriteData(serial.data_buff, serial.data_len_exp);

        if (serial.IsUp()) {
            serial.PrintStringHeader();
            serial.print("uC Rx: ");
            for (int i = 0; i < serial.data_len_exp; ++i) {
                serial.print(serial.data_buff[i], HEX);
                serial.print(" ");
            }
            // serial.print("Hello back!");
            // serial.print(test_data[0]);
            serial.println();
        }

    }


    // Blink the LED (slow when serial down, fast when serial up)
    led_wait_time = serial.IsUp() ? 100 : 500;
    if (millis() - last_led_t > led_wait_time) {
        last_led_t = millis();
        digitalWriteFast(LED_PIN, led_state);
        led_state = !led_state;
    }
}
