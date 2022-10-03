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

CAN_message_t tx_msg;

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

    // Setup Serial
    serial.begin(1);
    while (!serial.IsUp()) {  // Blocking wait for Serial port to open
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
        serial.WriteData(can0.rx_msg_.raw, sizeof(can0.rx_msg_.fields), true);
    }
    if (can1.ReadCan()) {
        serial.WriteData(can1.rx_msg_.raw, sizeof(can1.rx_msg_.fields), true);
    }

    // Read data from Serial
    if (serial.Read()) {
        // Echo back the bytes for verification
        // serial.WriteData(serial.data_buff, serial.data_len_exp);

        // Echo back Serial
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

        // Echo CAN data to can bus
        if (serial.msg_type == serial.kDataMsgType) {
            tx_msg.id = serial.data_buff[3] << 24 |
                        serial.data_buff[2] << 16 |
                        serial.data_buff[1] <<  8 |
                        serial.data_buff[0] <<  0;

            tx_msg.ext = 1;
            tx_msg.len = serial.data_len_exp - 4;  // Total msg length, minus ID
            tx_msg.timeout = 0;
            memcpy(tx_msg.buf, &serial.data_buff[4], tx_msg.len);

            can0.WriteCan(tx_msg);

            if (serial.IsUp()) {
                serial.PrintStringHeader();
                serial.print("tx_msg.id: ");
                serial.print(tx_msg.id, HEX);
                serial.print(", tx_msg.len: ");
                serial.print(tx_msg.len, HEX);
                serial.println();
            }

        }

    }

    // Blink the LED (slow when serial down, fast when serial up)
    led_wait_time = serial.IsUp() ? 100 : 500;
    if (millis() - led_last_t > led_wait_time) {
        led_last_t = millis();
        digitalWriteFast(LED_PIN, led_state);  // Toggle LED
        led_state = !led_state;
    }

}
