/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  can_relay.cpp
 *
 *      Relay CAN data to/from USB Serial from/to CAN bus
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 */

#include <Arduino.h>  // Core library

#include <serial_driver.h>
#include <can_driver.h>

// Serial Driver: Interface to USB, wraps the comms protocols
SerialDriver serial = SerialDriver();
bool serial_up = false;  // Flag used for some LED diagnostics

// Create CAN Bus object
CanDriver can0 = CanDriver(500000, 0);  // Standard Baud of Vehicle Bus
CanDriver can1 = CanDriver(250000, 1);  // Standard Baud of Implement Bus (ISOBUS)

CAN_message_t tx_msg;  // Object for storing message for writing to CAN bus
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

    // Setup Serial
    serial.begin(1);
    while (!Serial) {  // Blocking wait for Serial port to open
        if (millis() - led_last_t > 2000) {
            led_last_t = millis();
            digitalWriteFast(LED_PIN, led_state);
            led_state = !led_state;  // Toggle LED
        }
    }
    serial_up = true;
    serial.PrintStringHeader();
    serial.println("Running 'can_relay.cpp'");
}


void loop() {

    // Read CAN bus, write to Serial
    for (int i = 0; i < MAX_READ_LOOP; ++i) {
        if (can0.ReadCan()) {
            serial.WriteData(can0.rx_msg_.raw, sizeof(can0.rx_msg_.fields));
        }
        if (can1.ReadCan()) {
            serial.WriteData(can1.rx_msg_.raw, sizeof(can1.rx_msg_.fields));
        }
    }

    // Read Serial, write to CAN bus
    if (serial.Read()) {
        // CAN Msg Type: Echo CAN data to can bus
        if (serial.msg_type == serial.kCanMsgType) {
            // Extract data from msg
            uint8_t bus_id = serial.data_buff[0];
            tx_msg.id = serial.data_buff[4] << 24 |
                        serial.data_buff[3] << 16 |
                        serial.data_buff[2] <<  8 |
                        serial.data_buff[1] <<  0;
            tx_msg.ext = 1;
            tx_msg.len = serial.data_len_exp - 5;  // Total msg length, minus ID
            tx_msg.timeout = 0;
            memcpy(tx_msg.buf, &serial.data_buff[5], tx_msg.len);
            // Send the data
            if (bus_id == 0) {
                can0.WriteCan(tx_msg);
            } else if (bus_id == 1) {
                can1.WriteCan(tx_msg);
            } else {
                serial.PrintStringHeader();
                Serial.print("ERROR: Unknown bus_id: 0x");
                Serial.print(bus_id, HEX);
                Serial.println();
            }
        }
        // Data Msg Type: Nothing
        // String Msg Type: Nothing
    }


    // Blink the LED (slow when serial down, fast when serial up)
    led_wait_time = serial.IsUp() ? 100 : 500;
    if (millis() - led_last_t > led_wait_time) {
        led_last_t = millis();
        digitalWriteFast(LED_PIN, led_state);  // Toggle LED
        led_state = !led_state;
    }
}
