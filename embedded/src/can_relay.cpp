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


// // TODO: Remove these filters, only necessary for slow python processing
// uint32_t mask0 = 0x00FFFFFF;
// uint32_t mask1 = 0x00FFFFFF;

// CAN_filter_t filt00 {0x0, 0x1, 0x00FFFE03};  // PowerTrainCondition3
// CAN_filter_t filt01 {0x0, 0x1, 0x00FFFB31};  // OperatorSwitchControls4
// CAN_filter_t filt02 {0x0, 0x1, 0x00FFFF1C};  // GpsStatus
// CAN_filter_t filt03 {0x0, 0x1, 0x00FFFE91};  // FrontConsoleSignalsMisc1

// CAN_filter_t filt10 {0x0, 0x1, 0x00FEF31C};  // VehiclePosition
// CAN_filter_t filt11 {0x0, 0x1, 0x00FEE81C};  // DirectionSpeed
// CAN_filter_t filt12 {0x0, 0x1, 0x00AC00F0};  // GuidanceMachineStatus
// CAN_filter_t filt13 {0x0, 0x1, 0x00FFFAF0};  // ExternalGuidanceCommandAndState
// CAN_filter_t filt14 {0x0, 0x1, 0x00FFFAF0};  // VehicleAutomationStatus
// CAN_filter_t filt15 {0x0, 0x1, 0x00FEFCF0};  // DashDisplay

// // Create CAN Bus object
// CanDriver can0 {500000, 0, 0, 0, mask0};  // Standard Baud of Vehicle Bus
// CanDriver can1 {250000, 1, 0, 0, mask1};  // Standard Baud of Implement Bus (ISOBUS)

CanDriver can0 {500000, 0};  // Standard Baud of Vehicle Bus
CanDriver can1 {250000, 1};  // Standard Baud of Implement Bus (ISOBUS)


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

    // // Setup filters
    // can0.SetCanFilter(filt00, 0); can1.SetCanFilter(filt10, 0);
    // can0.SetCanFilter(filt01, 1); can1.SetCanFilter(filt11, 1);
    // can0.SetCanFilter(filt02, 2); can1.SetCanFilter(filt12, 2);
    // can0.SetCanFilter(filt03, 3); can1.SetCanFilter(filt13, 3);
    // can0.SetCanFilter(filt00, 4); can1.SetCanFilter(filt14, 4);
    // can0.SetCanFilter(filt00, 5); can1.SetCanFilter(filt15, 5);
    // can0.SetCanFilter(filt00, 6); can1.SetCanFilter(filt10, 6);
    // can0.SetCanFilter(filt00, 7); can1.SetCanFilter(filt10, 7);

    // Setup Serial
    serial.begin(1);
    while (!serial.IsUp()) {  // Blocking wait for Serial port to open
        if (millis() - led_last_t > 2000) {
            led_last_t = millis();
            digitalWriteFast(LED_PIN, led_state);
            led_state = !led_state;  // Toggle LED
        }
    }
    serial.PrintStringHeader();
    serial.println("Running 'can_relay.cpp'");
}


void loop() {

    // Read CAN bus, write to Serial
    for (int i = 0; i < MAX_READ_LOOP; ++i) {
        if (can0.ReadCan()) {
            serial.WriteData(can0.rx_msg_.raw, sizeof(can0.rx_msg_.fields), true);
        }
        if (can1.ReadCan()) {
            serial.WriteData(can1.rx_msg_.raw, sizeof(can1.rx_msg_.fields), true);
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
                serial.print("ERROR: Unknown bus_id: 0x");
                serial.print(bus_id, HEX);
                serial.println();
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
