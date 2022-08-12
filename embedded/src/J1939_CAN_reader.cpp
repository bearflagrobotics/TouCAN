/**
 *  Copyright 2020 Bear Flag Robotics
 *
 *  J1939_CAN_reader.cpp
 *
 *      Teensy designated to reading the J1939 CAN Bus port of tractors (Diagnostics Port).
 *      All data is piped to ROS via Serial USB, where it is republished to a ROS topic,
 *      for later analysis/filtering as needed.
 *
 *      Author: Austin Chun
 *      Date:   Apr 2020
 */

#include <Arduino.h>  // Core library

#include <FlexCAN.h>

// #include <emb_version.h>

#define START_BYTE  0x00
#define SYNCH_BYTE  0x55


const uint8_t LED_PIN = 13;
bool LED_STATE = 0;

// Create CAN Bus object
FlexCAN can0 = FlexCAN(500000, 0);
FlexCAN can1 = FlexCAN(250000, 1);

uint32_t last_can_msg_t;
uint32_t last_led_t;  // Log time of last LED toggle
uint32_t led_wait_time;  // How long between LED toggles (changes based on USB connection)


// Simple Union to allow easy read/write of CAN msg struct
union CANMsg {
    // The CAN Message struct
    struct Fields {
        uint16_t ind;       // Track msg number (recevied by uC)
        uint8_t bus_id;     // Which bus is this on
        uint8_t  : 8;       // Byte padding, for 32-bit words
        uint32_t tstamp;    // microseconds since launch
        CAN_message_t msg;
        // CAN_message_t struct for reference
        //          typedef struct CAN_message_t {
        //            uint32_t id; // can identifier
        //            uint8_t ext; // identifier is extended
        //            uint8_t len; // length of data
        //            uint16_t timeout; // milliseconds, zero will disable waiting
        //            uint8_t buf[8];
        //          } CAN_message_t;

    } fields;
    // Raw Data
    uint8_t raw[];
} can_msg;

// Forward Declaration
void readCAN(FlexCAN* can, uint8_t bus_id);
void printCANData();
void printStruct_Hex();

uint16_t Fletcher16(uint8_t *data, int32_t count);
uint32_t IDtoPGN(uint32_t ID);

////////////////////////////////////////////////////////////////////////////
///                         Setup/Loop Functions                         ///
////////////////////////////////////////////////////////////////////////////

bool serial_up = false;

uint8_t msg_idx = 0;
uint8_t msg_len = 8;
uint32_t last_mock_print_t = millis();
uint32_t t2_offset = 25;
uint32_t last_mock_print_t2 = millis() + t2_offset;

void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Set the Mask and Filter for the CAN object
    can0.begin();
    can1.begin();

    // Setup Serial
    Serial.begin(1);
    while (!Serial) {  // Blocking wait for Serial port to open
        if (millis() - last_led_t > 2000) {
            last_led_t = millis();
            digitalWriteFast(LED_PIN, LED_STATE); LED_STATE = !LED_STATE;  // Toggle LED
        }
    }
    serial_up = true;

    Serial.println("Running 'J1939_CAN_reader'");
    Serial.println("Initialization Complete.");
}


void loop() {

    // Serial just closed
    if (serial_up && !Serial) {
        Serial.begin(1);
        serial_up = false;
    }
    if (Serial) serial_up = true;

    // Read from CAN buses
    readCAN(&can0, 0);
    readCAN(&can1, 1);

    // Blink the LED (slow when serial down, fast when serial up)
    led_wait_time = serial_up ? 100 : 500;
    if (millis() - last_led_t > led_wait_time) {
        last_led_t = millis();
        digitalWriteFast(LED_PIN, LED_STATE);  // Toggle LED
        LED_STATE = !LED_STATE;
    }

}

void readCAN(FlexCAN* can, uint8_t bus_id) {

    if (can->available()) {

        // Read in the CAN msg
        can->read(can_msg.fields.msg);

        // Increment count
        can_msg.fields.ind++;

        can_msg.fields.bus_id = bus_id;
        can_msg.fields.tstamp = micros();

        // Calculate the checksum
        uint16_t chksm = Fletcher16(can_msg.raw, sizeof(can_msg.fields));

        if (serial_up) {

            // Write the CAN msg to Serial port
            Serial.write(START_BYTE); Serial.write(SYNCH_BYTE);
            Serial.write(msg_idx++); Serial.write(sizeof(can_msg.fields));
            Serial.write(can_msg.raw, sizeof(can_msg.fields));
            Serial.write((uint8_t*) &chksm, 2);

            // // Debug printing
            // printStruct_Hex();
            // printCANData();
            // Serial.print("Chksm: "); Serial.println(chksm);
            // Serial.print("ID: "); Serial.println(can_msg.fields.msg.id, HEX);
            // Serial.print("PGN: "); Serial.println(IDtoPGN(can_msg.fields.msg.id));
        }

        // Log last time, to check for timeouts
        last_can_msg_t = millis();

    }

    // // HACK: Just publish random data
    // else {
        // // if (millis() - last_mock_print_t > 20) {
        //     last_mock_print_t = millis();
        //     // Calculate the checksum
        //     uint8_t mock_data[4] = {0x01, 0x02, msg_idx/10, msg_idx};
        //     uint16_t chksm = Fletcher16(mock_data, 4);
        //     // Write the CAN msg to Serial port
        //     Serial.write(START_BYTE); Serial.write(SYNCH_BYTE);
        //     Serial.write(msg_idx++); Serial.write(4);
        //     Serial.write(mock_data, 4);
        //     Serial.write((uint8_t*) &chksm, 2);
        // // }
        // // if (millis() - last_mock_print_t2 > 50 + t2_offset) {
        //     // last_mock_print_t2 = millis() + t2_offset;
        //     // Calculate the checksum
        //     uint8_t mock_data2[6] = {0x01, 0x02, 0x03, 0x04, msg_idx/10, msg_idx};
        //     chksm = Fletcher16(mock_data2, 6);
        //     // Write the CAN msg to Serial port
        //     Serial.write(START_BYTE); Serial.write(SYNCH_BYTE);
        //     Serial.write(msg_idx++); Serial.write(6);
        //     Serial.write(mock_data2, 6);
        //     Serial.write((uint8_t*) &chksm, 2);
        // // }
    // }


}

void printStruct_Hex() {
    Serial.print("Raw: ");
    for (uint i = 0; i < sizeof(can_msg.fields); ++i) {
        // Line below triggers compilation warning, but should be fine, since Union
        Serial.print(can_msg.raw[i], HEX); Serial.print(" ");
    }
    Serial.println();
}


void printCANData() {
    Serial.print("ID: "); Serial.print(can_msg.fields.msg.id, HEX);
    Serial.print(", Data: ");

    for (int i = 0; i < can_msg.fields.msg.len; ++i) {
        Serial.print(can_msg.fields.msg.buf[i], HEX); Serial.print(" ");
    }
    Serial.println();

}



/**
 * @brief      Calculate the Fletcher16 checksum for Serial Comms w/ ROS
 *
 * @param[in]  ID     ID of the Serial msg
 * @param      data   byte array with the data
 * @param[in]  count  the size (bytes) of the msg
 *
 * @return     { description_of_the_return_value }
 */
#define MIN(a, b) ((a) < (b)) ? (a) : (b)
uint16_t Fletcher16(uint8_t *data, int32_t count) {
    uint32_t c0, c1;
    uint32_t i;

    // Found by solving for c1 overflow:
    // n > 0 and n * (n+1) / 2 * (2^8-1) < (2^32-1).
    for (c0 = c1 = 0; count > 0; count -= 5802) {
            uint32_t blocklen = MIN(5802, count);
            for (i = 0; i < blocklen; ++i) {
                    c0 = c0 + *data++;
                    c1 = c1 + c0;
            }
            c0 = c0 % 255;
            c1 = c1 % 255;
    }
    return (c1 << 8 | c0);
}

uint32_t IDtoPGN(uint32_t ID) {
    // For Addressable messages, last 8 bits are used for destination address, thus not
    // included as PGN
    uint32_t PGN;
    uint8_t PF = (ID >> 16) & 0xFF;  // PDU Format byte
    if (PF < 240) {
        PGN = (ID >> 8) & 0x3FF00;
    } else {
        PGN = (ID >> 8) & 0x3FFFF;
    }
    return PGN;
}
