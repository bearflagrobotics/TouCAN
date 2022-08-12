/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  serial_driver.cpp
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 */

#include <serial_driver.h>

#include <Arduino.h>


bool SerialDriver::WriteData(const uint8_t* data, const uint8_t data_len) {
    if (IsUp()) {
        // Calculate the checksum
        uint16_t chksm_ = Fletcher16(data, data_len);
        // Write the CAN msg to Serial port
        Serial.write(kSTART_BYTE);
        Serial.write(kSYNCH_BYTE);
        Serial.write(msg_idx++);
        Serial.write(data_len);
        Serial.write(data, data_len);
        Serial.write((uint8_t*) &chksm_, 2);
        return true;
    }
    return false;
}

bool SerialDriver::IsUp() {
    if (is_up && !Serial) {
        Serial.begin(1);
        is_up = false;
    }
    if (Serial) {
        is_up = true;
    }
    return is_up;
}


/**
 * @brief      Calculate the Fletcher16 checksum for Serial Comms w/ ROS
 *
 * @param      data   byte array with the data
 * @param[in]  count  the size (bytes) of the msg
 *
 * @return     { description_of_the_return_value }
 */
#define MIN(a, b) ((a) < (b)) ? (a) : (b)
uint16_t SerialDriver::Fletcher16(const uint8_t* data, int32_t count) {
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
