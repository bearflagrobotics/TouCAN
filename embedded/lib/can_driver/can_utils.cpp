/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  can_utils.cpp
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Aug 2022
 */

#include <can_utils.h>

#include <Arduino.h>  // Core library


void PrintStructHex(const CanMsg& can_msg) {
    Serial.print("Raw: ");
    for (uint i = 0; i < sizeof(can_msg.fields); ++i) {
        // Line below triggers compilation warning, but should be fine, since Union
        Serial.print(can_msg.raw[i], HEX); Serial.print(" ");
    }
    Serial.println();
}

void PrintCanData(const CanMsg& can_msg) {
    Serial.print("ID: "); Serial.print(can_msg.fields.msg.id, HEX);
    Serial.print(", Data: ");
    for (int i = 0; i < can_msg.fields.msg.len; ++i) {
        Serial.print(can_msg.fields.msg.buf[i], HEX); Serial.print(" ");
    }
    Serial.println();
}

// /**
//  * @brief      Calculate the Fletcher16 checksum for Serial Comms w/ ROS
//  *
//  * @param[in]  ID     ID of the Serial msg
//  * @param      data   byte array with the data
//  * @param[in]  count  the size (bytes) of the msg
//  *
//  * @return     { description_of_the_return_value }
//  */
// #define MIN(a, b) ((a) < (b)) ? (a) : (b)
// uint16_t Fletcher16(const uint8_t* data, int32_t count) {
//     uint32_t c0, c1;
//     uint32_t i;

//     // Found by solving for c1 overflow:
//     // n > 0 and n * (n+1) / 2 * (2^8-1) < (2^32-1).
//     for (c0 = c1 = 0; count > 0; count -= 5802) {
//         uint32_t blocklen = MIN(5802, count);
//         for (i = 0; i < blocklen; ++i) {
//             c0 = c0 + *data++;
//             c1 = c1 + c0;
//         }
//         c0 = c0 % 255;
//         c1 = c1 % 255;
//     }
//     return (c1 << 8 | c0);
// }

uint32_t IdToPgn(uint32_t id) {
    // For Addressable messages, last 8 bits are used for destination address, thus not
    // included as PGN
    uint32_t pgn;
    uint8_t pf = (id >> 16) & 0xFF;  // PDU Format byte
    if (pf < 240) {
        pgn = (id >> 8) & 0x3FF00;
    } else {
        pgn = (id >> 8) & 0x3FFFF;
    }
    return pgn;
}



