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
        Serial.write(kStartBytes, sizeof(kStartBytes));
        Serial.write(kDataMsgType);
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

bool SerialDriver::Read() {
    uint16_t bytes_in_buff = this->available();  // Check how many bytes to read

    uint8_t cur_c;

    // Limit reading to only what was in buffer (fixed loop)
    for (int i = 0; i < bytes_in_buff; ++i) {
        cur_c = this->read();

        // PrintStringHeader();
        // this->print("read_state: ");
        // this->print(read_state);
        // this->print(", cur_c: ");
        // this->print(cur_c, HEX);
        // this->println();

        // Simple state machine for reading
        switch (read_state) {
            // Read the Start Byte Sequence (eg. [0x00, 0x55])
            case READ_START:
                // Check against start_byte sequence
                if (cur_c == kStartBytes[start_bytes_i]) {
                    start_bytes_i++;
                } else {  // Incorrect, so restart
                    start_bytes_i = 0;
                }
                // Next State
                if (start_bytes_i == kNumStartBytes) {
                    start_bytes_i = 0;
                    read_state = READ_TYPE;
                }
                break;
            // Read the message type
            case READ_TYPE:
                msg_type = cur_c;
                // Next State
                if (msg_type == kDataMsgType) {
                    read_state = READ_IDX;
                } else if (msg_type == kCanMsgType) {
                    read_state = READ_IDX;
                } else if (msg_type == kStringMsgType) {
                    read_state = READ_STRING;
                    string_buffer_i = 0;
                }
                break;

            // Read the message index number (one byte)
            case READ_IDX:
                // TODO: Check index against expected, to track dropped packets
                // Next state
                read_msg_index = cur_c;
                read_state = READ_LEN;
                break;
            // Read the data length (one byte)
            case READ_LEN:
                data_len_exp = cur_c;
                // Next State
                data_buff_i = 0;  // Reset the data buffer (just reset index)
                read_state = READ_DATA;
                break;
            // Read the Data into a buffer
            case READ_DATA:
                data_buff[data_buff_i++] = cur_c;
                // Next State
                if (data_buff_i >= data_len_exp) {
                    chksm_i = 0;
                    read_state = READ_CHKSM;
                }
                break;
            case READ_CHKSM:
                // PrintStringHeader();
                // this->print("Chksm[");
                // this->print(chksm_i);
                // this->print("] = ");
                // this->print(cur_c, HEX);
                // this->println();

                chksm[chksm_i++] = cur_c;

                // Next State
                if (chksm_i >= kChksmLen) {
                    uint16_t rcvd_chksm = (chksm[1] << 8) | chksm[0];
                    if (VerifyChecksum(data_buff, data_len_exp, rcvd_chksm)) {
                        // Valid checksum... do something about it
                        // PrintStringHeader();
                        // this->print("Received a msg!");
                        // this->println();
                        read_state = READ_START;

                        return true;
                    // } else {
                    //     PrintStringHeader();
                    //     this->print("Invalid checksum. Rcvd: ");
                    //     this->print(rcvd_chksm, HEX);
                    //     this->print("  Exp: ");
                    //     this->print(Fletcher16(data_buff, data_len_exp), HEX);
                    //     this->println();
                    }

                    read_state = READ_START;
                }

                break;

            case READ_STRING:
                if (cur_c != '\n') {
                    string_buffer[string_buffer_i++] = cur_c;
                } else {
                    read_state = READ_START;
                    return true;
                }
                break;
        }
    }

    return false;
}

bool SerialDriver::VerifyChecksum(const uint8_t* data, uint8_t data_len, uint16_t chksm_rcvd) {
    return (chksm_rcvd == Fletcher16(data, data_len));
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

bool SerialDriver::PrintStringHeader() {
    if (IsUp()) {
        Serial.write(kStartBytes, sizeof(kStartBytes));
        Serial.write(kStringMsgType);
        return true;
    }
    return false;
}


