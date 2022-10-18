/**
 *  Copyright 2022 Bear Flag Robotics
 *
 *  hello_world_test.cpp
 *
 *      TODO
 *
 *      Author: Austin Chun
 *      Date:   Oct 2022
 *
 */

#include <Arduino.h>

const uint8_t LED_PIN = LED_BUILTIN;
bool led_state = 0;
uint32_t last_led_t;  // Log time of last LED toggle
uint32_t led_wait_time;  // How long between LED toggles (changes based on USB connection)

uint32_t print_last_t;
uint8_t count = 0;

////////////////////////////////////////////////////////////////////////////
///                         Setup/Loop Functions                         ///
////////////////////////////////////////////////////////////////////////////

void setup() {
    while (!Serial) {}
    Serial.begin(1);

    pinMode(LED_PIN, OUTPUT);

    // Serial.write(0x00); Serial.write(0x55); Serial.write(0x02);
    Serial.println("#Running hello_world_test.cpp");
}

void loop() {
    // Write data periodically
    if (millis() - print_last_t > 1000) {
        print_last_t = millis();

        // Serial.write(0x00); Serial.write(0x55); Serial.write(0x02);
        Serial.print("#Hellow world, ");
        Serial.println(count++);
    }


    // Blink the LED (slow when Serial down, fast when Serial up)
    led_wait_time = (Serial) ? 100 : 500;
    if (millis() - last_led_t > led_wait_time) {
        last_led_t = millis();
        digitalWriteFast(LED_PIN, led_state);
        led_state = !led_state;
    }
}
