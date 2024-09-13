/**
 * @file      Touchpad.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-04-12
 *
 */

#include <Arduino.h>
#include "utilities.h"
#include "TouchDrvGT911.hpp"

TouchDrvGT911 touch;

void setup()
{
    Serial.begin(115200);
    pinMode(BOARD_TOUCH_INT, INPUT);

    //! The board peripheral power control pin needs to be set to HIGH when using the peripheral
    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);


    touch.setPins(-1, BOARD_TOUCH_INT);
    if (!touch.begin(Wire, GT911_SLAVE_ADDRESS_L)) {
        while (1) {
            Serial.println("Failed to find GT911 - check your wiring!");
            delay(1000);
        }
    }

    Serial.println("Init GT911 Sensor success!");

    // Set touch max xy
    touch.setMaxCoordinates(320, 240);

    // Set swap xy
    touch.setSwapXY(true);

    // Set mirror xy
    touch.setMirrorXY(false, true);

}


int16_t x[5], y[5];

void loop()
{
    if (touch.isPressed()) {
        Serial.println("Pressed!");
        uint8_t touched = touch.getPoint(x, y, touch.getSupportTouchPoint());
        if (touched > 0) {
            Serial.print(millis());
            Serial.print("ms ");
            for (int i = 0; i < touched; ++i) {
                Serial.print("X[");
                Serial.print(i);
                Serial.print("]:");
                Serial.print(x[i]);
                Serial.print(" ");
                Serial.print(" Y[");
                Serial.print(i);
                Serial.print("]:");
                Serial.print(y[i]);
                Serial.print(" ");
            }
            Serial.println();
        }
    }
    delay(100);
}

















