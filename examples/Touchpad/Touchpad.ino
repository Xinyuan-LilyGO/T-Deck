/**
 * @file      Touchpad.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-04-12
 *
 */

#include <Arduino.h>

#define TOUCH_MODULES_GT911
#include "TouchLib.h"
#include "utilities.h"

TouchLib touch(Wire, BOARD_I2C_SDA, BOARD_I2C_SCL, GT911_SLAVE_ADDRESS1);


void setup()
{
    Serial.begin(115200);
    pinMode(BOARD_TOUCH_INT, INPUT);

    //! The board peripheral power control pin needs to be set to HIGH when using the peripheral
    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);

    touch.init();

}

void loop()
{
    // You can judge the level of the touch interrupt to get whether there is a touch
    if (digitalRead(BOARD_TOUCH_INT)) {
        bool res =  touch.read();
        if (res) {
            TP_Point  p = touch.getPoint(0);
            Serial.printf("X:%d Y:%d\n", p.x, p.y);
        }
    }
    delay(5);
}

















