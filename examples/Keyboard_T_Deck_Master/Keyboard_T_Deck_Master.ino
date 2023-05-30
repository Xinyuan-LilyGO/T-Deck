/**
 * @file      Keyboard_T_Deck_Master.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-05-30
 *
 */
#include <Arduino.h>
#include <Wire.h>

#define LILYGO_KB_SLAVE_ADDRESS     0x55


#define BOARD_POWERON       10
#define BOARD_I2C_SDA       18
#define BOARD_I2C_SCL       8


void setup()
{
    Serial.begin(115200);

    Serial.println("T-Deck Keyboard Master");

    //!⚠️ The board peripheral power control pin needs to be set to HIGH when using the peripheral
    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

    // There needs to be a delay after power on, give LILYGO-KEYBOARD some startup time
    delay(500);

    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);

    // Check keyboard
    Wire.requestFrom(LILYGO_KB_SLAVE_ADDRESS, 1);
    if (Wire.read() == -1) {
        while (1) {
            Serial.println("LILYGO Keyboad not online .");
            delay(1000);
        }
    }
}

void loop()
{
    // Read key value from esp32c3
    char keyValue = 0;
    Wire.requestFrom(LILYGO_KB_SLAVE_ADDRESS, 1);
    while (Wire.available() > 0) {
        keyValue = Wire.read();
        if (keyValue != (char)0x00) {
            Serial.print("keyValue : ");
            Serial.println(keyValue);
        }
    }
    delay(5);
}



