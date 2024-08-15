/**
 * @file      GPSShield.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-03-29
 * @note      This sketch only applies to boards carrying GPS shields, the default is T-Deck
 *            It does not support GPS function. Of course, you can use an external GPS module to connect to the Gover interface.
 */

#ifndef SerialGPS
#define SerialGPS Serial1
#endif

#define BOARD_GPS_TX_PIN                 43
#define BOARD_GPS_RX_PIN                 44

#define BOARD_POWERON                    10

#include <Arduino.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

// L76K GPS USE 9600 BAUDRATE
#define GPS_BAUD        9600

// M10Q GPS USE 38400 BAUDRATE
// #define GPS_BAUD        38400

void setup()
{
    Serial.begin(115200);

    //!⚠️ The board peripheral power control pin needs to be set to HIGH when using the peripheral
    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

    //GPS Serial port
    SerialGPS.begin(GPS_BAUD, SERIAL_8N1, BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);

    Serial.println("This sketch only applies to boards carrying GPS shields, the default is T-Deck");

    Serial.println("It does not support GPS function. Of course, you can use an external GPS module to connect to the Gover interface.");


    delay(2000);

}

void loop()
{
    while (SerialGPS.available()) {
        int c = SerialGPS.read();
        // Serial.write(c);
        if (gps.encode(c)) {
            displayInfo();
        }
    }
    delay(1);
}


void displayInfo()
{
    Serial.print(F("Location: "));
    if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid()) {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid()) {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}
