#pragma once

//! The board peripheral power control pin needs to be set to HIGH when using the peripheral
#define TDECK_PERI_POWERON 10

#define TDECK_SPI_MOSI 41
#define TDECK_SPI_MISO 38
#define TDECK_SPI_SCK 40

#define TDECK_SDCARD_CS 39

#define TDECK_TFT_CS 12
#define TDECK_TFT_DC 11
#define TDECK_TFT_BACKLIGHT 42

#define TDECK_RADIO_CS 9
#define TDECK_RADIO_BUSY 13
#define TDECK_RADIO_RST 17
#define TDECK_RADIO_DIO1 45

#define TDECK_I2C_SDA 18
#define TDECK_I2C_SCL 8

#define TDECK_TOUCH_INT 16
#define TDECK_KEYBOARD_INT 46

#define TDECK_KEYBOARD_ADDR 0x55

#define TDECK_TRACKBALL_UP 3
#define TDECK_TRACKBALL_DOWN 15
#define TDECK_TRACKBALL_LEFT 1
#define TDECK_TRACKBALL_RIGHT 2
#define TDECK_TRACKBALL_CLICK 0

#define TDECK_ES7210_MCLK 48
#define TDECK_ES7210_LRCK 21
#define TDECK_ES7210_SCK 47
#define TDECK_ES7210_DIN 14

#define TDECK_I2S_WS 5
#define TDECK_I2S_BCK 7
#define TDECK_I2S_DOUT 6

#define TDECK_BAT_ADC 4
