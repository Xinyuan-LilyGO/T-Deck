#pragma once

#define FT3267_SLAVE_ADDRESS           (0x38)
#define FT5x06_ADDR                    (0x38)


/** @brief FT5x06 register map and function codes */

#define FT5x06_DEVICE_MODE             ((uint8_t)0x00)
#define FT5x06_GESTURE_ID              ((uint8_t)0x01)
#define FT5x06_TOUCH_POINTS            ((uint8_t)0x02)

#define FT5x06_TOUCH1_EV_FLAG          ((uint8_t)0x03)
#define FT5x06_TOUCH1_XH               ((uint8_t)0x03)
#define FT5x06_TOUCH1_XL               ((uint8_t)0x04)
#define FT5x06_TOUCH1_YH               ((uint8_t)0x05)
#define FT5x06_TOUCH1_YL               ((uint8_t)0x06)

#define FT5x06_TOUCH2_EV_FLAG          ((uint8_t)0x09)
#define FT5x06_TOUCH2_XH               ((uint8_t)0x09)
#define FT5x06_TOUCH2_XL               ((uint8_t)0x0A)
#define FT5x06_TOUCH2_YH               ((uint8_t)0x0B)
#define FT5x06_TOUCH2_YL               ((uint8_t)0x0C)

#define FT5x06_TOUCH3_EV_FLAG          ((uint8_t)0x0F)
#define FT5x06_TOUCH3_XH               ((uint8_t)0x0F)
#define FT5x06_TOUCH3_XL               ((uint8_t)0x10)
#define FT5x06_TOUCH3_YH               ((uint8_t)0x11)
#define FT5x06_TOUCH3_YL               ((uint8_t)0x12)

#define FT5x06_TOUCH4_EV_FLAG          ((uint8_t)0x15)
#define FT5x06_TOUCH4_XH               ((uint8_t)0x15)
#define FT5x06_TOUCH4_XL               ((uint8_t)0x16)
#define FT5x06_TOUCH4_YH               ((uint8_t)0x17)
#define FT5x06_TOUCH4_YL               ((uint8_t)0x18)

#define FT5x06_TOUCH5_EV_FLAG          ((uint8_t)0x1B)
#define FT5x06_TOUCH5_XH               ((uint8_t)0x1B)
#define FT5x06_TOUCH5_XL               ((uint8_t)0x1C)
#define FT5x06_TOUCH5_YH               ((uint8_t)0x1D)
#define FT5x06_TOUCH5_YL               ((uint8_t)0x1E)

#define FT5x06_ID_G_THGROUP            ((uint8_t)0x80)
#define FT5x06_ID_G_THPEAK             ((uint8_t)0x81)
#define FT5x06_ID_G_THCAL              ((uint8_t)0x82)
#define FT5x06_ID_G_THWATER            ((uint8_t)0x83)
#define FT5x06_ID_G_THTEMP             ((uint8_t)0x84)
#define FT5x06_ID_G_THDIFF             ((uint8_t)0x85)
#define FT5x06_ID_G_CTRL               ((uint8_t)0x86)
#define FT5x06_ID_G_TIME_ENTER_MONITOR ((uint8_t)0x87)
#define FT5x06_ID_G_PERIODACTIVE       ((uint8_t)0x88)
#define FT5x06_ID_G_PERIODMONITOR      ((uint8_t)0x89)
#define FT5x06_ID_G_AUTO_CLB_MODE      ((uint8_t)0xA0)
#define FT5x06_ID_G_LIB_VERSION_H      ((uint8_t)0xA1)
#define FT5x06_ID_G_LIB_VERSION_L      ((uint8_t)0xA2)
#define FT5x06_ID_G_CIPHER             ((uint8_t)0xA3)
#define FT5x06_ID_G_MODE               ((uint8_t)0xA4)
#define FT5x06_ID_G_PMODE              ((uint8_t)0xA5)
#define FT5x06_ID_G_FIRMID             ((uint8_t)0xA6)
#define FT5x06_ID_G_STATE              ((uint8_t)0xA7)
#define FT5x06_ID_G_FT5201ID           ((uint8_t)0xA8)
#define FT5x06_ID_G_ERR                ((uint8_t)0xA9)