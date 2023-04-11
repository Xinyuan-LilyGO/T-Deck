/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 micky
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      ModulesFT3267.tpp
 * @author    Micky (513673326@qq.com)
 * @date      2022-12-14
 *
 */

#if defined(ARDUINO)
#include <Arduino.h>
#endif
#include "REG/FT5x06Constants.h"
#include "TouchLibCommon.tpp"
#include "TouchLibInterface.hpp"

class TouchLibFT3267 : public TouchLibCommon<TouchLibFT3267>, public TouchLibInterface, public TouchLibGesture {
  friend class TouchLibCommon<TouchLibFT3267>;


typedef enum {
  ft3267_gesture_none = 0x00,
  ft3267_gesture_move_up = 0x10,
  ft3267_gesture_move_left = 0x14,
  ft3267_gesture_move_down = 0x18,
  ft3267_gesture_move_right = 0x1c,
  ft3267_gesture_zoom_in = 0x48,
  ft3267_gesture_zoom_out = 0x49,
} ft3267_gesture_t;

public:
#if defined(ARDUINO)
  TouchLibFT3267(TwoWire &w, int sda = SDA, int scl = SCL, uint8_t addr = FT3267_SLAVE_ADDRESS, int rst = -1) {
    __wire = &w;
    __sda = sda;
    __scl = scl;
    __addr = addr;
    __rst = rst;
  }
#endif

  TouchLibFT3267() {
#if defined(ARDUINO)
    __wire = &Wire;
    __sda = SDA;
    __scl = SCL;
    __rst = -1;
#endif
    __addr = FT3267_SLAVE_ADDRESS;
  }

  ~TouchLibFT3267() {
    log_i("~TouchLibFT3267");
    deinit();
  }

  bool init() {
    if (begin()) {
      // Valid touching detect threshold
      this->writeRegister(FT5x06_ID_G_THGROUP, 70);

      // valid touching peak detect threshold
      this->writeRegister(FT5x06_ID_G_THPEAK, 60);

      // Touch focus threshold
      this->writeRegister(FT5x06_ID_G_THCAL, 16);

      // threshold when there is surface water
      this->writeRegister(FT5x06_ID_G_THWATER, 60);

      // threshold of temperature compensation
      this->writeRegister(FT5x06_ID_G_THTEMP, 10);

      // Touch difference threshold
      this->writeRegister(FT5x06_ID_G_THDIFF, 20);

      // Delay to enter 'Monitor' status (s)
      this->writeRegister(FT5x06_ID_G_TIME_ENTER_MONITOR, 2);

      // Period of 'Active' status (ms)
      this->writeRegister(FT5x06_ID_G_PERIODACTIVE, 12);

      // Timer to enter 'idle' when in 'Monitor' (ms)
      this->writeRegister(FT5x06_ID_G_PERIODMONITOR, 40);

      return 1;
    }
    return 0;
  }

  void deinit() { end(); }

  bool enableSleep() { return 0; }

  bool read() {
    this->readRegister(FT5x06_TOUCH_POINTS, &raw_data, 1);
    return (raw_data & 0x0f) > 0;
  }

  uint8_t getPointNum() { return raw_data & 0x0f; }

  TP_Point getPoint(uint8_t n) {
    TP_Point t;
    uint8_t tmp[4] = {0};
    switch (n) {
    case 0:
      this->readRegister(FT5x06_TOUCH1_XH, tmp, sizeof(tmp));
      break;
    case 1:
      this->readRegister(FT5x06_TOUCH2_XH, tmp, sizeof(tmp));
      break;
    case 2:
      this->readRegister(FT5x06_TOUCH3_XH, tmp, sizeof(tmp));
      break;
    case 3:
      this->readRegister(FT5x06_TOUCH4_XH, tmp, sizeof(tmp));
      break;
    case 4:
      this->readRegister(FT5x06_TOUCH5_XH, tmp, sizeof(tmp));
      break;
    default:
      log_i("The parameter range of getPoint is between 0 and 4.");
      break;
    }

    t.x = ((tmp[0] & 0x0f) << 8) + tmp[1];
    t.y = ((tmp[2] & 0x0f) << 8) + tmp[3];
    t.id = n;

    if (rotation == 0) {
    } else if (rotation == 1) {
      uint16_t tmp = t.x;
      t.x = t.y;
      t.y = tmp;
    }
    return t;
  }

  void setRotation(uint8_t r) { rotation = r % 4; }

  uint8_t getRotation() { return rotation; }

  uint8_t getGesture(void) {
    uint8_t tmp = 0;
    this->readRegister(FT5x06_GESTURE_ID, &tmp, 1);
    return tmp;
  }

private:
  bool isEnableGesture() { return 0; }
  bool enableGesture() { return 0; }
  bool disableGesture() { return 0; }

protected:
  bool initImpl() { return true; }
  uint8_t raw_data = 0;
  uint8_t rotation = 0;
};