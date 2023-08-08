/*******************************************************************************
 * Keyboard libraries:
 * https://github.com/Xinyuan-LilyGO/T-Deck/tree/master/examples/Keyboard_T_Deck_Master
 ******************************************************************************/

#include "TDECK_PINS.h"
#include <Wire.h>

void keyboard_init() {
  // Check keyboard
  Wire.requestFrom(TDECK_KEYBOARD_ADDR, 1);
  if (Wire.read() == -1) {
    Serial.println("LILYGO Keyboad not online!");
  }
}

char keyboard_get_key() {
    // Read key value from esp32c3
    Wire.requestFrom(TDECK_KEYBOARD_ADDR, 1);
    if (Wire.available() > 0) {
        return Wire.read();
    } else {
      return 0;
    }
}
