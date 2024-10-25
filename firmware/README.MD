# Write to new users

- This page provides a quick diagnosis of whether the hardware is normal. The firmware in the example table below provides quick testing of the hardware. Please follow the steps below to test

# 1️⃣ Quickly diagnose hardware

| Firmware Name                                                                                        | description                                    |
| ---------------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| [firmware-c3-t-keyboard.bin](./firmware-c3-t-keyboard.bin)                                           | [T-Deck Keyboard C3 firmware][1]               |
| [T-Deck_UnitTest_241004.bin](./T-Deck_UnitTest_241004.bin)                                           | [T-Deck/T-Deck-Plus function test firmware][1] |
| [firmware.meshtastic.factory.v2.5.8-pset-gps.bin](./firmware.meshtastic.factory.v2.5.8-pset-gps.bin) | [T-Deck/T-Deck-Plus factory firmware][2]       |
| [T-Deck-Plus-TouchFix_241025.bin](./T-Deck-Plus-TouchFix_241025.bin)                                 | [T-Deck-Plus Touch Fix][2]                     |

⚠⚠⚠

**T-Deck-Plus-TouchFix_241025.bin** Touch repair program, can only be flashed on rounded corner screens, please do not flash this on T-Deck rectangular screens, otherwise the screen touch will fail

**T-Deck-Plus-TouchFix_241025.bin** Touch repair program, can only be flashed on rounded corner screens, please do not flash this on T-Deck rectangular screens, otherwise the screen touch will fail

**T-Deck-Plus-TouchFix_241025.bin** Touch repair program, can only be flashed on rounded corner screens, please do not flash this on T-Deck rectangular screens, otherwise the screen touch will fail


[1]: https://www.lilygo.cc/products/t-deck
[2]: https://www.lilygo.cc/products/t-deck
[3]: https://www.lilygo.cc/products/t-deck



### Use ESP Download Tool

- Download [Flash_download_tool](https://www.espressif.com.cn/sites/default/files/tools/flash_download_tool_3.9.7_0.zip)

![web_flasher](./images/esp_downloader.gif)

* Note that after writing is completed, you need to press RST to reset.
* When writing the keyboard firmware, you need to select ESP32-C3 in the chip type

### Use Web Flasher

- [ESP Web Flasher Online](https://espressif.github.io/esptool-js/)

![web_flasher](./images/web_flasher.gif)

* Note that after writing is completed, you need to press RST to reset.

### Use command line


If system asks about install Developer Tools, do it.

```
python3 -m pip install --upgrade pip
python3 -m pip install esptool
```

In order to launch esptool.py, exec directly with this:

```
python3 -m esptool
```

For T-Deck/T-Deck-Plus use the following command to write

```
esptool --chip esp32s3  --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m 0x0 T-Deck_UnitTest_241004.bin

```


For ESP32-C3 Keyboard use the following command to write

```
esptool --chip esp32c3  --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m 0x0 firmware-c3-t-keyboard.bin

```


# 2️⃣FAQ

- Can't upload any sketch，Please enter the upload mode manually.
   1. Connect the board via the USB cable
   2. Press and hold the **BOOT** button , While still pressing the **BOOT** button (On the trackball above the keyboard, press down)
   3. Press **RST** button
   4. Release the **RST** button
   5. Release the **BOOT** button (If there is no **BOOT** button, disconnect IO0 from GND.)
   6. Upload sketch
   7. Press the **RST** button to exit download mode

- T-Keyboard the programming and burning interface of ESP32C3 is located at the 6Pin header on the side of the RST button, and the order starts from the top of the RST button, which are 3V3, GND, RST, BOOT, RX, TX, You need to connect USB TO TTL externally to burn the keyboard firmware. For TTL without automatic download circuit, before powering on, short-circuit the BOOT Pin and GND, and then power on. At this time, T-Keyboard will enter the download mode. After the burning is completed, you need to disconnect GND and BOOT before running the program.

   ![c3_boot](../images/c3_boot.jpg)