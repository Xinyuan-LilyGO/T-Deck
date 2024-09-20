/**
 * @file      UnitTest.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-04-11
 * @note      Arduino Setting
 *            Tools ->
 *                  Board:"ESP32S3 Dev Module"
 *                  USB CDC On Boot:"Enable"
 *                  USB DFU On Boot:"Disable"
 *                  Flash Size : "16MB(128Mb)"
 *                  Flash Mode"QIO 80MHz
 *                  Partition Scheme:"16M Flash(3M APP/9.9MB FATFS)"
 *                  PSRAM:"OPI PSRAM"
 *                  Upload Mode:"UART0/Hardware CDC"
 *                  USB Mode:"Hardware CDC and JTAG"
 */
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <SD.h>
#include "es7210.h"
#include <Audio.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include "TouchDrvGT911.hpp"
#include <esp_sntp.h>

// By default, the audio pass-through speaker is used for testing, and esp_sr can also be used for noise detection.
#define USE_ESP_VAD

#include "utilities.h"
#include "config.h"

#ifndef BOARD_HAS_PSRAM
#error "Detected that PSRAM is not turned on. Please set PSRAM to OPI PSRAM in ArduinoIDE"
#endif

#ifndef SerialGPS
#define SerialGPS Serial1
#endif

#include <TinyGPS++.h>
TinyGPSPlus gps;


#define MIC_I2S_SAMPLE_RATE         16000
#define MIC_I2S_PORT                I2S_NUM_1
#define SPK_I2S_PORT                I2S_NUM_0
#define VAD_SAMPLE_RATE_HZ          16000
#define VAD_FRAME_LENGTH_MS         30
#define VAD_BUFFER_LENGTH           (VAD_FRAME_LENGTH_MS * VAD_SAMPLE_RATE_HZ / 1000)

#if TFT_DC !=  BOARD_TFT_DC || TFT_CS !=  BOARD_TFT_CS || TFT_MOSI !=  BOARD_SPI_MOSI || TFT_SCLK !=  BOARD_SPI_SCK
#error "Not using the already configured T-Deck file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
#error "Not using the already configured T-Deck file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
#error "Not using the already configured T-Deck file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
#endif

#define LVGL_BUFFER_SIZE            (TFT_WIDTH * TFT_HEIGHT * sizeof(lv_color_t))


typedef struct {
    uint8_t cmd;
    uint8_t data[14];
    uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
    {0x01, {0}, 0 | 0x80},
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0x36, {0x55}, 1},
    {0xB2, {0x0C, 0x0C, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X1A}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X13}, 1},
    {0xC4, {0X20}, 1},
    {0xC6, {0X0F}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XD0, 0X0D, 0X14, 0X0D, 0X0D, 0X09, 0X38, 0X44, 0X4E, 0X3A, 0X17, 0X18, 0X2F, 0X30}, 14},
    {0xE1, {0XD0, 0X09, 0X0F, 0X08, 0X07, 0X14, 0X37, 0X44, 0X4D, 0X38, 0X15, 0X16, 0X2C, 0X3E}, 14},
    {0x21, {0}, 0}, //invertDisplay
    {0x29, {0}, 0},
    {0x2C, {0}, 0},
};

TFT_eSPI        tft;
SemaphoreHandle_t xSemaphore = NULL;
TouchDrvGT911 touch;

lv_indev_t  *kb_indev = NULL;
lv_indev_t  *mouse_indev = NULL;
lv_indev_t  *touch_indev = NULL;
lv_group_t  *kb_indev_group;


LV_IMG_DECLARE(image_emoji);



#ifdef USE_ESP_VAD
#include <esp_vad.h>
int16_t         *vad_buff;
vad_handle_t    vad_inst;
const size_t    vad_buffer_size = VAD_BUFFER_LENGTH * sizeof(short);
#else
uint16_t loopbackBuffer[3200] = {0};
#endif

SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
Audio           audio;
size_t          bytes_read;
uint8_t         status;
TaskHandle_t    playHandle = NULL;
TaskHandle_t    radioHandle = NULL;

static lv_obj_t *vad_btn_label;
static uint32_t vad_detected_counter = 0;
static TaskHandle_t vadTaskHandler;
bool        kbDetected = false;
bool        touchDetected = false;
bool        transmissionFlag = true;
bool        enableInterrupt = true;
int         transmissionState ;
bool        hasRadio = false;
bool        sender = true;
bool        enterSleep = false;
uint32_t    sendCount = 0;
uint32_t    configTxInterval = 1000;
uint32_t    last_update_millis = 0;


lv_obj_t    *main_count;





static void setupLvgl();
static void keypad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static uint32_t keypad_get_key(void);
static void touchpad_read( lv_indev_drv_t *indev_driver, lv_indev_data_t *data );
static void mouse_read(lv_indev_drv_t *indev, lv_indev_data_t *data);
static void disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p );
static bool GPS_Recovery();



extern void updateGPS(double lat, double lng,
                      uint16_t year, uint8_t month, uint8_t day,
                      uint8_t hour, uint8_t minute, uint8_t second,
                      double speed, uint32_t rx_char );
extern void updateNoiseLabel(uint32_t cnt);
extern void setLoRaMessage(const char *text);
extern void setupUI(void);


bool setupGPS()
{
    SerialGPS.begin(9600, SERIAL_8N1, BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);

    bool result = false;
    uint32_t startTimeout ;
    for (int i = 0; i < 3; ++i) {
        SerialGPS.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
        delay(5);
        // Get version information
        startTimeout = millis() + 3000;
        Serial.print("Try to init L76K . Wait stop .");
        while (SerialGPS.available()) {
            Serial.print(".");
            SerialGPS.readString();
            if (millis() > startTimeout) {
                Serial.println("Wait L76K stop NMEA timeout!");
                return false;
            }
        };
        Serial.println();
        SerialGPS.flush();
        delay(200);

        SerialGPS.write("$PCAS06,0*1B\r\n");
        startTimeout = millis() + 500;
        String ver = "";
        while (!SerialGPS.available()) {
            if (millis() > startTimeout) {
                Serial.println("Get L76K timeout!");
                return false;
            }
        }
        SerialGPS.setTimeout(10);
        ver = SerialGPS.readStringUntil('\n');
        if (ver.startsWith("$GPTXT,01,01,02")) {
            Serial.println("L76K GNSS init succeeded, using L76K GNSS Module\n");
            result = true;
            break;
        }
        delay(500);
    }
    // Initialize the L76K Chip, use GPS + GLONASS
    SerialGPS.write("$PCAS04,5*1C\r\n");
    delay(250);
    // only ask for RMC and GGA
    SerialGPS.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
    delay(250);
    // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
    SerialGPS.write("$PCAS11,3*1E\r\n");
    return result;
}

static void time_available_cb(struct timeval *t)
{
    Serial.println("Got time adjustment from NTP!");
}

static void wifi_event_cb(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event) {
    case ARDUINO_EVENT_WIFI_READY:
        Serial.println("WiFi interface ready");
        break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
        Serial.println("Completed scan for access points");
        break;
    case ARDUINO_EVENT_WIFI_STA_START:
        Serial.println("WiFi client started");
        break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
        Serial.println("WiFi clients stopped");
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.println("Connected to access point");
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("Disconnected from WiFi access point");
        lv_msg_send(_BV(1), NULL);
        break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
        Serial.println("Authentication mode of access point has changed");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("Obtained IP address: ");
        Serial.println(WiFi.localIP());
        lv_msg_send(_BV(1), NULL);
        break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        Serial.println("Lost IP address and IP address is reset to 0");
        lv_msg_send(_BV(1), NULL);
        break;
    default: break;
    }
}


void setupWiFi()
{
    WiFi.onEvent(wifi_event_cb);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    configTzTime(DEFAULT_TIMEZONE, NTP_SERVER1, NTP_SERVER2);
    // set notification call-back function
    sntp_set_time_sync_notification_cb(time_available_cb);

}

// LilyGo  T-Deck  control backlight chip has 16 levels of adjustment range
// The adjustable range is 0~15, 0 is the minimum brightness, 15 is the maximum brightness
void setBrightness(uint8_t value)
{
    static uint8_t level = 0;
    static uint8_t steps = 16;
    if (value == 0) {
        digitalWrite(BOARD_BL_PIN, 0);
        delay(3);
        level = 0;
        return;
    }
    if (level == 0) {
        digitalWrite(BOARD_BL_PIN, 1);
        level = steps;
        delayMicroseconds(30);
    }
    int from = steps - level;
    int to = steps - value;
    int num = (steps + to - from) % steps;
    for (int i = 0; i < num; i++) {
        digitalWrite(BOARD_BL_PIN, 0);
        digitalWrite(BOARD_BL_PIN, 1);
    }
    level = value;
}

void setFlag(void)
{
    // check if the interrupt is enabled
    if (!enableInterrupt) {
        return;
    }
    // we got a packet, set the flag
    transmissionFlag = true;
}

bool setupRadio()
{
    digitalWrite(BOARD_SDCARD_CS, HIGH);
    digitalWrite(RADIO_CS_PIN, HIGH);
    digitalWrite(BOARD_TFT_CS, HIGH);
    SPI.end();
    SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI); //SD

    int state = radio.begin(RADIO_FREQ);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Start Radio success!");
    } else {
        Serial.print("Start Radio failed,code:");
        Serial.println(state);
        return false;
    }

    hasRadio = true;

    // set carrier frequency to 868.0 MHz
    if (radio.setFrequency(RADIO_FREQ) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        Serial.println(F("Selected frequency is invalid for this module!"));
        return false;
    }

    // set bandwidth to 125 kHz
    if (radio.setBandwidth(RADIO_BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        Serial.println(F("Selected bandwidth is invalid for this module!"));
        return false;
    }

    // set spreading factor to 10
    if (radio.setSpreadingFactor(RADIO_SF) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        Serial.println(F("Selected spreading factor is invalid for this module!"));
        return false;
    }

    // set coding rate to 6
    if (radio.setCodingRate(RADIO_CR) == RADIOLIB_ERR_INVALID_CODING_RATE) {
        Serial.println(F("Selected coding rate is invalid for this module!"));
        return false;
    }

    // set LoRa sync word to 0xAB
    if (radio.setSyncWord(0xAB) != RADIOLIB_ERR_NONE) {
        Serial.println(F("Unable to set sync word!"));
        return false;
    }

    // set output power to 10 dBm (accepted range is -17 - 22 dBm)
    if (radio.setOutputPower(RADIO_TX_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        Serial.println(F("Selected output power is invalid for this module!"));
        return false;
    }

    // set over current protection limit to 140 mA (accepted range is 45 - 140 mA)
    // NOTE: set value to 0 to disable overcurrent protection
    if (radio.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        Serial.println(F("Selected current limit is invalid for this module!"));
        return false;
    }

    // set LoRa preamble length to 15 symbols (accepted range is 0 - 65535)
    if (radio.setPreambleLength(15) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
        Serial.println(F("Selected preamble length is invalid for this module!"));
        return false;
    }

    // disable CRC
    if (radio.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        Serial.println(F("Selected CRC is invalid for this module!"));
        return false;
    }

    // set the function that will be called
    // when new packet is received
    radio.setDio1Action(setFlag);
    return true;

}

bool setupSD()
{
    digitalWrite(BOARD_SDCARD_CS, HIGH);
    digitalWrite(RADIO_CS_PIN, HIGH);
    digitalWrite(BOARD_TFT_CS, HIGH);

    if (SD.begin(BOARD_SDCARD_CS, SPI, 800000U)) {
        uint8_t cardType = SD.cardType();
        if (cardType == CARD_NONE) {
            Serial.println("No SD_MMC card attached");
            return false;
        } else {
            Serial.print("SD_MMC Card Type: ");
            if (cardType == CARD_MMC) {
                Serial.println("MMC");
            } else if (cardType == CARD_SD) {
                Serial.println("SDSC");
            } else if (cardType == CARD_SDHC) {
                Serial.println("SDHC");
            } else {
                Serial.println("UNKNOWN");
            }
            uint32_t cardSize = SD.cardSize() / (1024 * 1024);
            uint32_t cardTotal = SD.totalBytes() / (1024 * 1024);
            uint32_t cardUsed = SD.usedBytes() / (1024 * 1024);
            Serial.printf("SD Card Size: %lu MB\n", cardSize);
            Serial.printf("Total space: %lu MB\n",  cardTotal);
            Serial.printf("Used space: %lu MB\n",   cardUsed);
            return true;
        }
    }
    return false;
}

bool setupCoder()
{
    uint32_t ret_val = ESP_OK;

    Wire.beginTransmission(ES7210_ADDR);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.println("ES7210 address not found"); return false;
    }

    audio_hal_codec_config_t cfg = {
        .adc_input = AUDIO_HAL_ADC_INPUT_ALL,
        .codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE,
        .i2s_iface =
        {
            .mode = AUDIO_HAL_MODE_SLAVE,
            .fmt = AUDIO_HAL_I2S_NORMAL,
            .samples = AUDIO_HAL_16K_SAMPLES,
            .bits = AUDIO_HAL_BIT_LENGTH_16BITS,
        },
    };

    ret_val |= es7210_adc_init(&Wire, &cfg);
    ret_val |= es7210_adc_config_i2s(cfg.codec_mode, &cfg.i2s_iface);
    ret_val |= es7210_adc_set_gain(
                   (es7210_input_mics_t)(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2),
                   (es7210_gain_value_t)GAIN_6DB);
    ret_val |= es7210_adc_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);
    return ret_val == ESP_OK;

}

void taskPlaySong(void *p)
{
    while (1) {
        if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE ) {
            playTTS("hello.mp3");
            xSemaphoreGive( xSemaphore );
        }
        vTaskSuspend(NULL);
    }
}

void loopGPS()
{
    static  uint32_t gps_update_interval = 0;

    unsigned long start = millis();
    do {
        while (SerialGPS.available()) {
            int c = SerialGPS.read();
            // Serial.write(c);
            gps.encode(c);
        }
    } while (millis() - start < 20);

    if (gps_update_interval < millis()) {
        updateGPS(gps.location.lat(),
                  gps.location.lng(),
                  gps.date.year(),
                  gps.date.month(),
                  gps.date.day(),
                  gps.time.hour(),
                  gps.time.minute(),
                  gps.time.second(),
                  gps.speed.value(),
                  gps.charsProcessed());
        gps_update_interval = millis()  + 3000;
    }
}

void loopRadio()
{
    char buf[256];
    static uint32_t senderInterval = 0;
    if (!hasRadio) {
        return ;
    }
    if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        digitalWrite(BOARD_SDCARD_CS, HIGH);
        digitalWrite(RADIO_CS_PIN, HIGH);
        digitalWrite(BOARD_TFT_CS, HIGH);

        if (sender) {
            if ((millis() - senderInterval) > configTxInterval) {
                // check if the previous transmission finished
                if (transmissionFlag) {
                    // disable the interrupt service routine while
                    // processing the data
                    enableInterrupt = false;
                    // reset flag
                    transmissionFlag = false;

                    if (transmissionState == RADIOLIB_ERR_NONE) {
                        // packet was successfully sent
                        // Serial.println(F("transmission finished!"));
                        // NOTE: when using interrupt-driven transmit method,
                        //       it is not possible to automatically measure
                        //       transmission data rate using getDataRate()
                    } else {
                        Serial.print(F("failed, code "));
                        Serial.println(transmissionState);
                    }

                    snprintf(buf, 256, "TX %u %s\n", sendCount,
                             transmissionState == RADIOLIB_ERR_NONE ? "finished" : "failed");

                    setLoRaMessage(buf);

                    // Serial.println(buf);

                    // you can also transmit byte array up to 256 bytes long
                    transmissionState = radio.startTransmit(String(sendCount++).c_str());

                    // we're ready to send more packets,
                    // enable interrupt service routine
                    enableInterrupt = true;
                }
                // snprintf(dispSenderBuff, sizeof(dispSenderBuff), "TX: %u", sendCount);

                senderInterval = millis();
            }
        } else {

            String recv;

            // check if the flag is set
            if (transmissionFlag) {
                // disable the interrupt service routine while
                // processing the data
                enableInterrupt = false;

                // reset flag
                transmissionFlag = false;

                // you can read received data as an Arduino String
                // int state = radio.readData(recv);

                // you can also read received data as byte array
                /*
                */
                int state = radio.readData(recv);
                if (state == RADIOLIB_ERR_NONE) {


                    // packet was successfully received
                    Serial.print(F("[RADIO] Received packet!"));

                    // print data of the packet
                    Serial.print(F(" Data:"));
                    Serial.print(recv);

                    // print RSSI (Received Signal Strength Indicator)
                    Serial.print(F(" RSSI:"));
                    Serial.print(radio.getRSSI());
                    Serial.print(F(" dBm"));
                    // snprintf(dispRecvicerBuff[1], sizeof(dispRecvicerBuff[1]), "RSSI:%.2f dBm", radio.getRSSI());

                    // print SNR (Signal-to-Noise Ratio)
                    Serial.print(F("  SNR:"));
                    Serial.print(radio.getSNR());
                    Serial.println(F(" dB"));


                    snprintf(buf, 256, "RX:%s RSSI:%.2f SNR:%.2f\n", recv.c_str(), radio.getRSSI(), radio.getSNR());

                    setLoRaMessage(buf);

                } else if (state ==  RADIOLIB_ERR_CRC_MISMATCH) {
                    // packet was received, but is malformed
                    Serial.println(F("CRC error!"));

                } else {
                    // some other error occurred
                    Serial.print(F("failed, code "));
                    Serial.println(state);
                }
                // put module back to listen mode
                radio.startReceive();

                // we're ready to receive more packets,
                // enable interrupt service routine
                enableInterrupt = true;
            }
        }
        xSemaphoreGive( xSemaphore );
    }
}

void serialToScreen(lv_obj_t *parent, String string,  bool result)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_scroll_dir(cont, LV_DIR_NONE);
    lv_obj_set_size(cont, LV_PCT(100), lv_font_get_line_height(&lv_font_montserrat_28) + 2 );

    lv_obj_t *label1 = lv_label_create(cont);
    lv_label_set_recolor(label1, true);
    lv_label_set_text(label1, string.c_str());
    lv_obj_align(label1, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *label = lv_label_create(cont);
    lv_label_set_recolor(label, true);
    lv_label_set_text(label, result ? "#FFFFFF [# #00ff00 PASS# #FFFFFF ]#" : "#FFFFFF [# #ff0000  FAIL# #FFFFFF ]#");
    lv_obj_align(label, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_scroll_to_y(parent, lv_disp_get_ver_res(NULL), LV_ANIM_ON);

    int i = 200;
    while (i--) {
        lv_task_handler();
        delay(1);
    }
}

bool checkKb()
{
    int retry = 3;
    do {
        Wire.requestFrom(0x55, 1);
        if (Wire.read() != -1) {
            return true;
        }
    } while (retry--);
    return false;
}

#ifdef USE_ESP_VAD
void vadTask(void *params)
{
    Serial.println("vadTask(void *params)");

    // vTaskSuspend(NULL);
    while (1) {
        size_t read_len = 0;
        if (i2s_read(MIC_I2S_PORT, (char *) vad_buff, vad_buffer_size, &read_len, portMAX_DELAY) == ESP_OK) {
            // if (watch.readMicrophone((char *) vad_buff, vad_buffer_size, &read_len)) {
            // Feed samples to the VAD process and get the result
#if  ESP_IDF_VERSION_VAL(4,4,1) == ESP_IDF_VERSION
            vad_state_t vad_state = vad_process(vad_inst, vad_buff);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,4,1) && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
            vad_state_t vad_state = vad_process(vad_inst, vad_buff, MIC_I2S_SAMPLE_RATE, VAD_FRAME_LENGTH_MS);
#else
#error "ESP VAD Not support Version > V5.0.0 , please use IDF V4.4.4"
#endif
            if (vad_state == VAD_SPEECH) {
                Serial.print(millis());
                // Serial.println(" -> Noise detected!!!");
                updateNoiseLabel(vad_detected_counter++);
                last_update_millis = millis();
            }

            if (millis() - last_update_millis > 5000) {
                last_update_millis = millis();
                vad_detected_counter = 0;
            }
        }
        delay(30);
    }
}

#else

void audioLoopbackTask(void *params)
{
    vTaskSuspend(NULL);
    while (1) {
        delay(5);
        size_t bytes_read = 0, bytes_write = 0;
        memset(loopbackBuffer, 0, sizeof(loopbackBuffer));
        i2s_read(MIC_I2S_PORT, loopbackBuffer, sizeof(loopbackBuffer), &bytes_read, 15);
        if (bytes_read) {
            i2s_write(SPK_I2S_PORT, loopbackBuffer, bytes_read, &bytes_write, 15);
        }
    }
    vTaskDelete(NULL);
}

#endif

void setupMicrophoneI2S(i2s_port_t  i2s_ch)
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = MIC_I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        .bits_per_chan = I2S_BITS_PER_CHAN_16BIT,
        .chan_mask = (i2s_channel_t)(I2S_TDM_ACTIVE_CH0 | I2S_TDM_ACTIVE_CH1 |
                                     I2S_TDM_ACTIVE_CH2 | I2S_TDM_ACTIVE_CH3),
        .total_chan = 4,
    };
    i2s_pin_config_t pin_config = {0};
    pin_config.data_in_num = BOARD_ES7210_DIN;
    pin_config.mck_io_num = BOARD_ES7210_MCLK;
    pin_config.bck_io_num = BOARD_ES7210_SCK;
    pin_config.ws_io_num = BOARD_ES7210_LRCK;
    pin_config.data_out_num = -1;
    i2s_driver_install(i2s_ch, &i2s_config, 0, NULL);
    i2s_set_pin(i2s_ch, &pin_config);
    i2s_zero_dma_buffer(i2s_ch);

#ifdef USE_ESP_VAD
    // Initialize esp-sr vad detected
#if ESP_IDF_VERSION_VAL(4,4,1) == ESP_IDF_VERSION
    vad_inst = vad_create(VAD_MODE_0, MIC_I2S_SAMPLE_RATE, VAD_FRAME_LENGTH_MS);
#elif  ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,4,1)
    vad_inst = vad_create(VAD_MODE_0);
#else
#error "No support this version."
#endif
    vad_buff = (int16_t *)ps_malloc(vad_buffer_size);
    if (vad_buff == NULL) {
        while (1) {
            Serial.println("Memory allocation failed!");
            delay(1000);
        }
    }
    xTaskCreate(vadTask, "vad", 8 * 1024, NULL, 12, &vadTaskHandler);
#else
    // xTaskCreate(audioLoopbackTask, "vad", 8 * 1024, NULL, 12, &vadTaskHandler);
#endif

}


void playTTS(const char *filename)
{
    bool findMp3 = false;
    if (SD.exists("/" + String(filename))) {
        findMp3 = audio.connecttoFS(SD, filename);
    } else if (SPIFFS.exists("/" + String(filename))) {
        findMp3 = audio.connecttoFS(SPIFFS, filename);
    }
    if (findMp3) {
        while (audio.isRunning()) {
            audio.loop();
            delay(3);
        }
    }
}

void setupAmpI2S(i2s_port_t  i2s_ch)
{
    audio.setPinout(BOARD_I2S_BCK, BOARD_I2S_WS, BOARD_I2S_DOUT);
    audio.setVolume(21);
}

void setTx()
{
    sender = radio.startTransmit("Hello World!") == RADIOLIB_ERR_NONE;
}

void setRx()
{
    sender = ! (radio.startReceive() == RADIOLIB_ERR_NONE);
}

void setFreq(float f)
{
    if (radio.setFrequency(f) != RADIOLIB_ERR_NONE) {
        Serial.println("setFrequency failed!");
    }
}

void setBandWidth(float bw)
{
    if (radio.setBandwidth(bw) != RADIOLIB_ERR_NONE) {
        Serial.println("setBandwidth failed!");
    }
}

void setTxPower(int16_t dBm)
{
    if (radio.setOutputPower(dBm) != RADIOLIB_ERR_NONE) {
        Serial.println("setOutputPower failed!");
    }
}

void setSenderInterval(uint32_t interval_ms)
{
    configTxInterval = interval_ms;
}

void soundPlay()
{
    if (playHandle) {
        vTaskResume(playHandle);
    }
}

void setup()
{
    Serial.begin(115200);

    Serial.println("T-DECK factory");

    //! The board peripheral power control pin needs to be set to HIGH when using the peripheral
    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

    //! Set CS on all SPI buses to high level during initialization
    pinMode(BOARD_SDCARD_CS, OUTPUT);
    pinMode(RADIO_CS_PIN, OUTPUT);
    pinMode(BOARD_TFT_CS, OUTPUT);

    digitalWrite(BOARD_SDCARD_CS, HIGH);
    digitalWrite(RADIO_CS_PIN, HIGH);
    digitalWrite(BOARD_TFT_CS, HIGH);

    pinMode(BOARD_SPI_MISO, INPUT_PULLUP);
    SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI); //SD

    pinMode(BOARD_BOOT_PIN, INPUT_PULLUP);
    pinMode(BOARD_TBOX_G02, INPUT_PULLUP);
    pinMode(BOARD_TBOX_G01, INPUT_PULLUP);
    pinMode(BOARD_TBOX_G04, INPUT_PULLUP);
    pinMode(BOARD_TBOX_G03, INPUT_PULLUP);

    //Add mutex to allow multitasking access
    xSemaphore = xSemaphoreCreateBinary();
    assert(xSemaphore);
    xSemaphoreGive( xSemaphore );


    Serial.print("Init display id:");
    Serial.println(USER_SETUP_ID);

    tft.begin();

    /**
     * * T-Deck-Plus and T-Deck display panels are different.
     * * This initialization is used to override the initialization parameters.
     * * It is used when the display is abnormal after the TFT_eSPI update. */
#if 0
    for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
        tft.writecommand(lcd_st7789v[i].cmd);
        for (int j = 0; j < (lcd_st7789v[i].len & 0x7f); j++) {
            tft.writedata(lcd_st7789v[i].data[j]);
        }

        if (lcd_st7789v[i].len & 0x80) {
            delay(120);
        }
    }
#endif

    tft.setRotation( 1 );
    tft.fillScreen(TFT_BLACK);

    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);

    touch.setPins(-1, BOARD_TOUCH_INT);
    touchDetected = touch.begin(Wire);
    if (touchDetected) {
        Serial.println("Init GT911 Sensor success!");

        // Keep high level when idle, and switch to low level when touched
        touch.setInterruptMode(LOW_LEVEL_QUERY);

        // Set touch max xy
        touch.setMaxCoordinates(320, 240);

        // Set swap xy
        touch.setSwapXY(true);

        // Set mirror xy
        touch.setMirrorXY(false, true);

    } else {
        Serial.println("Failed to find GT911 - check your wiring!");
    }

    kbDetected = checkKb();


    pinMode(BOARD_BL_PIN, OUTPUT);
    setBrightness(16);


    setupLvgl();

    // Show logo

    LV_FONT_DECLARE(logo_font);
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), LV_PART_MAIN);
    lv_obj_t *logo = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(logo, &logo_font, LV_PART_MAIN);
    lv_obj_set_style_text_color(logo, lv_color_white(), LV_PART_MAIN);
    lv_label_set_text(logo, "LilyGo\nT-Deck");
    lv_obj_center(logo);

    for (int i = 0; i <= 16; ++i) {
        setBrightness(i);
        lv_timer_handler();
        delay(30);
    }


    SPIFFS.begin();

    setupSD();

    setupRadio();

    setupCoder();

    setupAmpI2S(SPK_I2S_PORT);

    setupMicrophoneI2S(MIC_I2S_PORT);

    // Test screen
#ifdef ENABLE_TEST_IMG
    lv_obj_t *label;

    const lv_img_dsc_t *img_src[4] = {&image1, &image2, &image3, &image4};
    lv_obj_t *img = lv_img_create(lv_scr_act());
    lv_img_set_src(img, (void *)(img_src[3]));

    // Adjust backlight
    pinMode(BOARD_BL_PIN, OUTPUT);
    //T-Deck control backlight chip has 16 levels of adjustment range
    for (int i = 0; i < 16; ++i) {
        setBrightness(i);
        lv_task_handler();
        delay(30);
    }
    delay(4000);

    int i = 2;
    while (i >= 0) {
        lv_img_set_src(img, (void *)(img_src[i]));
        lv_task_handler();
        i--;
        delay(2000);
    }

    lv_obj_del(img);
#endif



#if 0
    main_count = lv_obj_create(lv_scr_act());
    lv_obj_set_style_bg_img_src(main_count, &image_output, LV_PART_MAIN);
    lv_obj_set_style_border_opa(main_count, LV_OPA_100, 0);
    lv_obj_set_style_radius(main_count, 0, 0);
    lv_obj_set_size(main_count, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(main_count, LV_FLEX_FLOW_COLUMN);
    lv_obj_center(main_count);

    // Show device state
    serialToScreen(main_count, "Keyboard C3", kbDetected);
    serialToScreen(main_count, "Capacitive Touch", touchDetected);
    serialToScreen(main_count, "Radio SX1262", hasRadio);
    if (SD.cardType() != CARD_NONE) {
        serialToScreen(main_count, "Mass storage #FFFFFF [# #00ff00  "
                       + String(SD.cardSize() / 1024 / 1024.0 )
                       + "MB# #FFFFFF ]#", true);
    } else {
        serialToScreen(main_count, "Mass storage", false);
    }

    uint32_t endTime = millis() + 5000;
    while (millis() < endTime) {
        lv_task_handler();
        delay(1);
    }

    lv_obj_clean(main_count);
    lv_obj_set_scroll_dir(main_count, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(main_count, LV_SCROLLBAR_MODE_OFF);
#endif

    setupWiFi();

    if (!setupGPS()) {
        SerialGPS.begin(38400, SERIAL_8N1, BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);
        uint32_t baudrate[] = {38400, 115200, 9600};
        // Restore factory settings
        for (int i = 0; i < 3; ++i) {
            Serial.printf("Use baudrate : %u\n", baudrate[i]);
            if (GPS_Recovery()) {
                break;
            }
            Serial.printf("Update baudrate : %u\n", baudrate[i]);
            SerialGPS.updateBaudRate(baudrate[i]);
        }
    }


    lv_obj_del(logo);

    setupUI();

    xTaskCreate(taskPlaySong, "play", 1024 * 4, NULL, 10, &playHandle);
}

void loop()
{
    if (enterSleep) {

        vTaskDelete(playHandle);
        playHandle = NULL;

#ifdef USE_ESP_VAD
        vTaskDelete(vadTaskHandler);
        vadTaskHandler = NULL;
#endif

        //LilyGo T-Deck control backlight chip has 16 levels of adjustment range
        for (int i = 16; i > 0; --i) {
            setBrightness(i);
            delay(30);
        }

        delay(1000);

        //If you need other peripherals to maintain power, please set the IO port to hold
        // gpio_hold_en((gpio_num_t)BOARD_POWERON);
        // gpio_deep_sleep_hold_en();

        touch.sleep();        //set touchpad enter sleep mode
        tft.writecommand(0x10);      //set display enter sleep mode
        SPI.end();
        Wire.end();
        esp_sleep_enable_ext1_wakeup(1ull << BOARD_BOOT_PIN, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_deep_sleep_start();
        //Deep sleep consumes approximately 240uA of current
    }

    loopRadio();
    loopGPS();
    lv_task_handler();
}



// !!! LVGL !!!
// !!! LVGL !!!
// !!! LVGL !!!
static void disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
    if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE ) {
        tft.startWrite();
        tft.setAddrWindow( area->x1, area->y1, w, h );
        tft.pushColors( ( uint16_t * )&color_p->full, w * h, false );
        tft.endWrite();
        lv_disp_flush_ready( disp );
        xSemaphoreGive( xSemaphore );
    }
}

static void mouse_read(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    static  int16_t last_x;
    static int16_t last_y;
    bool left_button_down = false;
    const uint8_t dir_pins[5] = {BOARD_TBOX_G02,
                                 BOARD_TBOX_G01,
                                 BOARD_TBOX_G04,
                                 BOARD_TBOX_G03,
                                 BOARD_BOOT_PIN
                                };
    static bool last_dir[5];
    uint8_t pos = 10;
    for (int i = 0; i < 5; i++) {
        bool dir = digitalRead(dir_pins[i]);
        if (dir != last_dir[i]) {
            last_dir[i] = dir;
            switch (i) {
            case 0:
                if (last_x < (lv_disp_get_hor_res(NULL) - image_emoji.header.w)) {
                    last_x += pos;
                }
                break;
            case 1:
                if (last_y > image_emoji.header.h) {
                    last_y -= pos;
                }
                break;
            case 2:
                if (last_x > image_emoji.header.w) {
                    last_x -= pos;
                }
                break;
            case 3:
                if (last_y < (lv_disp_get_ver_res(NULL) - image_emoji.header.h)) {
                    last_y += pos;
                }
                break;
            case 4:
                left_button_down = true;
                break;
            default:
                break;
            }
        }
    }
    // Serial.printf("indev:X:%04d  Y:%04d \n", last_x, last_y);
    /*Store the collected data*/
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = left_button_down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/*Read the touchpad*/
static void touchpad_read( lv_indev_drv_t *indev_driver, lv_indev_data_t *data )
{
    static int16_t x[5], y[5];
    data->state =  LV_INDEV_STATE_REL;
    if (touch.isPressed()) {
        uint8_t touched = touch.getPoint(x, y, 1);
        if (touched > 0) {
            data->state = LV_INDEV_STATE_PR;
            data->point.x = x[0];
            data->point.y = y[0];
        }
    }
}

// Read key value from esp32c3
static uint32_t keypad_get_key(void)
{
    static uint32_t retry = 0;
    char key_ch = 0;
    if (retry > 10) {
        lv_indev_delete(kb_indev);
        kb_indev = NULL;
        return 0;
    }
    Wire.beginTransmission(0x55);
    if (Wire.endTransmission() != 0) {
        Serial.println("Keyboard Failed!");
        retry++;
        return 0;
    }
    Wire.requestFrom(0x55, 1);
    while (Wire.available() > 0) {
        key_ch = Wire.read();
        retry = 0;
    }
    return key_ch;
}

/*Will be called by the library to read the mouse*/
static void keypad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static uint32_t last_key = 0;
    uint32_t act_key ;
    act_key = keypad_get_key();
    if (act_key != 0) {
        data->state = LV_INDEV_STATE_PR;
        Serial.printf("Key pressed : 0x%x\n", act_key);
        last_key = act_key;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    data->key = last_key;
}

static void setupLvgl()
{
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t *buf = (lv_color_t *)ps_malloc(LVGL_BUFFER_SIZE);
    if (!buf) {
        Serial.println("memory alloc failed!");
        delay(5000);
        assert(buf);
    }
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

    lv_init();

    lv_group_set_default(lv_group_create());

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, LVGL_BUFFER_SIZE );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );

    /*Change the following line to your display resolution*/
    disp_drv.hor_res = TFT_HEIGHT;
    disp_drv.ver_res = TFT_WIDTH;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 1;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the  input device driver*/

    /*Register a touchscreen input device*/
    if (touchDetected) {
        static lv_indev_drv_t indev_touchpad;
        lv_indev_drv_init( &indev_touchpad );
        indev_touchpad.type = LV_INDEV_TYPE_POINTER;
        indev_touchpad.read_cb = touchpad_read;
        touch_indev = lv_indev_drv_register( &indev_touchpad );
    }

    /*Register a mouse input device*/
    static lv_indev_drv_t indev_mouse;
    lv_indev_drv_init( &indev_mouse );
    indev_mouse.type = LV_INDEV_TYPE_POINTER;
    indev_mouse.read_cb = mouse_read;
    mouse_indev = lv_indev_drv_register( &indev_mouse );
    lv_indev_set_group(mouse_indev, lv_group_get_default());

    lv_obj_t *cursor_obj;
    cursor_obj = lv_img_create(lv_scr_act());         /*Create an image object for the cursor */
    lv_img_set_src(cursor_obj, &image_emoji);   /*Set the image source*/
    lv_indev_set_cursor(mouse_indev, cursor_obj);           /*Connect the image  object to the driver*/

    if (kbDetected) {
        Serial.println("Keyboard registered!!");
        /*Register a keypad input device*/
        static lv_indev_drv_t indev_keypad;
        lv_indev_drv_init(&indev_keypad);
        indev_keypad.type = LV_INDEV_TYPE_KEYPAD;
        indev_keypad.read_cb = keypad_read;
        kb_indev = lv_indev_drv_register(&indev_keypad);
        lv_indev_set_group(kb_indev, lv_group_get_default());
    }
}







uint8_t buffer[256];

int getAck(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID)
{
    uint16_t    ubxFrameCounter = 0;
    bool        ubxFrame = 0;
    uint32_t    startTime = millis();
    uint16_t    needRead;

    while (millis() - startTime < 800) {
        while (SerialGPS.available()) {
            int c = SerialGPS.read();
            switch (ubxFrameCounter) {
            case 0:
                if (c == 0xB5) {
                    ubxFrameCounter++;
                }
                break;
            case 1:
                if (c == 0x62) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 2:
                if (c == requestedClass) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 3:
                if (c == requestedID) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 4:
                needRead = c;
                ubxFrameCounter++;
                break;
            case 5:
                needRead |=  (c << 8);
                ubxFrameCounter++;
                break;
            case 6:
                if (needRead >= size) {
                    ubxFrameCounter = 0;
                    break;
                }
                if (SerialGPS.readBytes(buffer, needRead) != needRead) {
                    ubxFrameCounter = 0;
                } else {
                    return needRead;
                }
                break;

            default:
                break;
            }
        }
    }
    return 0;
}

static bool GPS_Recovery()
{
    uint8_t cfg_clear1[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1C, 0xA2};
    uint8_t cfg_clear2[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA1};
    uint8_t cfg_clear3[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1D, 0xB3};
    SerialGPS.write(cfg_clear1, sizeof(cfg_clear1));

    if (getAck(buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    SerialGPS.write(cfg_clear2, sizeof(cfg_clear2));
    if (getAck(buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }
    SerialGPS.write(cfg_clear3, sizeof(cfg_clear3));
    if (getAck(buffer, 256, 0x05, 0x01)) {
        Serial.println("Get ack successes!");
    }

    // UBX-CFG-RATE, Size 8, 'Navigation/measurement rate settings'
    uint8_t cfg_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
    SerialGPS.write(cfg_rate, sizeof(cfg_rate));
    if (getAck(buffer, 256, 0x06, 0x08)) {
        Serial.println("Get ack successes!");
    } else {
        return false;
    }
    return true;
}
