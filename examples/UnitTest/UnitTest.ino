/**
 * @file      UnitTest.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-04-11
 *
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
#include <esp_vad.h>

#define TOUCH_MODULES_GT911
#include "TouchLib.h"
#include "utilities.h"
#include "AceButton.h"


using namespace ace_button;


#define USING_SX1262



#define VAD_SAMPLE_RATE_HZ              16000
#define VAD_FRAME_LENGTH_MS             30
#define VAD_BUFFER_LENGTH               (VAD_FRAME_LENGTH_MS * VAD_SAMPLE_RATE_HZ / 1000)
#define I2S_CH                          I2S_NUM_1

LV_IMG_DECLARE(image);
LV_IMG_DECLARE(image1);
LV_IMG_DECLARE(image2);
LV_IMG_DECLARE(image3);
LV_IMG_DECLARE(image4);
LV_IMG_DECLARE(mouse_cursor_icon); /*Declare the image file.*/



TouchLib touch(Wire, BOARD_I2C_SDA, BOARD_I2C_SCL, GT911_SLAVE_ADDRESS1);

#ifdef USING_SX1262
#define RADIO_FREQ          868.0
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#else
#define RADIO_FREQ          433.0
SX1268 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

TFT_eSPI        tft;
Audio           audio;
size_t          bytes_read;
uint8_t         status;
int16_t         *vad_buff;
vad_handle_t    vad_inst;
TaskHandle_t    playHandle = NULL;
TaskHandle_t    radioHandle = NULL;

AceButton   button;
bool        clicked = false;
bool        transmissionFlag = true;
bool        enableInterrupt = true;
int         transmissionState ;
bool        hasRadio = false;
bool        touchDected = false;
bool        kbDected = false;
bool        sender = true;
uint32_t    sendCount = 0;
uint32_t    runningMillis = 0;


lv_indev_t  *kb_indev = NULL;
lv_indev_t  *mouse_indev = NULL;
lv_indev_t  *touch_indev = NULL;
lv_group_t  *kb_indev_group;
lv_obj_t    *hw_ta;
lv_obj_t    *radio_ta;
lv_obj_t    *tv ;


void setupLvgl();


void setFlag(void)
{
    // check if the interrupt is enabled
    if (!enableInterrupt) {
        return;
    }
    // we got a packet, set the flag
    transmissionFlag = true;
}

void scanDevices(TwoWire *w)
{
    uint8_t err, addr;
    int nDevices = 0;
    uint32_t start = 0;
    for (addr = 1; addr < 127; addr++) {
        start = millis();
        w->beginTransmission(addr); delay(2);
        err = w->endTransmission();
        if (err == 0) {
            nDevices++;
            Serial.print("I2C device found at address 0x");
            if (addr < 16) {
                Serial.print("0");
            }
            Serial.print(addr, HEX);
            Serial.println(" !");

        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16) {
                Serial.print("0");
            }
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
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

    // set bandwidth to 250 kHz
    if (radio.setBandwidth(250.0) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        Serial.println(F("Selected bandwidth is invalid for this module!"));
        return false;
    }

    // set spreading factor to 10
    if (radio.setSpreadingFactor(10) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        Serial.println(F("Selected spreading factor is invalid for this module!"));
        return false;
    }

    // set coding rate to 6
    if (radio.setCodingRate(6) == RADIOLIB_ERR_INVALID_CODING_RATE) {
        Serial.println(F("Selected coding rate is invalid for this module!"));
        return false;
    }

    // set LoRa sync word to 0xAB
    if (radio.setSyncWord(0xAB) != RADIOLIB_ERR_NONE) {
        Serial.println(F("Unable to set sync word!"));
        return false;
    }

    // set output power to 10 dBm (accepted range is -17 - 22 dBm)
    if (radio.setOutputPower(17) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
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
                   (es7210_gain_value_t)GAIN_0DB);
    ret_val |= es7210_adc_set_gain(
                   (es7210_input_mics_t)(ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4),
                   (es7210_gain_value_t)GAIN_37_5DB);
    ret_val |= es7210_adc_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);
    return ret_val == ESP_OK;

}

void taskplaySong(void *p)
{
    while (1) {
        if (SD.exists("/key.mp3")) {
            const char *path = "key.mp3";
            audio.setPinout(BOARD_I2S_BCK, BOARD_I2S_WS, BOARD_I2S_DOUT);
            audio.setVolume(12);
            audio.connecttoFS(SD, path);
            Serial.printf("play %s\r\n", path);
            while (audio.isRunning()) {
                audio.loop();
            }
        }
        audio.stopSong();
        vTaskSuspend(NULL);
    }
}

void addMessage(const char *str)
{
    lv_textarea_add_text(hw_ta, str);
    uint32_t run = millis() + 200;
    while (millis() < run) {
        lv_task_handler();
        delay(5);
    }
}

void loopRadio()
{
    if (!hasRadio) {
        lv_textarea_set_text(radio_ta, "Radio not online !");
        return ;
    }

    digitalWrite(BOARD_SDCARD_CS, HIGH);
    digitalWrite(RADIO_CS_PIN, HIGH);
    digitalWrite(BOARD_TFT_CS, HIGH);

    char buf[256];
    if (lv_tabview_get_tab_act(tv) != 1) {
        return ;
    }
    if (strlen(lv_textarea_get_text(radio_ta)) >= lv_textarea_get_max_length(radio_ta)) {
        lv_textarea_set_text(radio_ta, "");
    }

    if (sender) {
        // Send data every 200 ms
        if (millis() - runningMillis > 1000) {
            // check if the previous transmission finished
            if (transmissionFlag) {
                // disable the interrupt service routine while
                // processing the data
                enableInterrupt = false;
                // reset flag
                transmissionFlag = false;

                if (transmissionState == RADIOLIB_ERR_NONE) {
                    // packet was successfully sent
                    Serial.println(F("transmission finished!"));
                    // NOTE: when using interrupt-driven transmit method,
                    //       it is not possible to automatically measure
                    //       transmission data rate using getDataRate()
                } else {
                    Serial.print(F("failed, code "));
                    Serial.println(transmissionState);
                }

                snprintf(buf, 256, "[ %u ]TX %u finished\n", millis() / 1000, sendCount);
                lv_textarea_add_text(radio_ta, buf);

                Serial.println(buf);

                // you can also transmit byte array up to 256 bytes long
                transmissionState = radio.startTransmit(String(sendCount++).c_str());

                // we're ready to send more packets,
                // enable interrupt service routine
                enableInterrupt = true;
            }
            // snprintf(dispSenderBuff, sizeof(dispSenderBuff), "TX: %u", sendCount);

            runningMillis = millis();
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

                lv_textarea_add_text(radio_ta, buf);


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
}

static void event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        Serial.printf("State: %s\n", lv_obj_has_state(obj, LV_STATE_CHECKED) ? "On" : "Off");
        if (hasRadio) {
            if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
                // RX
                lv_textarea_set_text(radio_ta, "");
                Serial.print(F("[Radio] Starting to listen ... "));
                int state = radio.startReceive();
                if (state == RADIOLIB_ERR_NONE) {
                    Serial.println(F("success!"));
                } else {
                    Serial.print(F("failed, code "));
                    Serial.println(state);
                }
                sender = !sender;
            } else {
                // TX
                lv_textarea_set_text(radio_ta, "");
                // send the first packet on this node
                Serial.print(F("[Radio] Sending first packet ... "));
                transmissionState = radio.startTransmit("Hello World!");
                sender = !sender;
            }
        } else {
            lv_textarea_set_text(radio_ta, "Radio is not online");
        }
    }

}

void factory_ui(lv_obj_t *parent)
{
    static lv_style_t lable_style;
    lv_style_init(&lable_style);
    lv_style_set_text_color(&lable_style, lv_color_white());

    static lv_style_t bg_style;
    lv_style_init(&bg_style);
    lv_style_set_text_color(&bg_style, lv_color_white());
    lv_style_set_bg_img_src(&bg_style, &image);
    lv_style_set_bg_opa(&bg_style, LV_OPA_100);

    tv = lv_tabview_create(parent, LV_DIR_TOP, 50);
    lv_obj_add_style(tv, &bg_style, LV_PART_MAIN);

    lv_obj_t *t1 = lv_tabview_add_tab(tv, "Hardware");
    lv_obj_t *t2 = lv_tabview_add_tab(tv, "Radio");
    lv_obj_t *t3 = lv_tabview_add_tab(tv, "Keyboard");


    static lv_style_t ta_bg_style;
    lv_style_init(&ta_bg_style);
    lv_style_set_text_color(&ta_bg_style, lv_color_white());
    lv_style_set_bg_opa(&ta_bg_style, LV_OPA_100);


    hw_ta = lv_textarea_create(t1);
    lv_textarea_set_cursor_click_pos(hw_ta, false);
    lv_textarea_set_text_selection(hw_ta, false);
    lv_obj_set_size(hw_ta, LV_HOR_RES, LV_VER_RES / 2);
    lv_textarea_set_text(hw_ta, "");
    lv_textarea_set_max_length(hw_ta, 1024);
    lv_obj_align(hw_ta, LV_ALIGN_TOP_MID, 0, 0);


    lv_obj_add_style(hw_ta, &ta_bg_style, LV_PART_ANY);

    radio_ta = lv_textarea_create(t2);
    lv_textarea_set_cursor_click_pos(radio_ta, false);
    lv_textarea_set_text_selection(radio_ta, false);
    lv_obj_set_size(radio_ta, LV_HOR_RES, LV_VER_RES / 2);
    lv_textarea_set_text(radio_ta, "");
    lv_textarea_set_max_length(radio_ta, 1024);
    lv_obj_align(radio_ta, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_add_style(radio_ta, &ta_bg_style, LV_PART_ANY);

    lv_obj_t *sw = lv_switch_create(t2);
    lv_obj_align_to(sw, radio_ta, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_obj_t *label = lv_label_create(t2);
    lv_label_set_text(label, "Tx");
    lv_obj_align_to(label, sw, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    lv_obj_add_style(label, &lable_style, LV_PART_MAIN);

    label = lv_label_create(t2);
    lv_label_set_text(label, "Rx");
    lv_obj_align_to(label, sw, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    lv_obj_add_style(label, &lable_style, LV_PART_MAIN);
    lv_obj_add_event_cb(sw, event_handler, LV_EVENT_VALUE_CHANGED, NULL);


    lv_obj_t *kb_ta = lv_textarea_create(t3);
    lv_textarea_set_cursor_click_pos(kb_ta, false);
    lv_textarea_set_cursor_pos(kb_ta, 0);
    lv_textarea_set_text_selection(kb_ta, false);
    lv_obj_set_size(kb_ta, LV_HOR_RES, LV_VER_RES / 2);
    lv_textarea_set_text(kb_ta, "");
    lv_textarea_set_max_length(kb_ta, 512);
    lv_obj_align(kb_ta, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_style(kb_ta, &ta_bg_style, LV_PART_ANY);

}


static bool getTouch(int16_t &x, int16_t &y);

bool checkKb()
{
    Wire.requestFrom(0x55, 1);
    return Wire.read() != -1;
}

void initBoard()
{
    bool ret = 0;

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
    pinMode(BOARD_TBOX_UP, INPUT_PULLUP);
    pinMode(BOARD_TBOX_DOWN, INPUT_PULLUP);
    pinMode(BOARD_TBOX_LEFT, INPUT_PULLUP);
    pinMode(BOARD_TBOX_RIGHT, INPUT_PULLUP);

    //Wakeup touch chip
    pinMode(BOARD_TOUCH_INT, OUTPUT);
    digitalWrite(BOARD_TOUCH_INT, HIGH);

    button.init();
    ButtonConfig *buttonConfig = button.getButtonConfig();
    buttonConfig->setEventHandler(handleEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);

    tft.begin();
    tft.setRotation( 1 );
    tft.fillScreen(TFT_BLACK);
    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);

    // Set touch int input
    pinMode(BOARD_TOUCH_INT, INPUT); delay(20);

    scanDevices(&Wire);

    touch.init();

    Wire.beginTransmission(GT911_SLAVE_ADDRESS1);
    ret = Wire.endTransmission() == 0;
    touchDected = ret;

    kbDected = checkKb();

    setupLvgl();

    // test image
    const lv_img_dsc_t *img_src[4] = {&image1, &image2, &image3, &image4};
    lv_obj_t *img = lv_img_create(lv_scr_act());
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
    lv_obj_set_width(label, 320);
    lv_label_set_text(label, "Press the key of the trackball in the middle of the board to enter the next picture");
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);

    int i = 3;
    while (i > 0) {
        lv_img_set_src(img, (void *)(img_src[i]));
        while (!clicked) {
            lv_task_handler(); delay(5);
            button.check();
        }
        i--;
        clicked = false;
    }

    lv_obj_del(label);
    lv_obj_del(img);


    factory_ui(lv_scr_act());


    char buf[256];
    Serial.print("Touch:"); Serial.println(ret);
    snprintf(buf, 256, "%s:%s\n", "Touch", ret == true ? "Successed" : "Failed");
    addMessage(buf);

    ret = setupSD();
    Serial.print("SDCard:"); Serial.println(ret);
    snprintf(buf, 256, "%s:%s\n", "SDCard", ret == true ? "Successed" : "Failed");
    addMessage(buf);

    ret = setupRadio();
    Serial.print("Radio:"); Serial.println(ret);
    snprintf(buf, 256, "%s:%s\n", "Radio", ret == true ? "Successed" : "Failed");
    addMessage(buf);

    ret = setupCoder();
    Serial.print("Decoder:"); Serial.println(ret);
    snprintf(buf, 256, "%s:%s\n", "Decoder", ret == true ? "Successed" : "Failed");
    addMessage(buf);

    Serial.print("Keyboard:"); Serial.println(kbDected);
    snprintf(buf, 256, "%s:%s\n", "Keyboard", kbDected == true ? "Successed" : "Failed");
    addMessage(buf);


    if (SD.exists("/winxp.mp3")) {
        const char *path = "winxp.mp3";
        audio.setPinout(BOARD_I2S_BCK, BOARD_I2S_WS, BOARD_I2S_DOUT);
        audio.setVolume(12);
        audio.connecttoFS(SD, path);
        Serial.printf("play %s\r\n", path);
        while (audio.isRunning()) {
            audio.loop();
        }
    }

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 16000,
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
        .chan_mask =
        (i2s_channel_t)(I2S_TDM_ACTIVE_CH0 | I2S_TDM_ACTIVE_CH1 |
                        I2S_TDM_ACTIVE_CH2 | I2S_TDM_ACTIVE_CH3),
        .total_chan = 4,
    };

    i2s_pin_config_t pin_config = {
        .mck_io_num = BOARD_ES7210_MCLK,
        .bck_io_num = BOARD_ES7210_SCK,
        .ws_io_num = BOARD_ES7210_LRCK,
        .data_in_num = BOARD_ES7210_DIN,
    };
    i2s_driver_install(I2S_CH, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_CH, &pin_config);
    i2s_zero_dma_buffer(I2S_CH);


    vad_inst = vad_create(VAD_MODE_0);
    vad_buff = (int16_t *)malloc(VAD_BUFFER_LENGTH * sizeof(short));
    if (vad_buff == NULL) {
        while (1) {
            Serial.println("Memory allocation failed!");
            delay(1000);
        }
    }

    // Wait until sound is detected before continuing
    uint32_t c = 0;
    while (1) {
        i2s_read(I2S_CH, (char *)vad_buff, VAD_BUFFER_LENGTH * sizeof(short), &bytes_read, portMAX_DELAY);
        // Feed samples to the VAD process and get the result
        vad_state_t vad_state = vad_process(vad_inst, vad_buff, VAD_SAMPLE_RATE_HZ, VAD_FRAME_LENGTH_MS);
        if (vad_state == VAD_SPEECH) {
            Serial.print(millis());
            Serial.println("Speech detected");
            c++;
            snprintf(buf, 256, "%s:%d\n", "Speech detected", c);
            addMessage(buf);
        }
        if (c >= 5)break;
        lv_task_handler();
        delay(5);
    }


    xTaskCreate(taskplaySong, "play", 1024 * 4, NULL, 10, &playHandle);

    i2s_driver_uninstall(I2S_CH);

    pinMode(BOARD_BOOT_PIN, INPUT);

    while (!digitalRead(BOARD_BOOT_PIN)) {
        Serial.println("BOOT HAS PRESSED!!!"); delay(500);
    }

    if (hasRadio) {
        if (sender) {
            transmissionState = radio.startTransmit("0");
            sendCount = 0;
            Serial.println("startTransmit!!!!");
        } else {
            int state = radio.startReceive();
            if (state == RADIOLIB_ERR_NONE) {
                Serial.println(F("success!"));
            } else {
                Serial.print(F("failed, code "));
                Serial.println(state);
            }
        }
    }


}

// !!! LVGL !!!
// !!! LVGL !!!
// !!! LVGL !!!
static void disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, false );
    tft.endWrite();
    lv_disp_flush_ready( disp );
}


static bool getTouch(int16_t &x, int16_t &y)
{
    uint8_t rotation = tft.getRotation();
    if (!touch.read()) {
        return false;
    }
    TP_Point t = touch.getPoint(0);
    switch (rotation) {
    case 1:
        x = t.y;
        y = tft.height() - t.x;
        break;
    case 2:
        x = tft.width() - t.x;
        y = tft.height() - t.y;
        break;
    case 3:
        x = tft.width() - t.y;
        y = t.x;
        break;
    case 0:
    default:
        x = t.x;
        y = t.y;
    }
    Serial.printf("R:%d X:%d Y:%d\n", rotation, x, y);
    return true;
}

static void mouse_read(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    static  int16_t last_x;
    static int16_t last_y;
    bool left_button_down = false;
    const uint8_t dir_pins[5] = {BOARD_TBOX_UP,
                                 BOARD_TBOX_DOWN,
                                 BOARD_TBOX_LEFT,
                                 BOARD_TBOX_RIGHT,
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
                if (last_x < (lv_disp_get_hor_res(NULL) - mouse_cursor_icon.header.w)) {
                    last_x += pos;
                }
                break;
            case 1:
                if (last_y > mouse_cursor_icon.header.h) {
                    last_y -= pos;
                }
                break;
            case 2:
                if (last_x > mouse_cursor_icon.header.w) {
                    last_x -= pos;
                }
                break;
            case 3:
                if (last_y < (lv_disp_get_ver_res(NULL) - mouse_cursor_icon.header.h)) {
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
    data->state = getTouch(data->point.x, data->point.y) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

// Read key value from esp32c3
static uint32_t keypad_get_key(void)
{
    char key_ch = 0;
    Wire.requestFrom(0x55, 1);
    while (Wire.available() > 0) {
        key_ch = Wire.read();
        if (key_ch != (char)0x00) {
            if (playHandle) {
                vTaskResume(playHandle);
            }
        }
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


void setupLvgl()
{
    static lv_disp_draw_buf_t draw_buf;

#ifndef BOARD_HAS_PSRAM
#define LVGL_BUFFER_SIZE    ( TFT_HEIGHT * 100 )
    static lv_color_t buf[ LVGL_BUFFER_SIZE ];
#else
#define LVGL_BUFFER_SIZE    (TFT_WIDTH * TFT_HEIGHT * sizeof(lv_color_t))
    static lv_color_t *buf = (lv_color_t *)ps_malloc(LVGL_BUFFER_SIZE);
    if (!buf) {
        Serial.println("menory alloc failed!");
        delay(5000);
        assert(buf);
    }
#endif


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
#ifdef BOARD_HAS_PSRAM
    disp_drv.full_refresh = 1;
#endif
    lv_disp_drv_register( &disp_drv );

    /*Initialize the  input device driver*/

    /*Register a touchscreen input device*/
    if (touchDected) {
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
    lv_img_set_src(cursor_obj, &mouse_cursor_icon);   /*Set the image source*/
    lv_indev_set_cursor(mouse_indev, cursor_obj);           /*Connect the image  object to the driver*/

    if (kbDected) {
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




void handleEvent(AceButton * /* button */, uint8_t eventType,
                 uint8_t /* buttonState */)
{
    switch (eventType) {
    case AceButton::kEventClicked:
        clicked = true;
        Serial.println("Clicked!");
        break;
    case AceButton::kEventLongPressed:

        Serial.println("ClickkEventLongPresseded!"); delay(2000);

#if TFT_BL !=  BOARD_BL_PIN
#error "Not using the already configured T-Deck file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
#error "Not using the already configured T-Deck file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
#error "Not using the already configured T-Deck file, please remove <Arduino/libraries/TFT_eSPI> and replace with <lib/TFT_eSPI>, please do not click the upgrade library button when opening sketches in ArduinoIDE versions 2.0 and above, otherwise the original configuration file will be replaced !!!"
#endif


        //If you need other peripherals to maintain power, please set the IO port to hold
        // gpio_hold_en((gpio_num_t)BOARD_POWERON);
        // gpio_deep_sleep_hold_en();

        // When sleeping, set the touch and display screen to sleep, and all other peripherals will be powered off
        pinMode(BOARD_TOUCH_INT, OUTPUT);
        digitalWrite(BOARD_TOUCH_INT, LOW); //Before touch to set sleep, it is necessary to set INT to LOW
        touch.enableSleep();        //set touchpad enter sleep mode
        tft.writecommand(0x10);     //set disaplay enter sleep mode
        delay(2000);
        esp_sleep_enable_ext1_wakeup(1ull << BOARD_BOOT_PIN, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_deep_sleep_start();
        //Deep sleep consumes approximately 240uA of current
        break;
    }
}


void setup()
{
    initBoard();
}


void loop()
{
    button.check();
    loopRadio();
    lv_task_handler();
    delay(5);
}



























