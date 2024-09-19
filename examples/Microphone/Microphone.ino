/**
 * @file      Microphone.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-04-11
 *
 */
#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_vad.h>
#include "es7210.h"
#include "utilities.h"

#if ESP_ARDUINO_VERSION != ESP_ARDUINO_VERSION_VAL(2,0,9)
void setup()
{
    Serial.begin(115200);
}
void loop()
{
    Serial.println("Can only run on arduino-esp32 core version 2.0.9"); delay(1000);
}
#else
#define VAD_SAMPLE_RATE_HZ              16000
#define VAD_FRAME_LENGTH_MS             30
#define VAD_BUFFER_LENGTH               (VAD_FRAME_LENGTH_MS * VAD_SAMPLE_RATE_HZ / 1000)
#define I2S_CH                          I2S_NUM_1

int16_t         *vad_buff;
vad_handle_t    vad_inst;
size_t          bytes_read;

void setup()
{
    Serial.begin(115200);

    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

    Serial.printf("psram size : %d kb\r\n", ESP.getPsramSize() / 1024);
    Serial.printf("FLASH size : %d kb\r\n", ESP.getFlashChipSize() / 1024);

    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);

    uint32_t ret_val = ESP_OK;

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



    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = VAD_SAMPLE_RATE_HZ,
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
        .chan_mask = (i2s_channel_t)(I2S_TDM_ACTIVE_CH0 | I2S_TDM_ACTIVE_CH1),
    };

    i2s_pin_config_t pin_config = {0};
    pin_config.bck_io_num = BOARD_ES7210_SCK;
    pin_config.ws_io_num = BOARD_ES7210_LRCK;
    pin_config.data_in_num = BOARD_ES7210_DIN;
    // pin_config.mck_io_num = BOARD_ES7210_MCLK;
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
    i2s_driver_install(I2S_CH, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_CH, &pin_config);
    i2s_zero_dma_buffer(I2S_CH);
    // i2s_start(I2S_NUM_1);


    vad_inst = vad_create(VAD_MODE_0);
    vad_buff = (int16_t *)malloc(VAD_BUFFER_LENGTH * sizeof(short));
    if (vad_buff == NULL) {
        while (1) {
            Serial.println("Memory allocation failed!");
            delay(1000);
        }
    }
}

void loop()
{
    i2s_read(I2S_CH, (char *)vad_buff, VAD_BUFFER_LENGTH * sizeof(short), &bytes_read, portMAX_DELAY);
    // Feed samples to the VAD process and get the result
    vad_state_t vad_state = vad_process(vad_inst, vad_buff, VAD_SAMPLE_RATE_HZ, VAD_FRAME_LENGTH_MS);
    if (vad_state == VAD_SPEECH) {
        Serial.print(millis());
        Serial.println("Speech detected");
    }
    delay(5);
}
#endif