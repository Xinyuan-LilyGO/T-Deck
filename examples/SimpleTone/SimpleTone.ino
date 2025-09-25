/**
 * @file      SimpleTone.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2025  ShenZhen XinYuan Electronic Technology Co., Ltd
 * @date      2025-09-25
 *
 */
#include <driver/i2s.h>
#include <Arduino.h>

//! The board peripheral power control pin needs to be set to HIGH when using the peripheral
#define BOARD_POWERON       10

// I2S configuration parameters
#define I2S_BCK_PIN 7 // Bit clock pin
#define I2S_WS_PIN 5 // Word select pin (LRCK)
#define I2S_DATA_PIN 6 // Data pin

// Sound parameters
#define SAMPLE_RATE 8000 // Sampling rate
#define BB_FREQ 1000 // BB sound frequency (Hz). 1000Hz sounds more like a beep.
#define AMPLITUDE 16384 // Amplitude (moderate volume)
#define BEEP_DURATION 100 // Duration of each beep (milliseconds)
#define INTERVAL 1000 // Interval between beeps (milliseconds)

unsigned long previousMillis = 0; // Record the last beep time
bool isBeeping = false; // Check if the sound is playing
unsigned long beepStartTime = 0; // Record the start time of this sound

void setup()
{
    Serial.begin(115200);

    //! The board peripheral power control pin needs to be set to HIGH when using the peripheral
    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // 单声道
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // 中断优先级
        .dma_buf_count = 8,
        .dma_buf_len = 64
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_DATA_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_set_pin(I2S_NUM_0, &pin_config);

}

void loop()
{
    unsigned long currentMillis = millis();

    if (!isBeeping && currentMillis - previousMillis >= INTERVAL) {
        isBeeping = true;
        beepStartTime = currentMillis;
        previousMillis = currentMillis;
    }

    if (isBeeping && currentMillis - beepStartTime >= BEEP_DURATION) {
        isBeeping = false;
    }

    if (isBeeping) {
        generateBeep();
    } else {
        generateSilence();
    }
}

void generateBeep()
{
    static uint32_t sample_counter = 0;
    const uint32_t samples_per_cycle = SAMPLE_RATE / BB_FREQ;

    int16_t samples[128];

    for (int i = 0; i < 128; i++) {
        if ((sample_counter % samples_per_cycle) < (samples_per_cycle / 2)) {
            samples[i] = AMPLITUDE;
        } else {
            samples[i] = -AMPLITUDE;
        }
        sample_counter++;
    }

    size_t bytes_written;
    i2s_write(I2S_NUM_0, samples, sizeof(samples), &bytes_written, portMAX_DELAY);
}

void generateSilence()
{
    int16_t samples[128] = {0};
    size_t bytes_written;
    i2s_write(I2S_NUM_0, samples, sizeof(samples), &bytes_written, portMAX_DELAY);
}
