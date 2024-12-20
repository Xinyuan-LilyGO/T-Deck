/**
 * @file      ui.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  ShenZhen XinYuan Electronic Technology Co., Ltd
 * @date      2024-09-12
 *
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SD.h>
#include "lvgl.h"
#include "utilities.h"
#include <vector>
#include "config.h"


typedef struct {
    lv_obj_t *label_lat;
    lv_obj_t *label_lng;
    lv_obj_t *label_date;
    lv_obj_t *label_time;
    lv_obj_t *label_speed;
    lv_obj_t *label_processchar;
} Deck_GPS_t;

Deck_GPS_t sub_gps_val;

typedef struct {
    lv_obj_t *label_radio_state;
    lv_obj_t *label_radio_msg;

} Deck_Radio_t;

Deck_Radio_t sub_radio_val;
static lv_obj_t *sound_vad_label;

std::vector<lv_obj_t *>sub_section;

enum {
    LV_MENU_ITEM_BUILDER_VARIANT_1,
    LV_MENU_ITEM_BUILDER_VARIANT_2
};
typedef uint8_t lv_menu_builder_variant_t;

static void back_event_handler(lv_event_t *e);
static void switch_handler(lv_event_t *e);
lv_obj_t *root_page;
static lv_obj_t *create_text(lv_obj_t *parent, const char *icon, const char *txt,
                             lv_menu_builder_variant_t builder_variant);
static lv_obj_t *create_slider(lv_obj_t *parent,
                               const char *icon, const char *txt, int32_t min, int32_t max,
                               int32_t val, lv_event_cb_t cb, lv_event_code_t filter);

static lv_obj_t *create_button(lv_obj_t *parent, const char *icon, const char *txt, lv_event_cb_t cb);

static lv_obj_t *create_switch(lv_obj_t *parent,
                               const char *icon, const char *txt, bool chk, lv_event_cb_t cb);
static lv_obj_t *create_label(lv_obj_t *parent, const char *icon, const char *txt, const char *default_text);
static lv_obj_t *create_dropdown(lv_obj_t *parent, const char *icon, const char *txt, const char *options, uint8_t default_sel, lv_event_cb_t cb);

extern void setBrightness(uint8_t value);
extern void setTx();
extern void setRx();
extern void setFreq(float f);
extern void soundPlay();
extern void setBandWidth(float bw);
extern void setTxPower(int16_t dBm);
extern void setSenderInterval(uint32_t interval_ms);
extern bool        enterSleep;

extern String gps_model;


void lv_brightness_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint8_t val =  lv_slider_get_value(obj);
    setBrightness(val);
}



void lv_background_opa_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint8_t val =  lv_slider_get_value(obj);
    std::vector<lv_obj_t *>::iterator it;
    for (it = sub_section.begin(); it != sub_section.end(); it++) {
        lv_obj_set_style_bg_opa(*it, val, LV_PART_MAIN);
    }
}

void lv_radio_tx_event_cb(lv_event_t *e)
{
    Serial.println("set TX");
    setTx();
    lv_label_set_text(sub_radio_val.label_radio_msg, "RF Tx Staring");
    lv_obj_t *obj =  (lv_obj_t *)lv_event_get_target(e);
    lv_obj_t *swRx = (lv_obj_t *)lv_obj_get_user_data(obj);
    lv_obj_clear_state(swRx, LV_STATE_CHECKED);
}

void lv_radio_rx_event_cb(lv_event_t *e)
{
    Serial.println("set RX");
    setRx();
    lv_label_set_text(sub_radio_val.label_radio_msg, "RF monitoring");
    lv_obj_t *obj =  (lv_obj_t *)lv_event_get_target(e);
    lv_obj_t *swTx = (lv_obj_t *)lv_obj_get_user_data(obj);
    lv_obj_clear_state(swTx, LV_STATE_CHECKED);
}

void speaker_play_event(lv_event_t *e)
{
    soundPlay();
}

void sleep_event_cb(lv_event_t *e)
{
    enterSleep = true;
}

void setLoRaMessage(const char *text)
{
    lv_label_set_text(sub_radio_val.label_radio_msg, text);
}

void updateGPS(double lat, double lng,
               uint16_t year, uint8_t month, uint8_t day,
               uint8_t hour, uint8_t minute, uint8_t second,
               double speed, uint32_t rx_char )
{
    lv_label_set_text_fmt(sub_gps_val.label_lng, "%.6f", lng);
    lv_label_set_text_fmt(sub_gps_val.label_lat, "%.6f", lat);

    lv_label_set_text_fmt(sub_gps_val.label_date, "%u/%u/%u", year, month, day);
    lv_label_set_text_fmt(sub_gps_val.label_time, "%u:%u:%u", hour, minute, second);
    lv_label_set_text_fmt(sub_gps_val.label_speed, "%.6f", speed);
    lv_label_set_text_fmt(sub_gps_val.label_processchar, "%u", rx_char);
}

void updateNoiseLabel(uint32_t cnt)
{
    if (sound_vad_label) {
        lv_label_set_text_fmt(sound_vad_label, "%u", cnt);
    }
}

const char *radio_freq_list =
    "433MHZ\n"
    "470MHZ\n"
    "850MHZ\n"
    "868MHZ\n"
    "915MHZ\n"
    "923MHZ";
const float freq_list[] = {433.0, 470.0, 850.0, 868.0, 915.0, 923.0};

const char *radio_bandwidth_list =
    "125KHz\n"
    "250KHz\n"
    "500KHz";
const float bandwidth_list[] = {125.0, 250.0, 500.0};


const char *radio_power_level_list =
    "2dBm\n"
    "5dBm\n"
    "10dBm\n"
    "12dBm\n"
    "17dBm\n"
    "20dBm\n"
    "22dBm";
const float radio_power_args_list[] = {2, 5, 10, 12, 17, 20, 22};

const char   *radio_tx_interval_list =
    "100ms\n"
    "200ms\n"
    "500ms\n"
    "1000ms\n"
    "2000ms\n"
    "3000ms\n"
    "5000ms";
const float radio_tx_interval_args_list[] = {100, 200, 500, 1000, 2000, 3000, 5000};

static void radio_freq_cb(lv_event_t *e)
{
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    uint32_t index = lv_dropdown_get_selected(obj);
    Serial.printf("Option: %s id:%u\n", buf, index);
    setFreq(freq_list[index]);
}


static void radio_power_cb(lv_event_t *e)
{
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    uint32_t index = lv_dropdown_get_selected(obj);
    Serial.printf("Option: %s id:%u\n", buf, index);
    setTxPower(radio_power_args_list[index]);

}

static void radio_bandwidth_cb(lv_event_t *e)
{
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    uint32_t index = lv_dropdown_get_selected(obj);
    Serial.printf("Option: %s id:%u\n", buf, index);
    setBandWidth(bandwidth_list[index]);
}

static void radio_interval_cb(lv_event_t *e)
{
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    uint32_t index = lv_dropdown_get_selected(obj);
    Serial.printf("Option: %s id:%u\n", buf, index);
    setSenderInterval(radio_tx_interval_args_list[index]);
}


LV_IMG_DECLARE(wallpaper);
extern bool  hasRadio;

void setupUI(void)
{
    lv_obj_t *menu = lv_menu_create(lv_scr_act());
    lv_obj_set_style_bg_img_src(menu, &wallpaper, LV_PART_MAIN);

    lv_color_t bg_color = lv_obj_get_style_bg_color(menu, 0);
    if (lv_color_brightness(bg_color) > 127) {
        lv_obj_set_style_bg_color(menu, lv_color_darken(lv_obj_get_style_bg_color(menu, 0), 10), 0);
    } else {
        lv_obj_set_style_bg_color(menu, lv_color_darken(lv_obj_get_style_bg_color(menu, 0), 50), 0);
    }
    lv_menu_set_mode_root_back_btn(menu, LV_MENU_ROOT_BACK_BTN_DISABLED);
    // lv_obj_add_event_cb(menu, back_event_handler, LV_EVENT_CLICKED, menu);
    lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(menu);

    lv_obj_t *cont;
    lv_obj_t *section;

    /*Create sub pages*/
    // !RADIO
    lv_obj_t *sub_mechanics_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_mechanics_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_mechanics_page);
    section = lv_menu_section_create(sub_mechanics_page);
    sub_section.push_back(section);

    if (hasRadio) {

        lv_obj_t *swTx = create_switch(section, LV_SYMBOL_UP, "Tx", true, lv_radio_tx_event_cb);
        lv_obj_t *swRx = create_switch(section, LV_SYMBOL_DOWN, "Rx", false, lv_radio_rx_event_cb);

        lv_obj_set_user_data(swTx, swRx);
        lv_obj_set_user_data(swRx, swTx);

        create_label(section, LV_SYMBOL_LOOP, "Message", NULL);
        sub_radio_val.label_radio_msg = create_label(section, NULL, NULL, "N.A");

        lv_obj_t *sub_rf_setting_page = lv_menu_page_create(menu, NULL);
        lv_obj_set_style_pad_hor(sub_rf_setting_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
        section = lv_menu_section_create(sub_rf_setting_page);
        sub_section.push_back(section);

        create_dropdown(section, NULL, "Freq", radio_freq_list, 2, radio_freq_cb);
        create_dropdown(section, NULL, "BandWidth", radio_bandwidth_list, 0, radio_bandwidth_cb);
        create_dropdown(section, NULL, "TxPower", radio_power_level_list, 6, radio_power_cb);
        create_dropdown(section, NULL, "Interval", radio_tx_interval_list, 3, radio_interval_cb);

        section = lv_menu_section_create(sub_mechanics_page);
        sub_section.push_back(section);

        cont = create_text(section, LV_SYMBOL_SETTINGS, "Radio Configure", LV_MENU_ITEM_BUILDER_VARIANT_1);
        lv_menu_set_load_page_event(menu, cont, sub_rf_setting_page);

    } else {
        lv_obj_t *cont = lv_obj_create(sub_mechanics_page);
        lv_obj_set_size(cont, LV_PCT(100), LV_PCT(95));
        sub_section.push_back(cont);
        lv_obj_t *label = lv_label_create(cont);
        lv_label_set_text(label, "Radio is offline");
        lv_obj_center(label);
    }


    // !SOUND
    lv_obj_t *sub_sound_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_sound_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_sound_page);
    section = lv_menu_section_create(sub_sound_page);
    sub_section.push_back(section);

    create_button(section, LV_SYMBOL_AUDIO, "Speaker", speaker_play_event);
    sound_vad_label =  create_label(section, LV_SYMBOL_VOLUME_MAX, "Microphone", "N.A");


    // !DISPLAY
    lv_obj_t *sub_display_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_display_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_display_page);
    section = lv_menu_section_create(sub_display_page);
    sub_section.push_back(section);

    create_slider(section, LV_SYMBOL_SETTINGS, "Brightness", 1, 16, 16, lv_brightness_cb, LV_EVENT_VALUE_CHANGED);
    create_slider(section, LV_SYMBOL_SETTINGS, "Background", 0, 255, DEFAULT_OPA, lv_background_opa_cb, LV_EVENT_VALUE_CHANGED);


    // !GPS
    lv_obj_t *sub_gps_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_gps_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_gps_page);
    section = lv_menu_section_create(sub_gps_page);
    sub_section.push_back(section);

    create_label(section, LV_SYMBOL_GPS, "Model", gps_model.c_str());
    sub_gps_val.label_lat = create_label(section, LV_SYMBOL_GPS, "lat", "N.A");
    sub_gps_val.label_lng = create_label(section, LV_SYMBOL_GPS, "lng", "N.A");
    sub_gps_val.label_speed = create_label(section, LV_SYMBOL_SETTINGS, "Speed", "N.A");
    sub_gps_val.label_date = create_label(section, LV_SYMBOL_SETTINGS, "Date", "N.A");
    sub_gps_val.label_time = create_label(section, LV_SYMBOL_SETTINGS, "Time", "N.A");
    sub_gps_val.label_processchar = create_label(section, LV_SYMBOL_SETTINGS, "Rx", "N.A");

    //! KEYBOARD
    lv_obj_t *sub_kb_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_kb_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_kb_page);
    section = lv_menu_section_create(sub_kb_page);
    sub_section.push_back(section);

    lv_obj_t *radio_ta = lv_textarea_create(sub_kb_page);
    lv_textarea_set_cursor_click_pos(radio_ta, false);
    lv_textarea_set_text_selection(radio_ta, false);
    lv_obj_set_size(radio_ta, LV_PCT(100), LV_PCT(95));
    lv_textarea_set_text(radio_ta, "");
    lv_textarea_set_max_length(radio_ta, 1024);
    sub_section.push_back(radio_ta);


    lv_timer_create([](lv_timer_t *t) {
        extern lv_indev_t  *kb_indev ;
        if (NULL == kb_indev) {
            lv_obj_t *radio_ta = (lv_obj_t *)t->user_data;
            lv_textarea_set_text(radio_ta, "Keyboard is offline");
            lv_obj_invalidate(radio_ta);
        }
    }, 3000, radio_ta);

    //! SD
    // lv_obj_t *sub_sd_page = lv_menu_page_create(menu, NULL);
    // lv_obj_set_style_pad_hor(sub_sd_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    // lv_menu_separator_create(sub_sd_page);
    // section = lv_menu_section_create(sub_sd_page);

    //! SETTING
    lv_obj_t *sub_setting_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_setting_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_setting_page);
    section = lv_menu_section_create(sub_setting_page);
    sub_section.push_back(section);


    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    static char buffer [128] = {0};
    snprintf(buffer, 128, "%X:%X:%X:%X:%X:%X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    create_label(section, NULL, "EFUS MAC", buffer);

    const char *wifi_name = WIFI_SSID;
    create_label(section, NULL, "WiFi SSID", wifi_name);



    lv_obj_t *ntp_datetime =  create_label(section, NULL, "NTP Datetime", "00:00:00");

    lv_timer_create([](lv_timer_t *t) {
        lv_obj_t *ntp_datetime = (lv_obj_t *)t->user_data;
        if (WiFi.isConnected()) {
            time_t now;
            struct tm  timeinfo;
            time(&now);
            localtime_r(&now, &timeinfo);
            char datetime[128] = {0};
            snprintf(datetime, 128, "%d/%d/%d %d:%d:%d", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            lv_label_set_text_fmt(ntp_datetime, "%s", datetime);
        }
    }, 1000, ntp_datetime);


    const char *wifi_ip = "N.A";
    lv_obj_t *label =    create_label(section, NULL, "IP", wifi_ip);
    lv_msg_subsribe_obj(_BV(1), label, NULL);
    lv_obj_add_event_cb( label, [](lv_event_t *e) {
        lv_obj_t *label = (lv_obj_t *)lv_event_get_target(e);
        if (WiFi.isConnected()) {
            lv_label_set_text_fmt(label, "%s", (WiFi.localIP().toString().c_str()));
        } else {
            lv_label_set_text(label, "N.A");
        }
    }, LV_EVENT_MSG_RECEIVED, NULL);

    const char *wifi_rssi = "N.A";
    lv_obj_t *wifi_rssi_label =  create_label(section, NULL, "RSSI", wifi_rssi);

    lv_timer_create([](lv_timer_t *t) {
        lv_obj_t *wifi_rssi_label = (lv_obj_t *)t->user_data;
        if (WiFi.isConnected()) {
            lv_label_set_text_fmt(wifi_rssi_label, "%d", (WiFi.RSSI()));
        }
    }, 3000, wifi_rssi_label);


    lv_obj_t *voltage_label = create_label(section, NULL, "Voltage", "N.A");
    lv_timer_create([](lv_timer_t *t) {
        lv_obj_t *voltage_label = (lv_obj_t *)t->user_data;
        lv_label_set_text_fmt(voltage_label, "%u mV", analogReadMilliVolts(BOARD_BAT_ADC) * 2);
    }, 10000, voltage_label);

    float sd_size = SD.cardSize() / 1024 / 1024 / 1024.0;
    create_label(section, NULL, "SD Card", (SD.cardSize() != 0) ? (String(sd_size) + "GB").c_str() : "N.A");



    String lvgl_version = String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
    create_label(section, NULL, "lvgl", lvgl_version.c_str());

    String arduino_version = String('V') + String(ESP_ARDUINO_VERSION_MAJOR) + "." + String(ESP_ARDUINO_VERSION_MINOR) + "." + String(ESP_ARDUINO_VERSION_PATCH);
    create_label(section, NULL, "Arduino", arduino_version.c_str());

    const char *tft_espi_version = "V2.5.22";
    create_label(section, NULL, "TFT_eSPI", tft_espi_version);

    create_button(section, LV_SYMBOL_POWER, "Sleep", sleep_event_cb);



    /*Create a root page*/
    root_page = lv_menu_page_create(menu, (char *)"T-Deck");
    lv_obj_set_style_pad_hor(root_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    section = lv_menu_section_create(root_page);
    sub_section.push_back(section);

    cont = create_text(section, LV_SYMBOL_SETTINGS, "Radio", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_mechanics_page);
    cont = create_text(section, LV_SYMBOL_AUDIO, "Sound", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_sound_page);
    cont = create_text(section, LV_SYMBOL_EYE_OPEN, "Display", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_display_page);
    cont = create_text(section, LV_SYMBOL_GPS, "GPS", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_gps_page);
    // cont = create_text(section, LV_SYMBOL_FILE, "SD", LV_MENU_ITEM_BUILDER_VARIANT_1);
    // lv_menu_set_load_page_event(menu, cont, sub_sd_page);
    cont = create_text(section, LV_SYMBOL_KEYBOARD, "KB", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_kb_page);
    cont = create_text(section, LV_SYMBOL_SETTINGS, "SETTING", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_setting_page);


    lv_menu_set_sidebar_page(menu, root_page);
    lv_event_send(lv_obj_get_child(lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0), LV_EVENT_CLICKED, NULL);


    std::vector<lv_obj_t *>::iterator it;
    for (it = sub_section.begin(); it != sub_section.end(); it++) {
        lv_obj_set_style_bg_opa(*it, DEFAULT_OPA, LV_PART_MAIN);
    }

    lv_timer_handler();
}

static void back_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_obj_t *menu = (lv_obj_t *)lv_event_get_user_data(e);

    if (lv_menu_back_btn_is_root(menu, obj)) {
        lv_obj_t *mbox1 = lv_msgbox_create(NULL, "Hello", "Root back btn click.", NULL, true);
        lv_obj_center(mbox1);
    }
}

static void switch_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *menu = (lv_obj_t *)lv_event_get_user_data(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
            lv_menu_set_page(menu, NULL);
            lv_menu_set_sidebar_page(menu, root_page);
            lv_event_send(lv_obj_get_child(lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0), LV_EVENT_CLICKED, NULL);
        } else {
            lv_menu_set_sidebar_page(menu, NULL);
            lv_menu_clear_history(menu); /* Clear history because we will be showing the root page later */
            lv_menu_set_page(menu, root_page);
        }
    }
}

static lv_obj_t *create_text(lv_obj_t *parent, const char *icon, const char *txt,
                             lv_menu_builder_variant_t builder_variant)
{
    lv_obj_t *obj = lv_menu_cont_create(parent);

    lv_obj_t *img = NULL;
    lv_obj_t *label = NULL;

    if (icon) {
        img = lv_img_create(obj);
        lv_img_set_src(img, icon);
    }

    if (txt) {
        label = lv_label_create(obj);
        lv_label_set_text(label, txt);
        lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_flex_grow(label, 1);
    }

    if (builder_variant == LV_MENU_ITEM_BUILDER_VARIANT_2 && icon && txt) {
        lv_obj_add_flag(img, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
        lv_obj_swap(img, label);
    }

    return obj;
}

static lv_obj_t *create_slider(lv_obj_t *parent, const char *icon, const char *txt, int32_t min, int32_t max,
                               int32_t val, lv_event_cb_t cb, lv_event_code_t filter)
{
    lv_obj_t *obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_2);

    lv_obj_t *slider = lv_slider_create(obj);
    lv_obj_set_flex_grow(slider, 1);
    lv_slider_set_range(slider, min, max);
    lv_slider_set_value(slider, val, LV_ANIM_OFF);

    if (cb != NULL) {
        lv_obj_add_event_cb(slider, cb, filter, NULL);
    }

    if (icon == NULL) {
        lv_obj_add_flag(slider, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
    }

    return slider;
}

static lv_obj_t *create_switch(lv_obj_t *parent, const char *icon, const char *txt, bool chk, lv_event_cb_t cb)
{
    lv_obj_t *obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_1);

    lv_obj_t *sw = lv_switch_create(obj);
    lv_obj_add_state(sw, chk ? LV_STATE_CHECKED : 0);
    lv_obj_add_event_cb(sw, cb, LV_EVENT_VALUE_CHANGED, NULL);
    return sw;
}

static lv_obj_t *create_button(lv_obj_t *parent, const char *icon, const char *txt, lv_event_cb_t cb)
{
    lv_obj_t *obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_obj_t *btn = lv_btn_create(obj);
    // lv_obj_add_state(btn, chk ? LV_STATE_CHECKED : 0);
    lv_obj_set_size(btn, lv_pct(10), lv_pct(100));
    if (cb) {
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    }
    return obj;
}

static lv_obj_t *create_label(lv_obj_t *parent, const char *icon, const char *txt, const char *default_text)
{
    lv_obj_t *obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_1);
    if (default_text) {
        lv_obj_t *label = lv_label_create(obj);
        lv_label_set_text(label, default_text);
        return label;
    }
    return obj;
}

static lv_obj_t *create_dropdown(lv_obj_t *parent, const char *icon, const char *txt, const char *options, uint8_t default_sel, lv_event_cb_t cb)
{
    lv_obj_t *obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_obj_t *dd = lv_dropdown_create(obj);
    lv_dropdown_set_options(dd, options);
    lv_dropdown_set_selected(dd, default_sel);
    if (cb) {
        lv_obj_add_event_cb(dd, cb, LV_EVENT_VALUE_CHANGED, NULL);
    }
    return dd;
}













