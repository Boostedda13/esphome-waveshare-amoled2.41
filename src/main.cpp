#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>

static const uint16_t screenWidth = 450;
static const uint16_t screenHeight = 600;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[screenWidth * 10];
static lv_disp_drv_t disp_drv;

// QSPI bus: CS=9, CLK=10, D0=11, D1=12, D2=13, D3=14
Arduino_DataBus *bus = new Arduino_ESP32QSPI(9 /* CS */, 10 /* CLK */, 11 /* D0 */, 12 /* D1 */, 13 /* D2 */, 14 /* D3 */);
Arduino_GFX *gfx = new Arduino_RM690B0(bus, 21 /* RST */, 0 /* rotation */, screenWidth, screenHeight, 16 /* col offset */);

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    const uint8_t addr = 0x38; // FT63x6 I2C address
    Wire.beginTransmission(addr);
    Wire.write(0x02);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(addr, (uint8_t)5) != 5) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }
    uint8_t buf[5];
    for (int i = 0; i < 5; i++) {
        buf[i] = Wire.read();
    }
    if (buf[0] & 0x0F) {
        uint16_t x = ((buf[1] & 0x0F) << 8) | buf[2];
        uint16_t y = ((buf[3] & 0x0F) << 8) | buf[4];
        data->state = LV_INDEV_STATE_PR;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(47, 48); // I2C bus for touch controller

    gfx->begin();
    gfx->fillScreen(BLACK);

    lv_init();

    lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, screenWidth * 10);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    lv_obj_t *slider = lv_slider_create(lv_scr_act());
    lv_obj_align(slider, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_width(slider, 300);
    lv_obj_add_event_cb(slider, [](lv_event_t *e) {
        lv_obj_t *obj = static_cast<lv_obj_t *>(lv_event_get_target(e));
        uint16_t value = lv_slider_get_value(obj);
        Serial.printf("Brightness: %u\n", value);
    }, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *hue_slider = lv_slider_create(lv_scr_act());
    lv_obj_align(hue_slider, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_set_width(hue_slider, 300);
    lv_slider_set_range(hue_slider, 0, 360);
    lv_obj_add_event_cb(hue_slider, [](lv_event_t *e) {
        lv_obj_t *obj = static_cast<lv_obj_t *>(lv_event_get_target(e));
        uint16_t value = lv_slider_get_value(obj);
        Serial.printf("Hue: %u\n", value);
    }, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *sat_slider = lv_slider_create(lv_scr_act());
    lv_obj_align(sat_slider, LV_ALIGN_TOP_MID, 0, 140);
    lv_obj_set_width(sat_slider, 300);
    lv_slider_set_range(sat_slider, 0, 100);
    lv_obj_add_event_cb(sat_slider, [](lv_event_t *e) {
        lv_obj_t *obj = static_cast<lv_obj_t *>(lv_event_get_target(e));
        uint16_t value = lv_slider_get_value(obj);
        Serial.printf("Saturation: %u\n", value);
    }, LV_EVENT_VALUE_CHANGED, nullptr);
}

void loop() {
    lv_timer_handler();
    delay(5);
}

