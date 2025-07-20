/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <Arduino.h>
#include "esp_timer.h"
#undef ESP_UTILS_LOG_TAG
#define ESP_UTILS_LOG_TAG "LvPort"
#include "esp_lib_utils.h"
#include <esp_display_panel.hpp>
#include <lvgl.h>

/**
/* To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 * You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 */
#include <demos/lv_demos.h>
// #include <examples/lv_examples.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

#define USE_LVGL_TASK

#define TFT_HOR_RES     480 // ESP_PANEL_BOARD_WIDTH
#define TFT_VER_RES     272 // ESP_PANEL_BOARD_HEIGHT

#define DISPLAY_RENDER_MODE_PARTIAL     1
#define DISPLAY_RENDER_MODE_PARTIAL_V2  2
#define DISPLAY_RENDER_MODE_DIRECT      3

#define DISPLAY_MODE    DISPLAY_RENDER_MODE_PARTIAL

#if DISPLAY_MODE == DISPLAY_RENDER_MODE_PARTIAL
    // LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes
    #define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
    static uint32_t draw_buf[DRAW_BUF_SIZE / 4];
#elif DISPLAY_MODE == DISPLAY_RENDER_MODE_PARTIAL_V2
    #define LVGL_PORT_BUFFER_MALLOC_CAPS            (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)       // Allocate LVGL buffer in SRAM
    #define LVGL_PORT_BUFFER_SIZE_HEIGHT            (20)
    #define LVGL_PORT_BUFFER_NUM                    (2)
    #define LVGL_PORT_BUFFER_NUM_MAX                (2)
    static void *lvgl_buf[LVGL_PORT_BUFFER_NUM_MAX] = {};
#elif DISPLAY_MODE == DISPLAY_RENDER_MODE_DIRECT
    #define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES * (LV_COLOR_DEPTH / 8))
#else
    #error "Invalid display type"
#endif

#ifdef USE_LVGL_TASK
static SemaphoreHandle_t lvgl_mux = nullptr;                  // LVGL mutex
static TaskHandle_t lvgl_task_handle = nullptr;
static esp_timer_handle_t lvgl_tick_timer = NULL;
#endif


#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
    LV_UNUSED(level);
    Serial.println(buf);
    Serial.flush();
}
#endif

void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
    LCD *lcd = (LCD *)lv_display_get_user_data(disp);
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

    lcd->drawBitmap(offsetx1, offsety1, offsetx2 - offsetx1 + 1, offsety2 - offsety1 + 1, (const uint8_t *)px_map);
    // For RGB LCD, directly notify LVGL that the buffer is ready
    if (lcd->getBus()->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        lv_disp_flush_ready(disp);
    }
}

#ifdef USE_LVGL_TASK
bool lvgl_port_lock(int timeout_ms)
{
    ESP_UTILS_CHECK_NULL_RETURN(lvgl_mux, false, "LVGL mutex is not initialized");

    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE);
}

bool lvgl_port_unlock(void)
{
    ESP_UTILS_CHECK_NULL_RETURN(lvgl_mux, false, "LVGL mutex is not initialized");

    xSemaphoreGiveRecursive(lvgl_mux);

    return true;
}

#define LVGL_PORT_TASK_MAX_DELAY_MS     (500)
#define LVGL_PORT_TASK_MIN_DELAY_MS     (2)
#define LVGL_PORT_TICK_PERIOD_MS        (2)

static void tick_increment(void *arg)
{
    /* Tell LVGL how many milliseconds have elapsed */
    lv_tick_inc(LVGL_PORT_TICK_PERIOD_MS);
}

static void tick_init(void)
{
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &tick_increment,
        .name = "LVGL tick"
    };
    assert(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer) == ESP_OK);
    assert(esp_timer_start_periodic(lvgl_tick_timer, LVGL_PORT_TICK_PERIOD_MS * 1000) == ESP_OK);
}

static void lvgl_task(void *arg)
{
    ESP_UTILS_LOGI("LVGL", "Starting LVGL task");

    uint32_t delay_ms = LVGL_PORT_TASK_MAX_DELAY_MS;
    while (1) {
        if (lvgl_port_lock(-1)) {
            delay_ms = lv_timer_handler();
            lvgl_port_unlock();
        }

        // Clamp delay to safe bounds
        if (delay_ms == LV_NO_TIMER_READY || delay_ms > LVGL_PORT_TASK_MAX_DELAY_MS) {
            delay_ms = LVGL_PORT_TASK_MAX_DELAY_MS;
        } else if (delay_ms < LVGL_PORT_TASK_MIN_DELAY_MS) {
            delay_ms = LVGL_PORT_TASK_MIN_DELAY_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
#else
// Use Arduinos millis() as tick source
static uint32_t display_tick(void)
{
    return millis();
}
#endif

static void touch_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    Touch *tp = (Touch *)lv_indev_get_user_data(indev);
    TouchPoint point;

    /* Read data from touch controller */
    int read_touch_result = tp->readPoints(&point, 1, 0);
    if (read_touch_result > 0) {
        data->point.x = point.x;
        data->point.y = point.y;
        data->state = LV_INDEV_STATE_PRESSED;

        ESP_UTILS_LOGI("Touch %d, %d", point.x, point.y);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void init_display()
{
    Board *board = new Board();
    board->init();
    assert(board->begin());

    LCD *lcd = board->getLCD();
    Touch *tp = board->getTouch();

    assert(lcd != nullptr);

    auto bus_type = lcd->getBus()->getBasicAttributes().type;
    assert(bus_type == ESP_PANEL_BUS_TYPE_RGB);

    // Initialise LVGL
    lv_init();

#ifdef USE_LVGL_TASK
    tick_init();
#endif

    ESP_UTILS_LOGI("Initializing LVGL display driver");

#if DISPLAY_MODE == DISPLAY_RENDER_MODE_PARTIAL
    lv_display_t *disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
    assert(disp != nullptr);
    lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
#elif DISPLAY_MODE == DISPLAY_RENDER_MODE_PARTIAL_V2
    int buffer_size = TFT_HOR_RES * LVGL_PORT_BUFFER_SIZE_HEIGHT * (LV_COLOR_DEPTH / 8);
    for (int i = 0; (i < LVGL_PORT_BUFFER_NUM) && (i < LVGL_PORT_BUFFER_NUM_MAX); i++) {
        lvgl_buf[i] = heap_caps_malloc(buffer_size, LVGL_PORT_BUFFER_MALLOC_CAPS);
        assert(lvgl_buf[i]);
        ESP_UTILS_LOGI("Buffer[%d] address: %p, size: %d", i, lvgl_buf[i], buffer_size);
    }
    lv_display_t *disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
    assert(disp != nullptr);
    // Set draw buffers
    lv_display_set_buffers(disp,
        lvgl_buf[0],
        lvgl_buf[1], // Use NULL if single buffering
        buffer_size,
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );
#elif DISPLAY_MODE == DISPLAY_RENDER_MODE_DIRECT
    lv_display_t *disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
    assert(disp != nullptr);
    void* draw_buf = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_SPIRAM);
    lv_display_set_buffers(disp, draw_buf, NULL, DRAW_BUF_SIZE, LV_DISPLAY_RENDER_MODE_DIRECT);
#endif

    lv_display_set_flush_cb(disp, flush_cb);
    lv_display_set_user_data(disp, lcd);

    if (tp != nullptr) {
        ESP_UTILS_LOGI("Initialize LVGL input driver");
        lv_indev_t *indev = lv_indev_create();
        assert(indev != nullptr);

        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, touch_read);
        lv_indev_set_user_data(indev, tp);
    }

#ifdef USE_LVGL_TASK
    ESP_UTILS_LOGI("Create mutex for LVGL");
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux != nullptr);

    #define LVGL_PORT_TASK_CORE         (1)
    #define LVGL_PORT_TASK_STACK_SIZE   (8 * 1024)
    #define LVGL_PORT_TASK_PRIORITY     (2)

    ESP_UTILS_LOGI("Create LVGL task");
    BaseType_t core_id = (LVGL_PORT_TASK_CORE < 0) ? tskNO_AFFINITY : LVGL_PORT_TASK_CORE;
    BaseType_t ret = xTaskCreatePinnedToCore(lvgl_task, "lvgl", LVGL_PORT_TASK_STACK_SIZE, NULL,
                     LVGL_PORT_TASK_PRIORITY, NULL, core_id);
    assert(ret == pdPASS);

#else
    // Set a tick source so that LVGL will know how much time elapsed
    lv_tick_set_cb(display_tick);
#endif
}

static void init_gui()
{
#if 0
    /**
     * Create the simple labels
     */
    lv_obj_t *label_1 = lv_label_create(lv_scr_act());
    lv_label_set_text(label_1, "Hello World!");
    lv_obj_set_style_text_font(label_1, &lv_font_montserrat_30, 0);
    lv_obj_align(label_1, LV_ALIGN_CENTER, 0, -20);
    lv_obj_t *label_2 = lv_label_create(lv_scr_act());
    lv_label_set_text_fmt(
        label_2, "ESP32_Display_Panel(%d.%d.%d)",
        ESP_PANEL_VERSION_MAJOR, ESP_PANEL_VERSION_MINOR, ESP_PANEL_VERSION_PATCH
    );
    lv_obj_set_style_text_font(label_2, &lv_font_montserrat_16, 0);
    lv_obj_align_to(label_2, label_1, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_t *label_3 = lv_label_create(lv_scr_act());
    lv_label_set_text_fmt(label_3, "LVGL(%d.%d.%d)", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
    lv_obj_set_style_text_font(label_3, &lv_font_montserrat_16, 0);
    lv_obj_align_to(label_3, label_2, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

#else
    /**
     * Try an example. Don't forget to uncomment header.
     * See all the examples online: https://docs.lvgl.io/master/examples.html
     * source codes: https://github.com/lvgl/lvgl/tree/e7f88efa5853128bf871dde335c0ca8da9eb7731/examples
     */
    //  lv_example_btn_1();

    /**
     * Or try out a demo.
     * Don't forget to uncomment header and enable the demos in `lv_conf.h`. E.g. `LV_USE_DEMO_WIDGETS`
     */
    // lv_demo_widgets();
    lv_demo_benchmark();
    // lv_demo_music();
    // lv_demo_stress();
#endif
}

void setup()
{
    char lvgl_version[20];

    // LGVL version string
    snprintf(lvgl_version, sizeof(lvgl_version), 
        "LVGL demo v%d.%d.%d", lv_version_major() + lv_version_minor() + lv_version_patch());

    // Initialize serial
    Serial.begin(115200);
    Serial.println(lvgl_version);

    // Initialize display
    ESP_UTILS_LOGI("Initialize display");
    init_display();

    // Initialize GUI
    ESP_UTILS_LOGI("Creating UI");

    #ifdef USE_LVGL_TASK
    lvgl_port_lock(-1);
#endif
    init_gui();
#ifdef USE_LVGL_TASK
    /* Release the mutex */
    lvgl_port_unlock();
#endif
}

void loop()
{
#ifdef USE_LVGL_TASK
    Serial.println("IDLE loop");
    delay(1000);
#else
    // Update the UI
    lv_timer_handler();
    delay(5);
#endif
}
