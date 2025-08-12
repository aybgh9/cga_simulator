#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#define VSYNC_GPIO  2
#define HSYNC_GPIO  4
#define RED_GPIO    5

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 200

#define TIMER_DIVIDER         80  // 1 us per tick (assuming 80 MHz APB clock)

// --- CGA-like horizontal timing (approximate) ---
// Total line period ~63.5 us @ ~15.734 kHz
#define H_TOTAL_US        64u
#define H_HSYNC_US        5u
#define H_PORCH_F_US      10u
#define H_ACTIVE_US       20u
#define H_PORCH_B_US      100u // extra long to make sure we trigger H timer

// --- CGA-like vertical timing (approximate) ---
#define FRAME_INTERVAL_US     16667u  // ~60 Hz frame timer
#define V_VSYNC_LINES         4
#define V_ACTIVE_LINES_END         V_VSYNC_LINES+SCREEN_HEIGHT
static const char *TAG = "CGA";

typedef enum {
    STATE_HSYNC_ON,
    STATE_HSYNC_OFF,
    STATE_RED_ON,
    STATE_RED_OFF
} cga_state_t;

static volatile cga_state_t state = STATE_HSYNC_ON;

static volatile uint32_t current_line = 0;  // [0, V_TOTAL_LINES)

// Helpers
static inline bool is_active_line(uint32_t line)
{
    if (line < V_VSYNC_LINES) {
        gpio_set_level(VSYNC_GPIO, 1);
        return false;
    }
    else if (line > V_ACTIVE_LINES_END) {
        gpio_set_level(VSYNC_GPIO, 0);
        return 0;
    }
    else {
        gpio_set_level(VSYNC_GPIO, 0);
        return 1;};
}

static inline void schedule_next_from_now_us(uint32_t next_delay_us)
{
    // Clear and schedule next alarm relative to current counter in ISR context
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    uint64_t counter_val = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, counter_val + next_delay_us);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
}

// Schedule next event relative to HSYNC reference time
static inline void schedule_next_from_hsync_ref_us(uint32_t delay_from_hsync_us)
{
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    uint64_t counter_val = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, delay_from_hsync_us+counter_val);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
}

// Apply GPIO outputs for a given high-level state
static inline void IRAM_ATTR apply_outputs_for_state(cga_state_t st, uint32_t line)
{
    // HSYNC
    if (st == STATE_HSYNC_ON) {
        gpio_set_level(HSYNC_GPIO, 1);
    } else {
        gpio_set_level(HSYNC_GPIO, 0);
    }

    // RED
    if (st == STATE_RED_ON) {
        gpio_set_level(RED_GPIO, 1);
    } else  {
        gpio_set_level(RED_GPIO, 0);
    }
}

// Timer initialization helpers
static inline void init_h_line_timer(void)
{
    // Reset and start horizontal line timer
    timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, H_TOTAL_US);
    timer_start(TIMER_GROUP_1, TIMER_1);
}

static inline void init_line_timer(void)
{
    // Reset and start line event timer
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, H_HSYNC_US);
    timer_start(TIMER_GROUP_0, TIMER_0);
}



static void IRAM_ATTR frame_line_timer_isr(void *arg)
{
    // Clear and re-enable Group1 Timer0 alarm (auto-reload)
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_0);

    // Reset frame counters/state
    current_line = 0;
    state = STATE_HSYNC_ON;
    gpio_set_level(VSYNC_GPIO, 1);
  
    // Ensure outputs are in a known state between frames (optional)
    apply_outputs_for_state(state, current_line);
    
    // Initialize horizontal line timer for new frame
    init_h_line_timer();
    
    // Reset watchdog timer to prevent system reset
    esp_task_wdt_reset();
}

static void IRAM_ATTR H_line_timer_isr(void *arg)
{
    // Clear and re-enable Group1 Timer1 alarm (auto-reload)
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_1);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_1);
    
    // Initialize line timer for this line
    // Move to next line
    current_line++;
    is_active_line(current_line);
    state = STATE_HSYNC_ON;
    apply_outputs_for_state(state, current_line);

    init_line_timer();

}



static void IRAM_ATTR line_timer_isr(void *arg)
{

    switch (state) {

        case STATE_HSYNC_ON: {
            state = STATE_HSYNC_OFF;
            apply_outputs_for_state(state, current_line);
            schedule_next_from_hsync_ref_us(H_PORCH_F_US);
            return; // Exit ISR, next event already scheduled
        }
        case STATE_HSYNC_OFF: {
            
            if (is_active_line(current_line)) {
                state = STATE_RED_ON;
                apply_outputs_for_state(state, current_line);
                schedule_next_from_hsync_ref_us(H_ACTIVE_US);
                return; 
            } else {}
            [[fallthrough]];
        }
        case STATE_RED_ON: {
            state = STATE_RED_OFF;
            apply_outputs_for_state(state, current_line);
            schedule_next_from_hsync_ref_us(H_PORCH_F_US);
            return;
        }
        case STATE_RED_OFF: {
        }
    }
}

void app_main(void) {
    // Configure GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << VSYNC_GPIO) | (1ULL << HSYNC_GPIO) | (1ULL << RED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Configure line event timer (Group0/T0)
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = false
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, line_timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

    // Configure frame timer at ~60Hz (Group1/T0) with auto-reload
    timer_config_t frame_cfg = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true
    };
    timer_init(TIMER_GROUP_1, TIMER_0, &frame_cfg);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, FRAME_INTERVAL_US);
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);
    timer_isr_register(TIMER_GROUP_1, TIMER_0, frame_line_timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_1, TIMER_0);

    // Configure horizontal line timer at ~15khz (Group1/T1) with auto-reload
    timer_config_t h_timer_cfg = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true
    };
    timer_init(TIMER_GROUP_1, TIMER_1, &h_timer_cfg);
    timer_enable_intr(TIMER_GROUP_1, TIMER_1);
    timer_isr_register(TIMER_GROUP_1, TIMER_1, H_line_timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);


    ESP_LOGI(TAG, "CGA signal simulation started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
