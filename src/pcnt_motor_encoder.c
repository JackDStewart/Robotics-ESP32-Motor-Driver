#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "pcnt_motor_encoder.h"

// GPIO I decided to use (all on the right side and can change later)
#define ENCODER_LEFT_A 13
#define ENCODER_LEFT_B 12
// #define ENCODER_RIGHT_A 27
// #define ENCODER_RIGHT_B 33
#define HIGH_LIMIT  1024
#define LOW_LIMIT   -1024

static const char *TAG = "PCNT_LOG:";

/*
Notes:
I found the headers that we need to use for PCNT

*/

/*
TODO: 
1. Need to figure out which pins on the ESP32 we want to use for the encoder (done)
2. Need to implement PCNT
    a. init
    b. start
*/
pcnt_unit_handle_t pcnt_unit = NULL;

// Need to initailize, enable, and start the PCNT unit
// PCNT init funciton (this one is using the LEFT GPIOs)
void encoder_pcnt_init(void){
    // PCNT unit configuration
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = HIGH_LIMIT,
        .low_limit = LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // Glitch filter (used to filter out noise)
    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // Channel A and B configuration
    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_LEFT_A,
        .level_gpio_num = ENCODER_LEFT_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_LEFT_B,
        .level_gpio_num = ENCODER_LEFT_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    // Edge level
    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // enable, clear, and start PCNT unit
    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

// ---- Task: read count every 500 ms ----
void pcnt_read_task(void *arg) {
    (void)arg;

    int last = 0;

    while (true) {
        int count = 0;
        esp_err_t err = pcnt_unit_get_count(pcnt_unit, &count);
        if (err == ESP_OK) {
            int delta = count - last;
            last = count;

            // If you want a "rate", delta per 0.5s -> ticks/sec = delta * 2
            ESP_LOGI(TAG, "left count=%d  delta(500ms)=%d  approx ticks/s=%d",
                     count, delta, delta * 2);
        } else {
            ESP_LOGE(TAG, "pcnt_unit_get_count failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// made it static so that it is only in this file and inline because it is accessing registers (units)
// static inline void pcnt_start(int unit){

//     // enabling the init and clock for PCNT
//     periph_module_enable(PERIPH_PCNT_MODULE);
//     PCNT.ctrl.clk_en = 1;

//     // want to pause, reset, and then resume (using the first two registers (units) because we only have two motors )
//     if (unit == 0){

//         PCNT.ctrl.cnt_pause_u0 = 1;
//         PCNT.ctrl.cnt_rst_u0 = 1;
//         PCNT.ctrl.cnt_rst_u0 = 0;
//         PCNT.ctrl.cnt_pause_u0 = 0;
//     }
//     else if (unit == 1){

//         PCNT.ctrl.cnt_pause_u1 = 1;
//         PCNT.ctrl.cnt_rst_u1 = 1;
//         PCNT.ctrl.cnt_rst_u1 = 0;
//         PCNT.ctrl.cnt_pause_u1 = 0;
//     }
// }
