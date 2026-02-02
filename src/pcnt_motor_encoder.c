#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "pcnt_motor_encoder.h"
#include "encoder_queue_struct.h"
#include "esp_timer.h"


// GPIO I decided to use (all on the right side and can change later)
#define ENCODER_LEFT_A 13
#define ENCODER_LEFT_B 12
#define ENCODER_RIGHT_A 27
#define ENCODER_RIGHT_B 33

#define HIGH_LIMIT 1024
#define LOW_LIMIT -1024

static const char *TAG = "PCNT_LOG:";

// global declarations for the left and the right encoder (made it static so that the other files don't access it)
static pcnt_unit_handle_t pcnt_unit_left = NULL;
static pcnt_unit_handle_t pcnt_unit_right = NULL;


// Need to initailize, enable, and start the PCNT unit
// PCNT init funciton (this one is using the LEFT GPIOs)
void encoder_pcnt_init(void){

    // PCNT unit configuration (left)
    ESP_LOGI(TAG, "install pcnt unit left");
    pcnt_unit_config_t unit_config = {
        .high_limit = HIGH_LIMIT,
        .low_limit = LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_left));

    // PCNT unit configuration (right)
    ESP_LOGI(TAG, "install pcnt unit right");
    pcnt_unit_config_t unit_config = {
        .high_limit = HIGH_LIMIT,
        .low_limit = LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_right));


    // Glitch filter for left and right encoder (used to filter out noise)
    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config_left = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_left, &filter_config_left));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config_right = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_right, &filter_config_right));


    // Channel A and B configuration For Left Enconder
    ESP_LOGI(TAG, "install pcnt channels (Left)");
    pcnt_chan_config_t chan_a_left_config = {
        .edge_gpio_num = ENCODER_LEFT_A,
        .level_gpio_num = ENCODER_LEFT_B,
    };
    pcnt_channel_handle_t pcnt_chan_left_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_left, &chan_a_left_config, &pcnt_chan_left_a));

    pcnt_chan_config_t chan_b_left_config = {
        .edge_gpio_num = ENCODER_LEFT_B,
        .level_gpio_num = ENCODER_LEFT_A,
    };
    pcnt_channel_handle_t pcnt_chan_left_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_left, &chan_b_left_config, &pcnt_chan_left_b));

    // uncommment when it is time to check right encoder
    // Channel A and B configuration For Right Enconder
    ESP_LOGI(TAG, "install pcnt channels (Right)");
    pcnt_chan_config_t chan_a_right_config = {
        .edge_gpio_num = ENCODER_RIGHT_A,
        .level_gpio_num = ENCODER_RIGHT_B,
    };
    pcnt_channel_handle_t pcnt_chan_right_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_right, &chan_a_right_config, &pcnt_chan_right_a));

    pcnt_chan_config_t chan_b_right_config = {
        .edge_gpio_num = ENCODER_RIGHT_B,
        .level_gpio_num = ENCODER_RIGHT_A,
    };
    pcnt_channel_handle_t pcnt_chan_right_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_right, &chan_b_right_config, &pcnt_chan_right_b));

    // Edge level
    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_left_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_left_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_left_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_left_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_right_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_right_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_right_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_right_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // enable, clear, and start PCNT unit (left)
    ESP_LOGI(TAG, "enable pcnt unit left");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_left));
    ESP_LOGI(TAG, "clear pcnt unit left");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_left));
    ESP_LOGI(TAG, "start pcnt unit left");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_left));

    // enable, clear, and start PCNT unit (right)
    ESP_LOGI(TAG, "enable pcnt unit right");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_right));
    ESP_LOGI(TAG, "clear pcnt unit right");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_right));
    ESP_LOGI(TAG, "start pcnt unit right");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_right));
}

// Note - Need to see if two types of values can be returned from pcnt_unit_get_count()

// this function is going to do the computing of the PCNT, and we can create another file for sending the data to the robot
// ---- Task: read count every 500 ms ----
void pcnt_read_task(void *arg) {

    QueueHandle_t q = (QueueHandle_t) arg;
    encoder_data compute_data;

    int left_last = 0;
    int right_last = 0;

    while (true) {

        int left_count = 0;
        int right_count = 0;

        esp_err_t left_err = pcnt_unit_get_count(pcnt_unit_left, &left_count);
        esp_err_t right_err = pcnt_unit_get_count(pcnt_unit_right, &right_count);

        if (left_err == ESP_OK && right_err == ESP_OK) {
            int left_delta = left_count - left_last;
            left_last = left_count;

            int right_delta = right_count - right_last;
            right_last = right_count;

            // not sure how we want to compute it, but I just have it set to the left_tick
            compute_data.left_tick = left_count;
            compute_data.right_tick = right_count;
            compute_data.timestamp_ms = esp_timer_get_time();

            // sending the data to the queue
            xQueueSend(q, &compute_data, 0);

            // If you want a "rate", delta per 0.5s -> ticks/sec = delta * 2
            ESP_LOGI(TAG, "left count=%d  delta(500ms)=%d  approx ticks/s=%d",
                     left_count, left_delta, left_delta * 2);
        } else {
            ESP_LOGE(TAG, "pcnt_unit_get_count failed: %s", esp_err_to_name(left_err));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

