#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "pi_controller.h"
#include "encoder_queue_struct.h"



void pid_controller_init(pid_controller_t *pid){

    // clearing controller mem vars
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    // pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    // clearing out var
    pid->out = 0.0f;
}

void pwm_init(mcpwm_cmpr_handle_t *left_cmp, mcpwm_cmpr_handle_t *right_cmp) {

    //timer config; 1 tick == 1us
    mcpwm_timer_config_t tim_config = {};
    tim_config.group_id = 0;
    tim_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    tim_config.resolution_hz = 1000000;  // 1MHz = 1us resolution
    tim_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    tim_config.period_ticks = 20000;     // 20000us = 20ms = 50Hz
    tim_config.intr_priority = 0;

    // create timer
    mcpwm_timer_handle_t timer;
    ESP_ERROR_CHECK(mcpwm_new_timer(&tim_config, &timer));


    //operator config
    mcpwm_operator_config_t op_config = {};
    op_config.group_id = 0; // same group as timer

    // left and right operators
    mcpwm_oper_handle_t left_oper, right_oper;
    ESP_ERROR_CHECK(mcpwm_new_operator(&op_config, &left_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(left_oper, timer));

    ESP_ERROR_CHECK(mcpwm_new_operator(&op_config, &right_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(right_oper, timer));


    //left and right comparators
    mcpwm_comparator_config_t cmp_config = {};
    cmp_config.flags.update_cmp_on_tez = true; // update on timer == zero

    ESP_ERROR_CHECK(mcpwm_new_comparator(left_oper, &cmp_config, left_cmp));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*left_cmp, PWM_NEUTRAL)); 

    ESP_ERROR_CHECK(mcpwm_new_comparator(right_oper, &cmp_config, right_cmp));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*right_cmp, PWM_NEUTRAL));


    //left and right generators
    mcpwm_generator_config_t gen_config = {};
    gen_config.gen_gpio_num = LEFT_MOTOR_GPIO;

    //left gen
    mcpwm_gen_handle_t left_gen;
    ESP_ERROR_CHECK(mcpwm_new_generator(left_oper, &gen_config, &left_gen));

    // high on timer zero
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(left_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH)));

    // low on comparator match
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(left_gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            *left_cmp,
            MCPWM_GEN_ACTION_LOW)));

    //right gen
    gen_config.gen_gpio_num = RIGHT_MOTOR_GPIO;
    mcpwm_gen_handle_t right_gen;
    ESP_ERROR_CHECK(mcpwm_new_generator(right_oper, &gen_config, &right_gen));

    // high on timer zero
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(right_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH)));

    // low on comparator match
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(right_gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            *right_cmp,
            MCPWM_GEN_ACTION_LOW)));

    // start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

float pid_controller_update(pid_controller_t *pid, float setpoint, float measurement){

    // error signal
    float error = setpoint - measurement;
    
    // proportional
    float proportional = pid->Kp * error;

    // Integral
    pid->integrator = pid->integrator + (0.5f * (pid->Ki * pid->T * (error + pid->prevError)));

    // Anti-windup via dynamic clamping
    float limMinInt, limMaxInt;

    // compute integrator limits (figure out the limits)
    if (pid->limMax > proportional){

        limMaxInt = pid->limMax - proportional;
    }
    else {
        limMaxInt = 0.0f;
    }

    if (pid->limMin < proportional){
        limMinInt = proportional - pid->limMin;
    }
    else {
        limMinInt = 0.0f;
    }


    // clamp integrator
    if (pid->integrator > limMaxInt){
        pid->integrator = limMaxInt;
    }
    else if (pid->integrator < limMinInt){
        pid->integrator = limMinInt;
    }


    // derivative (band-limited differentiator)
    // pid->differentiator = (2.0f * pid->Kd * (measurement - pid->prevMeasurement)      // Note: derivative on measurement
    //                      + (2.0f * pid->tau - pid->T) * pid->differentiator)
    //                      / (2.0f * pid->tau + pid->T);


    // compute and apply limits
    // pid->out = proportional + pid->integrator + pid->differentiator;
    pid->out = proportional + pid->integrator;


    if (pid->out > pid->limMax){
        pid->out = pid->limMax;
    }
    else if (pid->out < pid->limMin){
        pid->out = pid->limMin;
    }

    // store error and measurement for later use
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    // return controller output
    return pid->out;
}

// need a function for pi_reset as well as pi_task

void pi_reset(pid_controller_t *pid){

    pid->integrator = 0;
    pid->prevError = 0;
    pid->prevMeasurement = 0;
}



void pi_task(void* arg){

    QueueHandle_t q = (QueueHandle_t) arg;

    pid_controller_t left_wheel;
    pid_controller_t right_wheel;

    pid_controller_init(&left_wheel);
    pid_controller_init(&right_wheel);

    mcpwm_cmpr_handle_t left_cmp;
    mcpwm_cmpr_handle_t right_cmp;

    pwm_init(&left_cmp, &right_cmp);


    // Set your Kp, Ki, T, limMin, limMax values on both structs (need to tune these by testing)
    // left wheel
    left_wheel.Kp = 1.0f;
    left_wheel.Ki = 0.1f;
    left_wheel.T = 0.02f;
    // need to set the limMin and limMax once we know what the PWM output range is

    // right wheel
    right_wheel.Kp = 1.0f;
    right_wheel.Ki = 0.1f;
    right_wheel.T = 0.02f;
    // need to set the limMin and limMax once we know what the PWM output range is


    // Note on tuning: The tuning process is iterative — start with Ki = 0 and tune Kp until the response is fast but not oscillating, then slowly increase Ki until steady state error disappears.

    target_speed_packet_t tsp;
    float local_velocity_left = 0, local_velocity_right = 0;
    float left_out = 0, right_out = 0;

    // forever loop
    for (;;){

        // left and right have the same T (20ms)
        vTaskDelay(pdMS_TO_TICKS(20));

        // if new data arrives
        xQueueReceive(q, &tsp, 0);

        if (xSemaphoreTake(vel_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

            local_velocity_left = shared_velocity_left;
            local_velocity_right = shared_velocity_right;
            xSemaphoreGive(vel_mutex);
        }

        left_out = pid_controller_update(&left_wheel, tsp.target_left_rads, local_velocity_left);
        right_out = pid_controller_update(&right_wheel, tsp.target_right_rads, local_velocity_right);

        // will unvoid them once we complete the TODOs below
        (void)left_out;
        (void)right_out;

        // TODO: figure out the motor driver interface
        // TODO: need to add the feedforward term
        // TODO: Apply the result to the PWM for each motor
    }

}