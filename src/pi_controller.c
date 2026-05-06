#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "pi_controller.h"
#include "encoder_queue_struct.h"

#include <math.h>

#define GAIN 0.082
#define STARTING_KP (1/GAIN)

#define INIT_GAIN 1.5F
#define FF_GAIN 80.0F // decrease to lower speed gain



// --------------------------- PID and PWM Init Functions  ------------------------------------------
void pid_controller_init(pid_controller_t *pid, bool is_left){

    // clearing controller mem vars
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    // pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    // clearing out var
    pid->out = 0.0f;

    pid->Kp = INIT_GAIN;
    pid->Ki = 0.01f;
    pid->T = 0.02f;
    pid->limMin = (PWM_MIN - PWM_NEUTRAL) / STARTING_KP;
    pid->limMax = (PWM_MAX - PWM_NEUTRAL) / STARTING_KP;
    pid->is_left = is_left;
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
// ---------------------------------------------------------------------------------------------------

// ----------------- PI controller logic ------------------------------------------------------------
float pid_controller_update(pid_controller_t *pid, float setpoint, float measurement){

    // error signal
    float error = (setpoint - measurement) * (pid->is_left ? 1.0f : -1.0f);    
    
    // proportional
    float proportional = pid->Kp * error;

    // Integral
    if (pid->Ki == 0.0f) {
        pid->integrator = 0.0f;
    }
    else {
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

        // error fix from claude
        if (pid->limMin < proportional){
            limMinInt = pid->limMin - proportional;  // Negative lower bound
        }
        else {
            limMinInt = 0.0f;
        }

        // causing integral windup for now
        // clamp integrator
        if (pid->integrator > limMaxInt){
            pid->integrator = limMaxInt;
        }
        else if (pid->integrator < limMinInt){
            pid->integrator = limMinInt;
        }
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

    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->prevMeasurement = 0;
}



void pi_task(void* arg){

    QueueHandle_t q = (QueueHandle_t) arg;

    pid_controller_t left_wheel;
    pid_controller_t right_wheel;

    pid_controller_init(&left_wheel, true);
    pid_controller_init(&right_wheel, false);

    mcpwm_cmpr_handle_t left_cmp;
    mcpwm_cmpr_handle_t right_cmp;

    pwm_init(&left_cmp, &right_cmp);

    // Note on tuning: The tuning process is iterative — start with Ki = 0 and tune Kp until the response is fast but not oscillating, then slowly increase Ki until steady state error disappears.

    target_speed_packet_t tsp = {0};
    float local_velocity_left = 0, local_velocity_right = 0;
    float left_out = 0, right_out = 0;

    // uint32_t left_width = 0, right_width = 0; 

    // for (;;){
    //     // if new data arrives
    //     if (xQueueReceive(q, &tsp, portMAX_DELAY) == pdTRUE){
            
    //         // left_width = tsp.target_left_rads * 79.2 + PWM_NEUTRAL;
    //         // right_width = tsp.target_right_rads * 79.2 + PWM_NEUTRAL;

    //         // testing:
    //         left_width = 1550;
    //         right_width = 1550;
    //         ESP_LOGE("PWM", "Sending a PWM signal of %0.2f", left_width);

    //         // no packet received in 100ms (added a timeout)
    //         mcpwm_comparator_set_compare_value(left_cmp, left_width);
    //         mcpwm_comparator_set_compare_value(right_cmp, right_width);
    //     }
    // }

    // for (;;){

    //     vTaskDelay(pdMS_TO_TICKS(20));

    //     int diff = 450;
    //     left_width = PWM_NEUTRAL + diff;
    //     right_width = PWM_NEUTRAL - diff;
    //     // ESP_LOGE("PWM", "Sending a PWM signal of %0.2f", left_width);

    //     // no packet received in 100ms (added a timeout)
    //     mcpwm_comparator_set_compare_value(left_cmp, left_width);
    //     mcpwm_comparator_set_compare_value(right_cmp, right_width);

    //     if (xSemaphoreTake(vel_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    //         ESP_LOGE("PWM", "left_vel=%.2f right_vel=%.2f", shared_velocity_left, shared_velocity_right);
    //         xSemaphoreGive(vel_mutex);
    //     }

    // }

    // forever loop
    for (;;){

        // if new data arrives
        if (xQueueReceive(q, &tsp, pdMS_TO_TICKS(100)) != pdTRUE){
            
            // no packet received in 100ms (added a timeout)
            mcpwm_comparator_set_compare_value(left_cmp, PWM_NEUTRAL);
            mcpwm_comparator_set_compare_value(right_cmp, PWM_NEUTRAL);
            pi_reset(&left_wheel);
            pi_reset(&right_wheel);
            continue;
        }

        if (xSemaphoreTake(vel_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

            local_velocity_left = shared_velocity_left;
            local_velocity_right = shared_velocity_right;
            xSemaphoreGive(vel_mutex);
        }

        left_out = pid_controller_update(&left_wheel, tsp.target_left_rads, local_velocity_left);
        right_out = pid_controller_update(&right_wheel, tsp.target_right_rads, local_velocity_right);
        right_out *= -1;
        
        ESP_LOGE("PWM", "left_out=%0.2f, right_out=%0.2f", left_out, right_out);

        // rough feedforward: map setpoint directly to PWM
        float ff_left  = tsp.target_left_rads  * FF_GAIN;
        float ff_right = tsp.target_right_rads * FF_GAIN;

        float left_pwm  = PWM_NEUTRAL + ff_left  + (left_out  * STARTING_KP);
        float right_pwm = PWM_NEUTRAL + ff_right + (right_out * STARTING_KP);

        left_pwm = fmaxf(PWM_MIN, fminf(PWM_MAX, left_pwm));
        right_pwm = fmaxf(PWM_MIN, fminf(PWM_MAX, right_pwm));
        
        // applying PI output to the comparator
        mcpwm_comparator_set_compare_value(left_cmp, (uint32_t) left_pwm);
        mcpwm_comparator_set_compare_value(right_cmp, (uint32_t) right_pwm);

        // ESP_LOGE("PI", "setpoint_left=%.2f setpoint_right=%.2f meas_left=%.2f meas_right=%.2f out_left=%.2f pwm_left=%lu",
        //     tsp.target_left_rads, tsp.target_right_rads, local_velocity_left, local_velocity_right,left_out, left_pwm);

        // TODO: figure out the motor driver interface
        // TODO: need to add the feedforward term
        // TODO: Apply the result to the PWM for each motor
    }

}