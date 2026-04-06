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


        // TODO: figure out the motor driver interface
        // TODO: need to add the feedforward term
        // TODO: Apply the result to the PWM for each motor

    }

}