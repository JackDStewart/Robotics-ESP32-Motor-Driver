#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include <stdbool.h>

#define PWM_NEUTRAL 1500
#define PWM_MAX 1950
#define PWM_MIN 1050

#define LEFT_MOTOR_GPIO 25 //pin A1
#define RIGHT_MOTOR_GPIO 26 //pin A0

extern float shared_velocity_left;
extern float shared_velocity_right;

typedef struct {
    bool is_left;
    // controller gains (proportional, integral, and derivative)
    float Kp;
    float Ki;
    float Kd;

    // derivative low-pass filter time constant
    float tau;
    
    // output limits
    float limMax;
    float limMin;

    // Sample time (in seconds)
    float T;

    // controller "memory"
    float integrator;
    float prevError;
    // float differentiator;
    float prevMeasurement;

    // controller output
    float out;

} pid_controller_t;

// functions for PID control
void pid_controller_init(pid_controller_t *pid, bool is_left);
void pwm_init(mcpwm_cmpr_handle_t *left_cmp, mcpwm_cmpr_handle_t *right_cmp);
float pid_controller_update(pid_controller_t *pid, float setpoint, float measurement);
void pi_reset(pid_controller_t *pid);
void pi_task(void* arg);
