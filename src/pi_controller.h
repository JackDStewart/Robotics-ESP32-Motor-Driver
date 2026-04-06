
typedef struct {

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
void pid_controller_init(pid_controller_t *pid);
float pid_controller_update(pid_controller_t *pid, float setpoint, float measurement);
void pi_reset(pid_controller_t *pid);
void pi_task(void* arg);
