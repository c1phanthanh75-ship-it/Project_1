#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    double kp;
    double ki;
    double kd;
    double setpoint;
    double output;
    double output_min;
    double output_max;
    double output_sum;
    double last_input;
    uint32_t sample_time_ms;
    uint32_t last_time_ms;
    bool automatic;
} pid_controller_t;

void pid_controller_init(pid_controller_t *pid,
                         double kp,
                         double ki,
                         double kd,
                         double setpoint,
                         uint32_t sample_time_ms);
void pid_controller_set_tunings(pid_controller_t *pid, double kp, double ki, double kd);
void pid_controller_set_output_limits(pid_controller_t *pid, double min_value, double max_value);
void pid_controller_set_mode(pid_controller_t *pid, bool automatic, double input, uint32_t now_ms);
bool pid_controller_compute(pid_controller_t *pid, double input, uint32_t now_ms);

#endif
