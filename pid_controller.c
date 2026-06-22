#include "pid_controller.h"

static double clamp_double(double value, double min_value, double max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

void pid_controller_set_output_limits(pid_controller_t *pid, double min_value, double max_value)
{
    if (min_value >= max_value) {
        return;
    }

    pid->output_min = min_value;
    pid->output_max = max_value;

    if (pid->automatic) {
        pid->output = clamp_double(pid->output, pid->output_min, pid->output_max);
        pid->output_sum = clamp_double(pid->output_sum, pid->output_min, pid->output_max);
    }
}

void pid_controller_set_tunings(pid_controller_t *pid, double kp, double ki, double kd)
{
    if (kp < 0.0 || ki < 0.0 || kd < 0.0 || pid->sample_time_ms == 0) {
        return;
    }

    const double sample_time_sec = (double)pid->sample_time_ms / 1000.0;
    pid->kp = kp;
    pid->ki = ki * sample_time_sec;
    pid->kd = kd / sample_time_sec;
}

static void pid_controller_initialize(pid_controller_t *pid, double input, uint32_t now_ms)
{
    pid->output_sum = clamp_double(pid->output, pid->output_min, pid->output_max);
    pid->last_input = input;
    pid->last_time_ms = now_ms - pid->sample_time_ms;
}

void pid_controller_set_mode(pid_controller_t *pid, bool automatic, double input, uint32_t now_ms)
{
    const bool switching_to_auto = automatic && !pid->automatic;
    pid->automatic = automatic;

    if (switching_to_auto) {
        pid_controller_initialize(pid, input, now_ms);
    }
}

void pid_controller_init(pid_controller_t *pid,
                         double kp,
                         double ki,
                         double kd,
                         double setpoint,
                         uint32_t sample_time_ms)
{
    *pid = (pid_controller_t) {
        .setpoint = setpoint,
        .output_min = 0.0,
        .output_max = 255.0,
        .sample_time_ms = sample_time_ms,
    };

    pid_controller_set_tunings(pid, kp, ki, kd);
}

bool pid_controller_compute(pid_controller_t *pid, double input, uint32_t now_ms)
{
    if (!pid->automatic) {
        return false;
    }

    const uint32_t elapsed_ms = now_ms - pid->last_time_ms;
    if (elapsed_ms < pid->sample_time_ms) {
        return false;
    }

    const double error = pid->setpoint - input;
    const double d_input = input - pid->last_input;

    pid->output_sum += pid->ki * error;
    pid->output_sum = clamp_double(pid->output_sum, pid->output_min, pid->output_max);

    pid->output = (pid->kp * error) + pid->output_sum - (pid->kd * d_input);
    pid->output = clamp_double(pid->output, pid->output_min, pid->output_max);

    pid->last_input = input;
    pid->last_time_ms = now_ms;
    return true;
}
