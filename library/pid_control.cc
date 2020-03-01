#include "library/pid_control.h"

PIDControl::PIDControl(double kp, double ki, double kd) {
    integral_sum_ = 0;
    previous_error_ = 0;
    
    pid_params.kp = kp;
    pid_params.ki = ki;
    pid_params.kd = kd;
}

double PIDControl::Compute(double setpoint, double current) {
    double error = setpoint - current;
    integral_sum_ += error * pid_params.ts;
    double control = pid_params.kp * error + pid_params.ki * integral_sum_ + pid_params.kd * ((error - previous_error_)/pid_params.ts);

    // Save the error for good luck.
    previous_error_ = error;

    // Saturate the control output.
    if (control > pid_params.max_clamp) {
        control = pid_params.max_clamp;
    }
    else if (control < pid_params.min_clamp) {
        control = pid_params.min_clamp;
    }
    
    return control;
}

double PIDControl::ComputeWVelocity(double setpoint, double current, double velocity) {
    double error = setpoint - current;
    integral_sum_ += error * pid_params.ts;
    double control = pid_params.kp * error + pid_params.ki * integral_sum_ + pid_params.kd * velocity;

    // Save the error for good luck.
    previous_error_ = error;

    // Saturate the control output.
    if (control > pid_params.max_clamp) {
        control = pid_params.max_clamp;
    }
    else if (control < pid_params.min_clamp) {
        control = pid_params.min_clamp;
    }
    
    return control;
}

void PIDControl::SetGains(double kp, double ki, double kd) {
    pid_params.kp = kp;
    pid_params.ki = ki;
    pid_params.kd = kd;
}

void PIDControl::SetLimits(double min, double max) {
    pid_params.min_clamp = min;
    pid_params.max_clamp = max;
}