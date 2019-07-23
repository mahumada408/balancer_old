#include "library/pid_control.h"

PIDControl::PIDControl() {
    integral_sum_ = 0;
    previous_error_ = 0;
    set_point_ = 0;
    kp_ = 0;
    kd_ = 0;
    ki_ = 0;
}

double PIDControl::Compute(double current, double ts) {
    double error = set_point_ - current;
    integral_sum_ += error * ts;
    double control = kp_* error + ki_ * integral_sum_ + kd_ * ((error - previous_error_)/ts);

    // Save the error for good luck.
    previous_error_ = error;

    // Saturate the control output.
    if (control > max_clamp_) {
        control = max_clamp_;
    }
    else if (control < min_clamp_) {
        control = min_clamp_;
    }
    
    return control;
}

void PIDControl::SetGains(double kp, double ki, double kd) {
    kp_ = kp; 
    kd_ = kd; 
    ki_ = ki;
}

void PIDControl::Setpoint(double setpoint) {
    set_point_ = set_point_;
}

void PIDControl::SetLimits(double min, double max) {
    min_clamp_ = min;
    max_clamp_ = max;
}