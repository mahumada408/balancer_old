#pragma once

class PIDControl {
    public:
        PIDControl();
        ~PIDControl();

        // Main cpid control algorithm.
        double Compute(double current, double ts);

        // Set the gains for the controller.
        void SetGains(double kp, double ki, double kd);

        // Change the reference setpoint for the controller.
        void Setpoint(double set_point);

        void SetLimits(double min, double max);

    private:
    // Controller gains.
    double kp_, ki_, kd_;

    // Reference set point for the controller.
    double set_point_;

    // Sum integral error.
    double integral_sum_;

    // Previous error.
    double previous_error_;

    // Output clamps.
    double min_clamp_;
    double max_clamp_;

    // Sample time for the controller.
    double ts_; 
};