#pragma once

class PIDControl {
    public:
        struct Parameters
        {
            double kp = 0;
            double ki = 0;
            double kd = 0;
            double min_clamp = 0;
            double max_clamp = 0;
            double ts = 0.005;

        };
        
        PIDControl(double kp, double ki, double kd);
        ~PIDControl();

        // Main cpid control algorithm.
        double Compute(double setpoint, double current);

        // Set the gains for the controller.
        void SetGains(double kp, double ki, double kd);

        void SetLimits(double min, double max);

    private:
    // Parameters for the controller.
    Parameters pid_params;

    // Sum integral error.
    double integral_sum_;

    // Previous error.
    double previous_error_;

    // Sample time for the controller.
    double ts_; 
};