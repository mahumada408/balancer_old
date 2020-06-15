// Hello World to sweep a servo through its full range
 
#include "mbed.h"
#include "Servo.h"

#include "library/BNO055.h"
#include "library/pid_control.h"
 
PwmOut wheels(D5);
BNO055 imu(PB_9, PB_8);
AnalogIn pot_offset(A5);
Ticker controltime;
bool control_flag{false};

RawSerial pc(USBTX, USBRX); // tx, rx

// Interrupt service routine for the controller.
void UpdateControlFlag() {
    control_flag = true;
}
 
int main() {
    pc.baud(9600);
    pc.printf("here we go\n");
    int count{0};

    // Balancer being controlled by the servo output of the Nucleo.
    // 0.5 equates to a command of 0 velocity for the balancer.
    double control{0.5};

    // Pid parameters.
    // 0.015, 0.0, 0.000015
    PIDControl pid(0.012, 0.1, 0.00001);
    pid.SetLimits(-0.5, 0.5);
    PIDControl::Parameters pid_params;

    // Attach timer to interrup routine.
    controltime.attach(&UpdateControlFlag, pid_params.ts);

    // Start the wheels with zero velocity.
    wheels.period(0.005);
    wheels.write(control);

    // Imu initilization.
    while (!imu.check()){
        // Let the imu initialize
        pc.printf("Initializing imu.\n");
    }
    imu.setmode(OPERATION_MODE_NDOF);
    imu.get_angles();
    imu.get_gyro();
    pc.printf("%f, %f\n", imu.euler.roll, imu.gyro.x);

    // Balance setpoint for the controller.
    double initial_pose{0};
    double initial_angular_velocity{0};
    double balance_setpoint{-0.750};

    for (int i = 0; i <= 1000; i++) {
        // Let the signal settle.
        imu.get_angles();
        imu.get_gyro();
        initial_pose = imu.euler.roll;
        initial_angular_velocity = imu.gyro.x;
    }

    // Main loop
    double pidout{0};
    double roll{0};
    double roll_rate{0};
    while (true) {
        if (control_flag) {
            // Update offset from potentiometer.
            balance_setpoint = pot_offset.read() * 8 - 4;

            imu.get_angles();
            imu.get_gyro();
            roll = imu.euler.roll;
            roll_rate = imu.gyro.x;
            pidout = pid.ComputeWVelocity(balance_setpoint, roll, roll_rate);
            control = -pidout + 0.5;
            control_flag = false;
        }
        
        wheels.write(control);
        if (count >= 1000) {
            pc.printf("%f\n", balance_setpoint );
            count = 0;
        }
        count++;
    }
}