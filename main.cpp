// Hello World to sweep a servo through its full range
 
#include "mbed.h"
#include "Servo.h"

#include "library/BNO055.h"
#include "library/pid_control.h"
 
Servo wheels(D5);
BNO055 imu(PB_9, PB_8);
Timer t;
Timeout controltime;
bool control_flag{false};

RawSerial pc(USBTX, USBRX); // tx, rx

void UpdateControlFlag() {
    control_flag = true;
}
 
int main() {
    pc.baud(9600);
    pc.printf("here we go\n");
    int count{0};
    double control{0.5};

    // Pid parameters.
    PIDControl pid(1.0, 0, 0);
    pid.SetLimits(0, 0.5);
    double balance_setpoint{0};

    // Attach timer to interrup routine.
    controltime.attach(&UpdateControlFlag, 0.005);

    // Start the wheels with zero velocity.
    wheels = control;

    // Imu initilization.
    while (!imu.check()){
        // Let the imu initialize
        pc.printf("checking imu.\n");
    }
    imu.setmode(OPERATION_MODE_NDOF);
    imu.get_angles();
    pc.printf("%f, %f, %f\n", imu.euler.roll, imu.euler.pitch, imu.euler.yaw);

    // Main loop
    while (true) {
        imu.get_angles();
        // servo_val = (imu.euler.roll/180) + 0.5;
        if (control_flag) {
            control = pid.Compute(balance_setpoint, imu.euler.roll) + 0.5;
            control_flag = false;
        }
        
        wheels = control;
        if (count >= 500) {
            pc.printf("%f, %f\n", servo_val, imu.euler.roll);
            count = 0;
        }
        count++;
    }
}