// Hello World to sweep a servo through its full range
 
#include "mbed.h"
#include "Servo.h"

#include "library/BNO055.h"
#include "library/pid_control.h"
 
Servo myservo(D5);
BNO055 imu(PB_9, PB_8);
Timer t;

RawSerial pc(USBTX, USBRX); // tx, rx
 
int main() {
    pc.baud(9600);
    pc.printf("here we go\n");
    int count{0};
    double servo_val{0};
    PIDControl pid;
    pid.SetGains(0.1,0,0);
    pid.SetLimits(0, 0.5);

    myservo = 0.5;

    while (!imu.check()){
        // Let the imu initialize
        pc.printf("checking imu.\n");
    }
    imu.setmode(OPERATION_MODE_NDOF);
    imu.get_angles();
    pc.printf("%f, %f, %f\n", imu.euler.roll, imu.euler.pitch, imu.euler.yaw);

    while (true) {
        imu.get_angles();
        // servo_val = (imu.euler.roll/180) + 0.5;
        servo_val = pid.Compute(imu.euler.roll, ts) + 0.5;
        myservo = servo_val;
        if (count >= 500) {
            pc.printf("%f, %f\n", servo_val, imu.euler.roll);
            count = 0;
        }
        count++;
    }
}