#include "controller/drive_controller.h"

void DriveController::set(int16_t left_pwm, int16_t right_pwm)
{
    _left_motor.drive(left_pwm);
    _right_motor.drive(right_pwm);
}

void DriveController::brake()
{
    _left_motor.brake();
    _right_motor.brake();
}

void DriveController::coast()
{
    _left_motor.coast();
    _right_motor.coast();
}