#pragma once
#include "middleware/motor.h"

class DriveController {
    Motor1 _left_motor;
    Motor2 _right_motor;

public:
    // PWM có dấu cho từng bánh: -255..255, dấu = chiều quay, 0 = thả trôi
    void set(int16_t left_pwm, int16_t right_pwm);
    void brake();
    void coast();
};