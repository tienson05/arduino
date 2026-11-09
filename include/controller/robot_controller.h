#pragma once
#include <Arduino.h>
#include "controller/drive_controller.h"
#include "controller/pid.h"

// Điều khiển vận tốc hai bánh (setpoint từ Nav2 qua Pi, đơn vị ticks/s).
// Quy ước: tiến thì encoder TĂNG, M1 = trái, M2 = phải.
class RobotController
{
public:
    RobotController();

    void begin();
    void update();      // gọi liên tục trong loop(), không chặn

    // Đặt tốc độ mục tiêu (ticks/s, có dấu). Mất lệnh quá CMD_TIMEOUT_MS thì tự về 0.
    void set_wheel_speed(int16_t left_tps, int16_t right_tps);

private:
    DriveController _drive;
    PID      _pid_left;
    PID      _pid_right;

    uint32_t _last_update_ms;
    uint32_t _last_cmd_ms;
    int16_t  _target_left_tps;
    int16_t  _target_right_tps;
    int32_t  _prev_left_count;
    int32_t  _prev_right_count;
    float    _meas_left_tps;
    float    _meas_right_tps;
};