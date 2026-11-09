#pragma once

class PID
{
public:
    PID(float kp, float ki, float kd, float out_min, float out_max)
        : _kp(kp), _ki(ki), _kd(kd),
          _out_min(out_min), _out_max(out_max)
    {
        reset();
    }

    // Gọi mỗi khi bắt đầu nhiệm vụ mới
    void reset()
    {
        _integral   = 0.0f;
        _prev_error = 0.0f;
        _first      = true;
    }

    // error: sai số hiện tại, dt: chu kỳ (giây)
    float update(float error, float dt)
    {
        // D: lần đầu chưa có sai số cũ, nên bỏ qua để tránh giật
        float derivative = _first ? 0.0f : (error - _prev_error) / dt;
        _first      = false;
        _prev_error = error;

        // Tích phân "thử", chỉ lưu lại nếu output chưa bão hòa
        float integral_try = _integral + error * dt;

        float output = _kp * error + _ki * integral_try + _kd * derivative;

        if      (output > _out_max) output = _out_max;
        else if (output < _out_min) output = _out_min;
        else    _integral = integral_try;   // chống tích phân tràn (anti-windup)

        return output;
    }

private:
    float _kp, _ki, _kd;
    float _out_min, _out_max;

    float _integral;
    float _prev_error;
    bool  _first;
};