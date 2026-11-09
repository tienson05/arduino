#pragma once
#include <Arduino.h>
#include "pins.h"

// ===== Kiểm tra cấu hình từ platformio.ini =====
#if !defined(DEFAULT_SPEED)
#error "Thieu DEFAULT_SPEED trong build_flags"
#endif


enum class Direction : uint8_t
{
    Forward,
    Backward
};


// INVERTED     : true nếu motor lắp ngược (ví dụ bánh bên phải)
template <typename In1, typename In2, typename En, bool INVERTED = false>
class Motor
{
    static_assert(In1::NUMBER != In2::NUMBER, "IN1 va IN2 phai la 2 chan khac nhau");

public:
    // Quay theo chiều, tốc độ 0–255
    void run(Direction dir, uint8_t speed = DEFAULT_SPEED)
    {
        const bool forward = (dir == Direction::Forward) != INVERTED;

        // Hạ chân LOW trước rồi mới nâng chân HIGH:
        // khi đảo chiều, trạng thái trung gian luôn là (LOW, LOW) = thả trôi
        if (forward) { In2::low(); In1::high(); }
        else         { In1::low(); In2::high(); }

        analogWrite(En::NUMBER, speed);
    }

    void forward(uint8_t speed = DEFAULT_SPEED)  { run(Direction::Forward, speed); }
    void backward(uint8_t speed = DEFAULT_SPEED) { run(Direction::Backward, speed); }

    // Tốc độ có dấu: -255..255 (dương = tiến, âm = lùi, 0 = thả trôi)
    // Tiện để nối thẳng với output của bộ PID
    void drive(int16_t speed)
    {
        speed = constrain(speed, -255, 255);

        if      (speed > 0) run(Direction::Forward,  speed);
        else if (speed < 0) run(Direction::Backward, -speed);
        else                coast();
    }

    // Thả trôi: EN = LOW, motor quay theo quán tính rồi tự dừng
    void coast()
    {
        analogWrite(En::NUMBER, 0);
        In1::low();
        In2::low();
    }

    // Phanh: EN = HIGH, IN1 = IN2, L298N nối tắt 2 đầu motor nên dừng nhanh
    void brake()
    {
        In1::low();
        In2::low();
        analogWrite(En::NUMBER, 255);
    }

    // Chạy theo quán tính rồi tự dừng
    void stop() { coast(); }
};


// ===== Cấu hình motor của robot =====
using Motor1 = Motor<pins::M1_IN1, pins::M1_IN2, pins::M1_EN, true>;
using Motor2 = Motor<pins::M2_IN1, pins::M2_IN2, pins::M2_EN>;