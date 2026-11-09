#pragma once
#include "board/io.h"

// ===== Kiểm tra build_flags =====
#if !defined(PIN_M1_EN) || !defined(PIN_M1_IN1) || !defined(PIN_M1_IN2) || \
    !defined(PIN_M2_EN) || !defined(PIN_M2_IN1) || !defined(PIN_M2_IN2)
#error "Thieu khai bao chan motor trong build_flags"
#endif

#if !defined(PIN_M1_H1) || !defined(PIN_M1_H2) || \
    !defined(PIN_M2_H1) || !defined(PIN_M2_H2)
#error "Thieu khai bao chan encoder trong build_flags"
#endif

namespace pins {

// ===== Driver L298N =====
using M1_EN  = fast_io::Pin<PIN_M1_EN>;
using M1_IN1 = fast_io::Pin<PIN_M1_IN1>;
using M1_IN2 = fast_io::Pin<PIN_M1_IN2>;

using M2_EN  = fast_io::Pin<PIN_M2_EN>;
using M2_IN1 = fast_io::Pin<PIN_M2_IN1>;
using M2_IN2 = fast_io::Pin<PIN_M2_IN2>;

// ===== Encoder =====
using M1_H1 = fast_io::Pin<PIN_M1_H1>;
using M1_H2 = fast_io::Pin<PIN_M1_H2>;

using M2_H1 = fast_io::Pin<PIN_M2_H1>;
using M2_H2 = fast_io::Pin<PIN_M2_H2>;

// ===== Ràng buộc phần cứng của board =====
static_assert(fast_io::is_pwm(M1_EN::NUMBER) && fast_io::is_pwm(M2_EN::NUMBER),
              "Chan EN phai ho tro PWM: 3, 5, 6, 9, 10, 11");

static_assert((M1_H1::NUMBER == 2 || M1_H1::NUMBER == 3) &&
              (M2_H1::NUMBER == 2 || M2_H1::NUMBER == 3) &&
              M1_H1::NUMBER != M2_H1::NUMBER,
              "H1 cua 2 encoder phai la D2 va D3 (ngat ngoai)");

// ===== Khởi tạo, gọi đầu tiên trong setup() =====
inline void initPins()
{
    M1_EN::output();  M1_IN1::output();  M1_IN2::output();
    M2_EN::output();  M2_IN1::output();  M2_IN2::output();

    M1_H1::input_pullup();  M1_H2::input_pullup();
    M2_H1::input_pullup();  M2_H2::input_pullup();
}

} // namespace pins