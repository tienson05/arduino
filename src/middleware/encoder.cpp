
#include <Arduino.h>
#include <util/atomic.h>
#include <PinChangeInterrupt.h>

#include "middleware/encoder.h"
#include "pins.h"

// Chiều A:  00 → 01 → 11 → 10 → 00
// Chiều B:  00 → 10 → 11 → 01 → 00
// Nếu muốn đổi chiều đếm, có thể đổi vị trí hai dây H1/H2 hoặc đảo dấu giá trị đếm trong code.

static volatile int32_t _pulse_count_m1 = 0;
static volatile int32_t _pulse_count_m2 = 0;

// Trạng thái encoder của hai motor trước đó: bit 1 = H1, bit 0 = H2
static volatile uint8_t _previous_state_m1 = 0;
static volatile uint8_t _previous_state_m2 = 0;

// Bảng chuyển trạng thái QUADRATURE
// Gia tri:
//  +1: quay theo một chiều
//  -1: quay theo chiều ngược lại
//   0: không thay đổi hoặc chuyển trạng thái không hợp lệ
static const int8_t _encoder_transition_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};


// Đọc trạng thái encoder
static inline uint8_t _read_encoder_state_m1()
{
    return (pins::M1_H1::read() << 1) | pins::M1_H2::read();
}


static inline uint8_t _read_encoder_state_m2()
{
    return (pins::M2_H1::read() << 1) | pins::M2_H2::read();
}


// Cập nhật bộ đếm encoder
static inline void _update_encoder_m1()
{
    uint8_t current_state = _read_encoder_state_m1();

    uint8_t index = (_previous_state_m1 << 2) | current_state;

    _pulse_count_m1 += _encoder_transition_table[index];

    _previous_state_m1 = current_state;
}


static inline void _update_encoder_m2()
{
    uint8_t current_state = _read_encoder_state_m2();

    uint8_t index = (_previous_state_m2 << 2) | current_state;

    _pulse_count_m2 -= _encoder_transition_table[index];

    _previous_state_m2 = current_state;
}

// ISR - Interrupt Service Routine - Hàm phục vụ ngắt
// ISR - MOTOR 1
static void _encoder_isr_m1()
{
    _update_encoder_m1();
}


// ISR - MOTOR 2
static void _encoder_isr_m2()
{
    _update_encoder_m2();
}


// Khởi tạo encoder
void init_encoder_interrupt(void)
{
    // Đọc trạng thái bắt đầu trước khi bật ngắt
    _previous_state_m1 = _read_encoder_state_m1();
    _previous_state_m2 = _read_encoder_state_m2();

    // Motor 1 - H1
    attachInterrupt(digitalPinToInterrupt(pins::M1_H1::NUMBER), _encoder_isr_m1, CHANGE);

    // Motor 1 - H2
    attachPCINT(digitalPinToPCINT(pins::M1_H2::NUMBER), _encoder_isr_m1, CHANGE);

    // Motor 2 - H1
    attachInterrupt(digitalPinToInterrupt(pins::M2_H1::NUMBER), _encoder_isr_m2, CHANGE);

    // Motor 2 - H2
    attachPCINT(digitalPinToPCINT(pins::M2_H2::NUMBER), _encoder_isr_m2, CHANGE);
}

// Đọc bộ đếm của hai motor
void get_encoder_count(int32_t &encoder_count_m1, int32_t &encoder_count_m2)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        encoder_count_m1 = _pulse_count_m1;
        encoder_count_m2 = _pulse_count_m2;
    }
}

// Lấy timestamp và cả hai bộ đếm trong cùng vùng nguyên tử để chúng thuộc một mẫu.
void get_encoder_sample(uint32_t &timestamp_ms, int32_t &encoder_count_m1, int32_t &encoder_count_m2)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        timestamp_ms = millis();
        encoder_count_m1 = _pulse_count_m1;
        encoder_count_m2 = _pulse_count_m2;
    }
}

// Reset bộ đếm
void reset_encoder_count(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        _pulse_count_m1 = 0;
        _pulse_count_m2 = 0;
        
        _previous_state_m1 = _read_encoder_state_m1();
        _previous_state_m2 = _read_encoder_state_m2();
    }
}
