#pragma once
#include <avr/io.h>
#include <stdint.h>

#if !defined(__AVR_ATmega328P__)
#error "fast_io.h chi ho tro ATmega328P (Arduino Uno/Nano)"
#endif

namespace fast_io {

// Chân Arduino → bit trong port
//   D0–D7   → PORTD bit 0–7
//   D8–D13  → PORTB bit 0–5
//   A0–A5   → PORTC bit 0–5 (chân 14–19)
constexpr uint8_t bit_of(uint8_t pin)
{
    return pin < 8 ? pin : (pin < 14 ? pin - 8 : pin - 14);
}

// Chân hỗ trợ PWM (analogWrite) trên Uno/Nano
constexpr bool is_pwm(uint8_t pin)
{
    return pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11;
}

template <uint8_t PIN>
struct Pin
{
    static_assert(PIN <= 19, "Chan khong ton tai tren ATmega328P");

    static constexpr uint8_t NUMBER = PIN;   // số chân Arduino, cho analogWrite, attachInterrupt…
    static constexpr uint8_t MASK   = static_cast<uint8_t>(1u << bit_of(PIN));

    static inline void output() // chân là OUTPUT
    {
        if      (PIN < 8)  DDRD |= MASK;
        else if (PIN < 14) DDRB |= MASK;
        else               DDRC |= MASK;
    }

    static inline void input()          // INPUT, tắt pull-up
    {
        if      (PIN < 8)  DDRD &= ~MASK;
        else if (PIN < 14) DDRB &= ~MASK;
        else               DDRC &= ~MASK;
        low();
    }

    static inline void input_pullup()   // INPUT_PULLUP
    {
        if      (PIN < 8)  DDRD &= ~MASK;
        else if (PIN < 14) DDRB &= ~MASK;
        else               DDRC &= ~MASK;
        high();                         // chân INPUT + PORT = 1 → bật pull-up
    }

    static inline void high()
    {
        if      (PIN < 8)  PORTD |= MASK;
        else if (PIN < 14) PORTB |= MASK;
        else               PORTC |= MASK;
    }

    static inline void low()
    {
        if      (PIN < 8)  PORTD &= ~MASK;
        else if (PIN < 14) PORTB &= ~MASK;
        else               PORTC &= ~MASK;
    }

    static inline void write(bool value)
    {
        if (value) high();
        else       low();
    }

    static inline bool read()
    {
        if      (PIN < 8)  return PIND & MASK;
        else if (PIN < 14) return PINB & MASK;
        else               return PINC & MASK;
    }

    // Ghi 1 vào thanh ghi PIN sẽ đảo trạng thái chân
    static inline void toggle()
    {
        if      (PIN < 8)  PIND = MASK;
        else if (PIN < 14) PINB = MASK;
        else               PINC = MASK;
    }
};

} // namespace fast_io