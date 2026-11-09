#include <Arduino.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "pins.h"
#include "comm/command_receiver.h"
#include "comm/senddata.h"
#include "controller/robot_controller.h"
// #include "middleware/encoder.h"

constexpr uint16_t WDT_MAGIC = 0xD06F;
uint16_t wdt_reset_mark __attribute__((section(".noinit")));

ISR(WDT_vect) { wdt_reset_mark = WDT_MAGIC; }

constexpr uint32_t ENCODER_PERIOD_MS = 20;      // 50 Hz

// CODE BLOCK FOR TEST
// constexpr uint32_t ENCODER_PRINT_PERIOD_MS = 1000;
// constexpr bool ENCODER_DEBUG_PRINT = true;
// ---END CODE BLOCK FOR TEST 

CommandReceiver receiver(Serial);
RobotController robot;
uint32_t last_encoder_ms = 0;

// CODE BLOCK FOR TEST
// void print_encoder_parameters()
// {
//     static uint32_t last_print_ms = 0;
//     const uint32_t now = millis();
//     if (now - last_print_ms < ENCODER_PRINT_PERIOD_MS) {
//         return;
//     }
//     last_print_ms = now;

//     int32_t encoder_count_m1;
//     int32_t encoder_count_m2;
//     get_encoder_count(encoder_count_m1, encoder_count_m2);

//     Serial.print(F("M1 encoder: "));
//     Serial.print(encoder_count_m1);
//     Serial.print(F(", M2 encoder: "));
//     Serial.println(encoder_count_m2);
// }
// END CODE BLOCK FOR TEST

void setup()
{
    MCUSR = 0;
    wdt_disable();

    const bool reset_by_wdt = (wdt_reset_mark == WDT_MAGIC);
    wdt_reset_mark = 0;

    Serial.begin(115200);
    
    pins::initPins();
    robot.begin();

    if (reset_by_wdt) Serial.println(F("RESET WDT"));   // ASCII, parser trên Pi tự bỏ qua

    wdt_enable(WDTO_250MS);
    WDTCSR |= (1 << WDIE);
}

void loop()
{
    wdt_reset();

    Command cmd;
    if (receiver.poll(cmd) && cmd.type == CommandType::WheelSpeed) {
        robot.set_wheel_speed(cmd.left_tps, cmd.right_tps);
    }

    robot.update();     // PID tốc độ + tự dừng khi mất lệnh

    // CODE BLOCK FOR TEST
    // if (ENCODER_DEBUG_PRINT) {
    //     print_encoder_parameters();
    // } else {
    //     const uint32_t now = millis();
    //     if (now - last_encoder_ms >= ENCODER_PERIOD_MS) {
    //         last_encoder_ms = now;
    //         send_encoder_data(Serial);
    //     }
    // }
    // END CODE BLOCK FOR TEST

    const uint32_t now = millis();
    if (now - last_encoder_ms >= ENCODER_PERIOD_MS) {
        last_encoder_ms = now;
        send_encoder_data(Serial);
    }
}