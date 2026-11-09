#include <Arduino.h>
#include <math.h>
#include "controller/robot_controller.h"
#include "middleware/encoder.h"

// CODE BLOCK FOR TEST
#include "controller/debug_test.h"
// END CODE BLOCK FOR TEST

namespace {

constexpr uint32_t CONTROL_PERIOD_MS = 20;                       // 50 Hz
constexpr float    DT                = CONTROL_PERIOD_MS / 1000.0f;
constexpr uint32_t CMD_TIMEOUT_MS    = 300;
constexpr float    MAX_TPS           = 3000.0f;

constexpr float    VEL_KP      = 0.05f;
constexpr float    VEL_KI      = 0.50f;
constexpr float    VEL_KD      = 0.0f;
constexpr float    VEL_PID_MAX = 100.0f;

constexpr float    DEADBAND_TPS = 20.0f;
constexpr float    MEAS_ALPHA   = 0.5f;

// --- Khởi động (đứng yên -> quay) ---
constexpr int16_t  START_PWM_L  = 160;     // ĐO LẠI trên sàn thật, cộng dư 10-15
constexpr int16_t  START_PWM_R  = 170;
constexpr float    KICK_STEP    = 2.0f;    // mỗi 20 ms chưa quay thì +2 PWM
constexpr float    KICK_MAX     = 60.0f;   // tối đa cộng thêm
constexpr float    MOVING_ON    = 80.0f;   // |meas| > mức này => coi là đang quay
constexpr float    MOVING_OFF   = 40.0f;   // |meas| < mức này => quay lại chế độ khởi động

// --- Khi đang quay ---
constexpr int16_t  RUN_MIN_PWM  = 60;      // sàn thấp để PID còn chỗ hạ PWM

// --- Feedforward riêng từng bánh: PWM = off + k * tps ---
constexpr float    FF_OFF_L     = 60.0f;   // ước từ log: ~112 PWM tại 1000 tps
constexpr float    FF_OFF_R     = 90.0f;   //            ~142 PWM tại 1000 tps
constexpr float    FF_K         = 0.052f;  // ước lượng, PID sẽ bù phần sai

// --- Ramp ---
constexpr float    ACCEL_TPS2   = 4000.0f; // đạt 1000 tps trong 0.25 s

struct WheelState {
    float ramp   = 0.0f;   // target sau ramp (ticks/s, có dấu)
    float kick   = 0.0f;   // PWM cộng thêm khi bánh kẹt ở trạng thái khởi động
    bool  moving = false;
};
WheelState g_ws_left, g_ws_right;

int16_t speed_to_pwm(int16_t target, float measured, PID &pid,
                     WheelState &ws, int16_t start_pwm, float ff_off)
{
    // target ~ 0: dừng hẳn, reset mọi trạng thái
    if (fabsf((float)target) < DEADBAND_TPS) {
        pid.reset();
        ws.ramp = 0.0f; ws.kick = 0.0f; ws.moving = false;
        return 0;
    }
    const float sign = (target > 0) ? 1.0f : -1.0f;

    // Ramp target
    const float step = ACCEL_TPS2 * DT;
    ws.ramp += constrain((float)target - ws.ramp, -step, step);
    const float ramp_mag = fmaxf(0.0f, ws.ramp * sign);   // chỉ lấy phần cùng chiều target

    // Hysteresis: đang quay hay chưa
    if (!ws.moving && fabsf(measured) > MOVING_ON)  ws.moving = true;
    if ( ws.moving && fabsf(measured) < MOVING_OFF) ws.moving = false;

    const float ff = ff_off + FF_K * ramp_mag;            // độ lớn, chưa có dấu
    float out_mag;

    if (!ws.moving) {
        // Giai đoạn khởi động: PWM đủ lớn để thắng ma sát tĩnh, tăng dần nếu bánh vẫn đứng.
        // Không chạy PID để integral không windup.
        pid.reset();
        ws.kick = fminf(ws.kick + KICK_STEP, KICK_MAX);
        out_mag = fmaxf(ff, (float)start_pwm + ws.kick);
    } else {
        ws.kick = 0.0f;
        const float fb = pid.update(sign * ramp_mag - measured, DT) * sign;
        out_mag = fmaxf(ff + fb, (float)RUN_MIN_PWM);
    }

    return (int16_t)constrain(sign * out_mag, -255.0f, 255.0f);
}

} // namespace

// CODE BLOCK FOR TEST
#if DEBUG_TEST
namespace {
constexpr uint32_t dbg_csv_period_ms = 100;
constexpr uint32_t dbg_stats_period_ms = 1000;
constexpr uint32_t dbg_run_period_ms = 5000;

int32_t dbg_encoder_prev_l = 0;
int32_t dbg_encoder_prev_r = 0;
int32_t dbg_encoder_total_l = 0;
int32_t dbg_encoder_total_r = 0;

int32_t dbg_run_ticks_l = 0;
int32_t dbg_run_ticks_r = 0;
bool dbg_test_active = false;
uint32_t dbg_run_start_ms = 0;
float dbg_run_ratio_sum = 0.0f;
uint32_t dbg_run_ratio_count = 0;

uint32_t dbg_last_csv_ms = 0;
uint32_t dbg_window_start_ms = 0;
uint32_t dbg_window_samples = 0;
float dbg_window_meas_l = 0.0f;
float dbg_window_meas_r = 0.0f;
float dbg_window_pwm_l = 0.0f;
float dbg_window_pwm_r = 0.0f;
float dbg_window_ratio_sum = 0.0f;
uint32_t dbg_window_ratio_count = 0;

bool dbg_pwm_clamped(int16_t pwm)
{
    const int value = abs((int)pwm);
    return pwm != 0 && (value == RUN_MIN_PWM || value == 255);
}
}

void dbg_test_on_serial_char(uint8_t byte)
{
    int32_t dbg_left_count;
    int32_t dbg_right_count;
    get_encoder_count(dbg_left_count, dbg_right_count);

    if (byte == 'p') {
        Serial.print(F("ENC,"));
        Serial.print(dbg_encoder_total_l);
        Serial.print(',');
        Serial.println(dbg_encoder_total_r);
    } else if (byte == 'r') {
        dbg_encoder_total_l = 0;
        dbg_encoder_total_r = 0;
        dbg_encoder_prev_l = dbg_left_count;
        dbg_encoder_prev_r = dbg_right_count;
        Serial.println(F("ENC_RESET"));
    } else if (byte == 's' && !dbg_test_active) {
        dbg_run_ticks_l = 0;
        dbg_run_ticks_r = 0;
        dbg_run_ratio_sum = 0.0f;
        dbg_run_ratio_count = 0;
        dbg_encoder_prev_l = dbg_left_count;
        dbg_encoder_prev_r = dbg_right_count;
        dbg_run_start_ms = millis();
        dbg_test_active = true;
        Serial.println(F("TEST_START"));
    }
}
#endif
// END CODE BLOCK FOR TEST


RobotController::RobotController()
    : _pid_left (VEL_KP, VEL_KI, VEL_KD, -VEL_PID_MAX, VEL_PID_MAX),
      _pid_right(VEL_KP, VEL_KI, VEL_KD, -VEL_PID_MAX, VEL_PID_MAX),
      _last_update_ms(0),
      _last_cmd_ms(0),
      _target_left_tps(0), _target_right_tps(0),
      _prev_left_count(0), _prev_right_count(0),
      _meas_left_tps(0), _meas_right_tps(0)
{
}

void RobotController::begin()
{
    init_encoder_interrupt();
    reset_encoder_count();

    get_encoder_count(_prev_left_count, _prev_right_count);
    _last_update_ms = millis();
    _last_cmd_ms    = millis();

    // CODE BLOCK FOR TEST
    #if DEBUG_TEST
        dbg_encoder_prev_l = _prev_left_count;
        dbg_encoder_prev_r = _prev_right_count;
        dbg_last_csv_ms = millis();
        dbg_window_start_ms = millis();
        Serial.println(F("t_ms,target_l,target_r,meas_l,meas_r,pwm_l,pwm_r,err_l,err_r"));
    #endif
    // END CODE BLOCK FOR TEST
}

void RobotController::set_wheel_speed(int16_t left_tps, int16_t right_tps)
{
    _target_left_tps  = constrain(left_tps,  (int16_t)-MAX_TPS, (int16_t)MAX_TPS);
    _target_right_tps = constrain(right_tps, (int16_t)-MAX_TPS, (int16_t)MAX_TPS);
    _last_cmd_ms = millis();
}

void RobotController::update()
{
    if (millis() - _last_update_ms < CONTROL_PERIOD_MS) return;
    _last_update_ms += CONTROL_PERIOD_MS;

    // CODE BLOCK FOR TEST
    #if DEBUG_TEST
        if (dbg_test_active) {
            set_wheel_speed(1000, 1000);
        }
    #endif
    // END CODE BLOCK FOR TEST

    // Mất lệnh -> dừng (Pi treo, rút dây)
    if (millis() - _last_cmd_ms > CMD_TIMEOUT_MS) {
        _target_left_tps = _target_right_tps = 0;
    }

    // Tốc độ đo (ticks/s), lọc nhẹ
    int32_t lc, rc;
    get_encoder_count(lc, rc);

    // CODE BLOCK FOR TEST
    #if DEBUG_TEST
        {
            const int32_t dbg_delta_l = lc - dbg_encoder_prev_l;
            const int32_t dbg_delta_r = rc - dbg_encoder_prev_r;
            dbg_encoder_prev_l = lc;
            dbg_encoder_prev_r = rc;
            dbg_encoder_total_l += dbg_delta_l;
            dbg_encoder_total_r += dbg_delta_r;

            if (dbg_test_active) {
                dbg_run_ticks_l += dbg_delta_l;
                dbg_run_ticks_r += dbg_delta_r;
            }
        }
    #endif
    // END CODE BLOCK FOR TEST

    const float raw_l = (float)(lc - _prev_left_count)  / DT;
    const float raw_r = (float)(rc - _prev_right_count) / DT;
    _prev_left_count  = lc;
    _prev_right_count = rc;
    _meas_left_tps  += MEAS_ALPHA * (raw_l - _meas_left_tps);
    _meas_right_tps += MEAS_ALPHA * (raw_r - _meas_right_tps);

    // PID từng bánh
    const int16_t pl = speed_to_pwm(_target_left_tps,  _meas_left_tps,  _pid_left,
                                g_ws_left,  START_PWM_L, FF_OFF_L);
    const int16_t pr = speed_to_pwm(_target_right_tps, _meas_right_tps, _pid_right,
                                g_ws_right, START_PWM_R, FF_OFF_R);

    // CODE BLOCK FOR TEST
    #if DEBUG_TEST
        {
            const uint32_t dbg_now = millis();
            const float dbg_err_l = (float)_target_left_tps - _meas_left_tps;
            const float dbg_err_r = (float)_target_right_tps - _meas_right_tps;

            if (dbg_now - dbg_last_csv_ms >= dbg_csv_period_ms) {
                dbg_last_csv_ms = dbg_now;
                Serial.print(dbg_now);
                Serial.print(',');
                Serial.print(_target_left_tps);
                Serial.print(',');
                Serial.print(_target_right_tps);
                Serial.print(',');
                Serial.print(_meas_left_tps, 1);
                Serial.print(',');
                Serial.print(_meas_right_tps, 1);
                Serial.print(',');
                Serial.print(pl);
                Serial.print(',');
                Serial.print(pr);
                Serial.print(',');
                Serial.print(dbg_err_l, 1);
                Serial.print(',');
                Serial.println(dbg_err_r, 1);
            }

            dbg_window_meas_l += _meas_left_tps;
            dbg_window_meas_r += _meas_right_tps;
            dbg_window_pwm_l += pl;
            dbg_window_pwm_r += pr;
            ++dbg_window_samples;

            if (fabsf(_meas_right_tps) > 1.0f) {
                dbg_window_ratio_sum += _meas_left_tps / _meas_right_tps;
                ++dbg_window_ratio_count;
            }

            if (dbg_test_active && fabsf(_meas_right_tps) > 1.0f) {
                dbg_run_ratio_sum += _meas_left_tps / _meas_right_tps;
                ++dbg_run_ratio_count;
            }

            if (dbg_now - dbg_window_start_ms >= dbg_stats_period_ms) {
                const bool dbg_clamp_l = dbg_pwm_clamped(pl);
                const bool dbg_clamp_r = dbg_pwm_clamped(pr);

                Serial.print(F("STAT,"));
                if (dbg_window_samples > 0) {
                    Serial.print(dbg_window_meas_l / dbg_window_samples, 1);
                    Serial.print(',');
                    Serial.print(dbg_window_meas_r / dbg_window_samples, 1);
                    Serial.print(',');
                    Serial.print(dbg_window_pwm_l / dbg_window_samples, 1);
                    Serial.print(',');
                    Serial.print(dbg_window_pwm_r / dbg_window_samples, 1);
                } else {
                    Serial.print(F("NA,NA,NA,NA"));
                }

                Serial.print(',');
                if (dbg_window_ratio_count > 0) {
                    Serial.print(dbg_window_ratio_sum / dbg_window_ratio_count, 3);
                } else {
                    Serial.print(F("NA"));
                }
                Serial.print(',');
                Serial.print(dbg_clamp_l ? 1 : 0);
                Serial.print(',');
                Serial.println(dbg_clamp_r ? 1 : 0);

                dbg_window_start_ms = dbg_now;
                dbg_window_samples = 0;
                dbg_window_meas_l = 0.0f;
                dbg_window_meas_r = 0.0f;
                dbg_window_pwm_l = 0.0f;
                dbg_window_pwm_r = 0.0f;
                dbg_window_ratio_sum = 0.0f;
                dbg_window_ratio_count = 0;
            }
        }
    #endif
    // END CODE BLOCK FOR TEST

    // CODE BLOCK FOR TEST
    #if DEBUG_TEST
        {
            const uint32_t dbg_now = millis();

            if (dbg_test_active && dbg_now - dbg_run_start_ms >= dbg_run_period_ms) {
                const int32_t dbg_tick_diff = dbg_run_ticks_l - dbg_run_ticks_r;

                Serial.print(F("TEST_END,"));
                Serial.print(dbg_tick_diff);
                Serial.print(',');

                if (dbg_run_ratio_count > 0) {
                    const float dbg_ratio = dbg_run_ratio_sum / dbg_run_ratio_count;
                    const float dbg_deviation_pct = fabsf(1.0f - dbg_ratio) * 100.0f;
                    Serial.print(dbg_ratio, 3);
                    Serial.print(',');
                    Serial.println(dbg_deviation_pct, 1);
                } else {
                    Serial.println(F("NA,NA"));
                }

                dbg_test_active = false;
                set_wheel_speed(0, 0);
            }
        }
    #endif
    // END CODE BLOCK FOR TEST


    if (pl == 0 && pr == 0) _drive.brake();     // target = 0 thì phanh, không để PID tự kéo về 0
    else                    _drive.set(pl, pr);
}