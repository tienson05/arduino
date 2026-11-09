#pragma once
#include <Arduino.h>

enum class CommandType : uint8_t
{
    None,
    WheelSpeed      // type 0x21: tốc độ mục tiêu 2 bánh (ticks/s)
};

struct Command
{
    CommandType type = CommandType::None;
    int16_t     left_tps  = 0;   // ticks/s, có dấu
    int16_t     right_tps = 0;
};

class CommandReceiver
{
public:
    explicit CommandReceiver(Stream &port) : _port(port) {}

    // Gọi liên tục trong loop(), không chặn.
    // Trả về true khi vừa nhận đủ một frame hợp lệ.
    bool poll(Command &cmd);

    uint16_t bad_frames() const { return _bad; }

private:
    void _drop(uint8_t n);      // bỏ n byte đầu rồi dồn về header kế tiếp

    static constexpr uint8_t HEADER    = 0xAA;
    static constexpr uint8_t FOOTER    = 0x55;
    static constexpr uint8_t TYPE_WHEEL = 0x21;
    static constexpr uint8_t FRAME_LEN = 8;

    Stream  &_port;
    uint8_t  _buf[FRAME_LEN];
    uint8_t  _len = 0;
    uint16_t _bad = 0;
};