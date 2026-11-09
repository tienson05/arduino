#include "comm/command_receiver.h"

// CODE BLOCK FOR TEST
#include "controller/debug_test.h"

#if DEBUG_TEST
void dbg_test_on_serial_char(uint8_t byte);
#endif
// END CODE BLOCK FOR TEST

bool CommandReceiver::poll(Command &cmd)
{
    while (_port.available()) {
        const uint8_t b = _port.read();

        // CODE BLOCK FOR TEST
        #if DEBUG_TEST
            if (_len == 0 && (b == 'p' || b == 'r' || b == 's')) {
                dbg_test_on_serial_char(b);
                continue;
            }
        #endif
        // END CODE BLOCK FOR TEST

        if (_len == 0 && b != HEADER) continue;   // chưa có header: bỏ byte rác
        _buf[_len++] = b;

        if (_len < FRAME_LEN) continue;

        // Đủ 8 byte: kiểm tra footer, type và checksum (XOR type + payload)
        uint8_t chk = 0;
        for (uint8_t i = 1; i <= 5; ++i) chk ^= _buf[i];

        const bool ok = _buf[7] == FOOTER && _buf[6] == chk && _buf[1] == TYPE_WHEEL;
        if (!ok) {
            ++_bad;
            _drop(1);          // chỉ bỏ 1 byte, tìm header kế tiếp trong phần còn lại
            continue;
        }

        cmd.type      = CommandType::WheelSpeed;
        cmd.left_tps  = static_cast<int16_t>(uint16_t(_buf[2]) | uint16_t(_buf[3]) << 8);
        cmd.right_tps = static_cast<int16_t>(uint16_t(_buf[4]) | uint16_t(_buf[5]) << 8);
        _len = 0;
        return true;
    }
    return false;
}

void CommandReceiver::_drop(uint8_t n)
{
    memmove(_buf, _buf + n, _len - n);
    _len -= n;
    // dồn về byte HEADER đầu tiên còn lại
    uint8_t skip = 0;
    while (skip < _len && _buf[skip] != HEADER) ++skip;
    if (skip) {
        memmove(_buf, _buf + skip, _len - skip);
        _len -= skip;
    }
}