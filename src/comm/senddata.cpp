#include "comm/senddata.h"
#include "middleware/encoder.h"

// CODE BLOCK FOR TEST
#include "controller/debug_test.h"
// END CODE BLOCK FOR TEST

constexpr uint8_t MSG_ENCODER = 0x11;
void send_encoder_data(Print &output)
{
    // CODE BLOCK FOR TEST
    #if DEBUG_TEST
        return;
    #endif
    // END CODE BLOCK FOR TEST
    uint32_t timestamp_ms;
    int32_t m1_count, m2_count;
    get_encoder_sample(timestamp_ms, m1_count, m2_count);

    uint8_t frame[16] = {};
    frame[0] = 0xAA;
    frame[1] = MSG_ENCODER;

    const uint32_t values[] = {
        timestamp_ms,
        static_cast<uint32_t>(m1_count),
        static_cast<uint32_t>(m2_count)
    };

    for (uint8_t value_index = 0; value_index < 3; ++value_index) {
        for (uint8_t byte_index = 0; byte_index < 4; ++byte_index) {
            const uint8_t payload_index = 2 + value_index * 4 + byte_index;
            frame[payload_index] = static_cast<uint8_t>(values[value_index] >> (byte_index * 8));
        }
    }

    uint8_t checksum = frame[1];
    for (uint8_t i = 2; i < 14; ++i) {
        checksum ^= frame[i];
    }
    frame[14] = checksum;
    frame[15] = 0x55;

    output.write(frame, sizeof(frame));
}
