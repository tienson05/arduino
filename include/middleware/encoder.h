#pragma once

void init_encoder_interrupt(void);
void get_encoder_count(int32_t &encoder_count_m1, int32_t &encoder_count_m2);
void get_encoder_sample(uint32_t &timestamp_ms, int32_t &encoder_count_m1, int32_t &encoder_count_m2);
void reset_encoder_count(void);
