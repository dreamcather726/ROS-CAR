#include "odom_frame.h"

static inline void write_i16_le(uint8_t *out, int16_t v)
{
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

static inline void write_i24_le(uint8_t *out, int32_t v)
{
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
}

void odom_frame_build_counts_i24_le(int32_t left_count, int32_t right_count, uint8_t out6[6])
{
  write_i24_le(&out6[0], left_count);
  write_i24_le(&out6[3], right_count);
}

void odom_frame_build_speed_i16_le_x100(float left_cm_s, float right_cm_s, uint8_t out6[6])
{
  const float scale = 100.0f;
  int32_t ls = static_cast<int32_t>(left_cm_s * scale);
  int32_t rs = static_cast<int32_t>(right_cm_s * scale);
  if (ls > 32767) ls = 32767;
  if (ls < -32768) ls = -32768;
  if (rs > 32767) rs = 32767;
  if (rs < -32768) rs = -32768;

  out6[4] = 0;
  out6[5] = 0;
  write_i16_le(&out6[0], static_cast<int16_t>(ls));
  write_i16_le(&out6[2], static_cast<int16_t>(rs));
}
