/*
Copyright 2019 Tomoaki Itoh
This software is released under the MIT License, see LICENSE.txt.
//*/

#include "userosc.h"
#include "LCWCommon.h"
#include "LCWPitchTable.h"
#include "LCWAntiAliasingTable.h"
#include "LCWOscWaveSource.h"

#define LCW_OSC_TIMER_BITS (LCW_PITCH_DELTA_VALUE_BITS)
#define LCW_OSC_TIMER_MAX (1 << LCW_OSC_TIMER_BITS)
#define LCW_OSC_TIMER_MASK ((LCW_OSC_TIMER_MAX) - 1)

static struct {
    const int16_t fn;
    const int16_t pitch;  // SQ3.10
    const int16_t gain;   // SQ3.10
} s_oscLutOption[] = {
  {  1,     0,  1024 },
  {  2,  1024,   512 },
  {  3,  1623,   341 },
  {  4,  2048,   256 },
  {  5,  2378,   205 },
  {  6,  2647,   171 },
  {  7,  2875,   146 },
  {  8,  3072,   128 },
  {  9,  3246,   114 },
  { 10,  3402,   102 },
  { 11,  3542,    93 },
  { 12,  3671,    85 },
  { 13,  3789,    79 },
  { 14,  3899,    73 },
  { 15,  4001,    68 },
  { 16,  4096,    64 },
  { 17,  4186,    60 },
  { 18,  4270,    57 },
  { 19,  4350,    54 },
  { 20,  4426,    51 },
  { 21,  4498,    49 },
  { 22,  4566,    47 },
  { 23,  4632,    45 },
  { 24,  4695,    43 },
  { 25,  4755,    41 },
  { 26,  4813,    39 },
  { 27,  4869,    38 },
  { 28,  4923,    37 },
  { 29,  4975,    35 },
  { 30,  5025,    34 },
  { 31,  5073,    33 },
  { 32,  5120,    32 },
  { 33,  5165,    31 },
  { 34,  5210,    30 },
  { 35,  5252,    29 },
  { 36,  5294,    28 },
  { 37,  5334,    28 },
  { 38,  5374,    27 },
  { 39,  5412,    26 },
  { 40,  5450,    26 },
  { 41,  5486,    25 },
  { 42,  5522,    24 },
  { 43,  5556,    24 },
  { 44,  5590,    23 },
  { 45,  5624,    23 },
  { 46,  5656,    22 },
  { 47,  5688,    22 },
  { 48,  5719,    21 }
};

static struct {
  float shape = 0;
  float shiftshape = 0;
} s_param;

static struct {
  uint32_t timer1 = 0;
  uint32_t timer2 = 0;
  int32_t pitch1 = 0;   // s7.24
  int32_t pitch2 = 0;   // s7.24
  int32_t shape_lfo = 0;
} s_state;

// pitch/return : s7.24
__fast_inline int32_t pitchLimitter(int32_t pitch)
{
#if 0
  if ( LCW_SQ7_24(6.0) < pitch ) {
    // 1:8
    return LCW_SQ7_24(5.6) + (pitch - (LCW_SQ7_24(6.0)) >> 3);
  }
  else if ( LCW_SQ7_24(5.6) < pitch ) {
    // 1:4
    return LCW_SQ7_24(5.5) + (pitch - (LCW_SQ7_24(5.6)) >> 2);
  }
  else if ( LCW_SQ7_24(5.4) < pitch ) {
    // 1:2
    return LCW_SQ7_24(5.4) + (pitch - (LCW_SQ7_24(5.4)) >> 1);
  }
  else {
    return pitch;
  }
#else
// 2^5.1502 * 440 = 31250 * 0.5(Hz)
  if ( LCW_SQ7_24(5.5) < pitch ) {
    // 1:8
    return LCW_SQ7_24(5.1);// + (pitch - (LCW_SQ7_24(5.5)) >> 3);
  }
  else if ( LCW_SQ7_24(5.1) < pitch ) {
    // 1:4
    return LCW_SQ7_24(5.0) + (pitch - (LCW_SQ7_24(5.1)) >> 2);
  }
  else if ( LCW_SQ7_24(4.9) < pitch ) {
    // 1:2
    return LCW_SQ7_24(4.9) + (pitch - (LCW_SQ7_24(4.9)) >> 1);
  }
  else {
    return pitch;
  }
#endif
}

// pitch : s7.24, return : SQ.22
__fast_inline int32_t myLookUp(
  const LCWOscWaveSource *src, uint32_t t, int32_t pitch)
{
  // AAテーブルを参照するための添字に加工
  const int32_t j0 = (pitch >> (24 - LCW_AA_TABLE_BITS)) - LCW_AA_PITCH_OFFSET;

  // 10bit線形補間の準備
  const uint32_t t0 = t >> (LCW_OSC_TIMER_BITS - (LCW_OSC_TABLE_BITS + 10));

  int32_t i = 0;
  if ( LCW_SQ7_24(2.29) < pitch ) {
    i = 0; // 2151 < freq
  }
  else if ( LCW_SQ7_24(1.64) < pitch ) {
    i = 1; // 1371 < freq
  }
  else if ( LCW_SQ7_24(1.19) < pitch ) {
    i = 2; // 1003 < freq
  }
  else if ( LCW_SQ7_24(0.93) < pitch ) {
    i = 3; // 838 < freq
  }
  else {
    i = 4;
  }

  int64_t out = 0; // = SQ.25 x AA
  {
    {
      const int16_t *p = src->tables[i];
      const int32_t fn = src->factors[i];

      // ほんとはi=0のケースでしか、AAを考慮する必要はない
      int32_t j = j0 + s_oscLutOption[fn - 1].pitch;
      j = ( j0 < 0 ) ? 0 : j0;
      if ( LCW_AA_TABLE_SIZE <= j ) {
        j = LCW_AA_TABLE_SIZE - 1;
      }

      int32_t gain = gLcwAntiAiliasingTable[j];
      uint32_t t1 = t0 >> 10;
      uint32_t t2 = (t1 + 1) & LCW_OSC_TABLE_MASK;

      int32_t ratio = t0 & 0x3FF;
      int32_t val = (p[t1] * ratio) + (p[t2] * (0x400 - ratio));
      out += ( (int64_t)val * gain );
    }
  }

  if ( i < 4 ) {
    for (int32_t k=0; k<6; k++) {
      const int16_t *p = src->tables[0]; // sin wave
      const int32_t fn = src->factors[i + 1] + k;

      int32_t j = j0 + s_oscLutOption[fn - 1].pitch;
      j = ( j < 0 ) ? 0 : j;
      if ( LCW_AA_TABLE_SIZE <= j ) {
        continue;
      }

      int32_t gain = gLcwAntiAiliasingTable[j];
      gain = (gain * s_oscLutOption[fn - 1].gain) >> 10;
      if ( (fn & 0x1) == 0 ) {
        gain = -gain;
      }

      uint32_t t1 = t0 >> 10;
      uint32_t t2 = (t1 + 1) & LCW_OSC_TABLE_MASK;

      int32_t ratio = t0 & 0x3FF;
      int32_t val = (p[t1] * ratio) + (p[t2] * (0x400 - ratio));
      out += ( (int64_t)val * gain );
    }
  }
  else {
    for (int32_t k=0; k<7; k++) {
      const int16_t *p = src->tables[i + 1 + k];
      const int32_t fn = src->factors[i + 1 + k];

      int32_t j = j0 + s_oscLutOption[i - 1].pitch;
      j = ( j < 0 ) ? 0 : j;
      if ( LCW_AA_TABLE_SIZE <= j ) {
        continue;
      }

      int32_t gain = gLcwAntiAiliasingTable[j];
      uint32_t t1 = t0 >> 10;
      uint32_t t2 = (t1 + 1) & LCW_OSC_TABLE_MASK;

      int32_t ratio = t0 & 0x3FF;
      int32_t val = (p[t1] * ratio) + (p[t2] * (0x400 - ratio));
      out += ( (int64_t)val * gain );
    }
  }

  return (int32_t)( out >> (LCW_AA_VALUE_BITS + (25 - 22)) );
}

void OSC_INIT(uint32_t platform, uint32_t api)
{
  s_param.shape = 0.f;
  s_param.shiftshape = 0.f;

  s_state.timer1 = 0;
  s_state.timer2 = 0;
  s_state.pitch1 =
  s_state.pitch2 = (LCW_NOTE_NO_A4 << 24) / 12;
  s_state.shape_lfo = 0;
}

void OSC_CYCLE(const user_osc_param_t * const params,
               int32_t *yn,
               const uint32_t frames)
{
  // s11.20に拡張してから、整数部がoctaveになるように加工
  int32_t pitch1 = (int32_t)params->pitch << 12;
  pitch1 = (pitch1 - (LCW_NOTE_NO_A4 << 20)) / 12;
  const int32_t note = (int32_t)si_roundf(24.f * s_param.shape);
  // [0 .. 24] -> [-12 .. +12]
  int32_t pitch2 = pitch1 + (((note - 12) << 20) / 12);

  int32_t lfo_delta = (params->shape_lfo - s_state.shape_lfo) / (int32_t)frames;

  // s11.20 -> s7.24
  pitch1 <<= 4;
  pitch2 <<= 4;

  // Temporaries.
  uint32_t t1 = s_state.timer1;
  uint32_t t2 = s_state.timer2;
  int32_t shape_lfo = s_state.shape_lfo;

  q31_t * __restrict y = (q31_t *)yn;
  const q31_t * y_e = y + frames;

  // Main Mix/Sub Mix, 8bit(= [0-256])
  const int32_t subVol = (int32_t)(0x100 * s_param.shiftshape);
  const int32_t mainVol = 0x100 - subVol;

  const LCWOscWaveSource *src = &gLcwOscSawSource;
  for (; y != y_e; ) {
    // q22
    int32_t out1 = myLookUp( src, t1, pitch1 );
    int32_t out2 = myLookUp( src, t2, pitch2 );

    // q22 -> q30
    int32_t out = (out1 * mainVol) + (out2 * subVol);

    // q30 -> q31
    *(y++) = (q31_t)(out << (31 - 30));

    // input: s7.24 -> s15.16
    uint32_t dt1 = pitch_to_timer_delta( pitchLimitter(pitch1) >> 8 );
    uint32_t dt2 = pitch_to_timer_delta( pitchLimitter(pitch2) >> 8 );
    t1 = (t1 + dt1) & LCW_OSC_TIMER_MASK;
    t2 = (t2 + dt2) & LCW_OSC_TIMER_MASK;
  
    shape_lfo += lfo_delta;
    pitch2 += (shape_lfo >> 16);
  }

  s_state.timer1 = t1;
  s_state.timer2 = t2;
  s_state.shape_lfo = params->shape_lfo;
  s_state.pitch1 = pitch1;
  s_state.pitch2 = pitch2;
}

void OSC_NOTEON(const user_osc_param_t * const params)
{
  return;
}

void OSC_NOTEOFF(const user_osc_param_t * const params)
{
  return;
}

void OSC_PARAM(uint16_t index, uint16_t value)
{
  float valf = param_val_to_f32( value );
  switch (index) {
  case k_user_osc_param_shape:
    s_param.shape = clip01f( valf );
    break;
  case k_user_osc_param_shiftshape:
    s_param.shiftshape = clip01f( valf );
    break;
  default:
    break;
  }
}
