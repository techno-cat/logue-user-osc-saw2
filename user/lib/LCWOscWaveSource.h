/*
Copyright 2019 Tomoaki Itoh
This software is released under the MIT License, see LICENSE.txt.
//*/

#include <stdint.h>

#define LCW_OSC_TABLE_BITS (10)
#define LCW_OSC_TABLE_SIZE (1 << LCW_OSC_TABLE_BITS)
#define LCW_OSC_TABLE_MASK ((LCW_OSC_TABLE_SIZE) - 1)

// SQ.15
typedef int16_t LCWOscWaveTable[LCW_OSC_TABLE_SIZE];

typedef struct {
    int32_t count;
    const LCWOscWaveTable *tables;
    const int16_t *factors;
} LCWOscWaveSource;

extern const LCWOscWaveSource gLcwOscSawSource;

