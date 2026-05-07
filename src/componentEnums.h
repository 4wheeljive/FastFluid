#pragma once
#include <stdint.h>

enum Emitter : uint8_t {
    EMITTER_FLUIDJET = 0,
    EMITTER_THREEJET = 1,
    EMITTER_COUNT
};

enum Flow : uint8_t {
    FLOW_FLUID = 0,
    FLOW_COUNT
};
