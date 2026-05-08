#pragma once
#include <stdint.h>

enum Emitter : uint8_t {
    EMITTER_SINGLEJET = 0,
    EMITTER_THREEJET,
    EMITTER_COUNT
};

enum Flow : uint8_t {
    FLOW_SMOKE = 0,
    FLOW_COUNT
};

enum Obstacle : uint8_t {
    OBSTACLE_PADDLE = 0,
    OBSTACLE_COUNT
};
