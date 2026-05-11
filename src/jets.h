#pragma once

// ============================================================================
//  JETS - shared multi-jet data model
// ============================================================================

#include "fastFluidTypes.h"

namespace fastFluid {

    enum MultiJetLayoutMode : uint8_t {
        MULTIJET_LAYOUT_RING_EVEN = 0,
        MULTIJET_LAYOUT_RING_CUSTOM = 1,
        MULTIJET_LAYOUT_FREE = 2
    };

    enum MultiJetDirectionMode : uint8_t {
        MULTIJET_DIR_RADIAL_OUT = 0,
        MULTIJET_DIR_RADIAL_IN = 1,
        MULTIJET_DIR_TANGENT_CW = 2,
        MULTIJET_DIR_TANGENT_CCW = 3,
        MULTIJET_DIR_ABSOLUTE = 4,
        MULTIJET_DIR_AIM_CENTER = 5
    };

    enum MultiJetColorMode : uint8_t {
        MULTIJET_COLOR_HUE_SPREAD = 0,
        MULTIJET_COLOR_NEAR_OPPOSITE = 1,
        MULTIJET_COLOR_WARM_ABSOLUTE = 2,
        MULTIJET_COLOR_PER_JET = 3
    };

    struct JetParams {
        bool enabled = true;

        // Physical placement.
        float radialAngleOffset = 0.0f;   // radians, added to pack radialAngle
        float anchorRadiusScale = 1.0f;   // multiplies pack ringRadius
        float anchorCol = (float)WIDTH * 0.5f;
        float anchorRow = (float)HEIGHT * 0.5f;

        // Direction/color offsets.
        float directionAngle = 0.0f;      // absolute mode: 0 = up
        float directionOffset = 0.0f;     // radians, added after base direction
        float hueOffset = 0.0f;           // hue cycles

        // Static per-jet scaling.
        float jetRadiusScale = 1.0f;
        float densityScale = 1.0f;
        float forceScale = 1.0f;
        float hueSpeedScale = 1.0f;

        // Per-jet modulation strength. Values near 1 keep jets synchronized;
        // small differences create related but non-identical motion.
        float radialAngleModScale = 1.0f;
        float anchorRadiusModScale = 1.0f;
        float wobbleModScale = 1.0f;
        float directionModScale = 1.0f;
        float jetRadiusModScale = 1.0f;
        float densityModScale = 1.0f;
        float forceModScale = 1.0f;
        float hueModScale = 1.0f;

        // Phase offsets sample the same noise stream at different points.
        // This is the cheap synchrony/asynchrony control.
        float radialAnglePhase = 0.0f;
        float anchorRadiusPhase = 0.0f;
        float wobblePhase = 0.0f;
        float directionPhase = 0.0f;
        float jetRadiusPhase = 0.0f;
        float densityPhase = 0.0f;
        float forcePhase = 0.0f;
        float huePhase = 0.0f;
    };

    struct JetPack {
        uint8_t numJets = 3;
        uint8_t layoutMode = MULTIJET_LAYOUT_RING_EVEN;
        uint8_t directionMode = MULTIJET_DIR_RADIAL_OUT;
        uint8_t colorMode = MULTIJET_COLOR_HUE_SPREAD;

        float centerCol = (float)WIDTH * 0.5f;
        float centerRow = (float)HEIGHT * 0.5f;

        float radialAngle = CT_PI * 0.5f;        // 90 deg starts at bottom
        float ringRadius = (float)MIN_DIMENSION * 0.25f;
        float jetRadius = (float)MIN_DIMENSION / 12.0f;
        float wobble = 0.0f;
        float directionAngle = 0.0f;
        float density = 50.0f;
        float force = 0.7f;
        float hueSpeed = 0.25f;
        float hueSpread = 1.0f;

        // Modulation amplitudes. Angle fields are radians; scalar fields are
        // fractional depth around the configured base value.
        float varianceRadialAngle = 0.0f;
        float varianceRingRadius = 0.50f;
        float varianceWobble = 0.0f;
        float varianceDirection = CT_PI;
        float varianceJetRadius = 0.0f;
        float varianceDensity = 0.0f;
        float varianceForce = 0.85f;
        float varianceHue = 0.0f;

        ModConfig modRadialAngle = {0, 0.3f, 0.0f};
        ModConfig modRingRadius = {0, 0.3f, 1.0f};
        ModConfig modWobble = {0, 0.3f, 0.0f};
        ModConfig modDirection = {0, 0.3f, 1.0f};
        ModConfig modJetRadius = {0, 0.3f, 0.0f};
        ModConfig modDensity = {0, 0.3f, 0.0f};
        ModConfig modForce = {0, 0.3f, 1.0f};
        ModConfig modHue = {0, 0.3f, 0.0f};
    };

} // namespace fastFluid
