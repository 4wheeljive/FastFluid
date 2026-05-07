#pragma once

// ═══════════════════════════════════════════════════════════════════
//  THREE-JET EMITTER — emitter_threeJet.h
// ═══════════════════════════════════════════════════════════════════
//
//  Three jets evenly spaced on a virtual ring around the grid center,
//  each rotating freely around its anchor via independent noise. Color
//  per jet is determined by ColorMode (TRIPLE / DOUBLE / ORANGE).
//
//  Anchor angles: 90° (bottom), 210° (top-left), 330° (top-right).
//  Jets point radially outward from the ring center by default; the
//  per-jet angle modulator rotates them around the anchor point.
//
//  Ported from colorTrailsOrig/solver_debug_views.py — three jets
//  with full-360° angle wandering + per-jet hue offsets.

#include "fluidSimTypes.h"
#include "modulators.h"
#include "../emitters.h"   // shared fluidJetSplat()

namespace fluidSim {
    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    // Color modes for three-jet hue assignment.
    //   TRIPLE: per-jet offsets at 0, 1/3, 2/3 (relative to time-rotating base hue)
    //   DOUBLE: per-jet offsets at 0, 170/360, 190/360 (relative; "near-opposite" pair)
    //   ORANGE: per-jet absolute hues at 0/360, 32/360, 54/360 (red/orange/yellow,
    //           do NOT rotate over time)
    enum ThreeJetColorMode : uint8_t {
        THREEJET_COLOR_TRIPLE = 0,
        THREEJET_COLOR_DOUBLE = 1,
        THREEJET_COLOR_ORANGE = 2
    };

    struct ThreeJetParams {
        // Shared across all 3 jets (sizes / forces / hue).
        float density       = 50.0f;
        float force         = 0.7f;       // base velocity magnitude per jet
        float radius        = (float)MIN_DIMENSION / 12.0f;
        float hueSpeed      = 0.25f;      // base hue rotation rate (Hz)
        // Ring radius — distance from grid center to each jet anchor, in cells.
        // Defaults to 25% of MIN_DIMENSION (matches solver_debug_views.py: GRID_SIZE * 0.25).
        float ringRadius    = (float)MIN_DIMENSION * 0.25f;

        // Color mode selects per-jet hue offset scheme.
        uint8_t colorMode = THREEJET_COLOR_TRIPLE;

        // Per-jet angle modulators — independent noise drives each jet's
        // rotation around its ring anchor point.
        ModConfig modJet0Angle = {EMITTER_SLOT_BASE + 0, 0.3f, 1.0f};
        ModConfig modJet1Angle = {EMITTER_SLOT_BASE + 1, 0.3f, 1.0f};
        ModConfig modJet2Angle = {EMITTER_SLOT_BASE + 2, 0.3f, 1.0f};
    };

    ThreeJetParams threeJet;

    // Phase 1 of frame: write this component's timer slot ratios.
    static void threeJetPrepareModulators() {
        timings.ratio[threeJet.modJet0Angle.modTimer] = 0.00040f * threeJet.modJet0Angle.modRate;
        timings.ratio[threeJet.modJet1Angle.modTimer] = 0.00045f * threeJet.modJet1Angle.modRate;
        timings.ratio[threeJet.modJet2Angle.modTimer] = 0.00050f * threeJet.modJet2Angle.modRate;
    }

    // Compute per-jet hue based on color mode.
    //   jetIndex: 0/1/2
    //   baseHue:  current time-rotating hue (already `t * hueSpeed` mod 1)
    static inline float threeJetHueForJet(uint8_t jetIndex, float baseHue) {
        switch (threeJet.colorMode) {
            case THREEJET_COLOR_DOUBLE: {
                static const float offsets[3] = { 0.0f, 170.0f / 360.0f, 190.0f / 360.0f };
                return fmodPos(baseHue + offsets[jetIndex], 1.0f);
            }
            case THREEJET_COLOR_ORANGE: {
                // Absolute hues — do NOT rotate with base.
                static const float absHues[3] = { 0.0f / 360.0f, 32.0f / 360.0f, 54.0f / 360.0f };
                return absHues[jetIndex];
            }
            case THREEJET_COLOR_TRIPLE:
            default: {
                static const float offsets[3] = { 0.0f, 1.0f / 3.0f, 2.0f / 3.0f };
                return fmodPos(baseHue + offsets[jetIndex], 1.0f);
            }
        }
    }

    // Emit one jet at a ring anchor point. The jet points radially OUTWARD
    // by default; the angle modulator rotates that direction around its anchor.
    static void emitOneRingJet(float anchorRow, float anchorCol,
                               float angleSignal, float baseHue,
                               float density, float force, float radius) {
        // Outward direction from grid center to anchor.
        const float centerRow = (float)HEIGHT * 0.5f;
        const float centerCol = (float)WIDTH  * 0.5f;
        float outRow = anchorRow - centerRow;
        float outCol = anchorCol - centerCol;
        const float baseLength = fl::sqrtf(outRow * outRow + outCol * outCol);
        if (baseLength <= 1e-6f) return;
        outRow /= baseLength;
        outCol /= baseLength;

        // Rotate the outward direction by the per-jet noise angle (full 360°).
        // angleSignal is bipolar [-1, +1] from directional_noise.
        const float angleRad = angleSignal * CT_2PI * 0.5f;   // ±π = full circle wander
        SinCosResult sc = sincos_fast(angleRad < 0.0f ? angleRad + CT_2PI : angleRad);
        const float velCol = outCol * sc.cos_val - outRow * sc.sin_val;
        const float velRow = outCol * sc.sin_val + outRow * sc.cos_val;
        const float velX = velCol * force;
        const float velY = velRow * force;

        // Three-layer splat (same structure as fluidJet plume).
        const float r = radius;
        const float layerMid = r * 0.6f;
        const float layerOut = r * 1.1f;
        // Core
        fluidJetSplat(anchorCol, anchorRow, r,
                      density * 0.55f,
                      velX, velY, baseHue);
        // Middle layer (shifted along jet's velocity direction)
        fluidJetSplat(anchorCol + velCol * layerMid, anchorRow + velRow * layerMid, r,
                      density * 0.30f,
                      velX * 0.82f, velY * 0.82f, baseHue);
        // Outer layer
        fluidJetSplat(anchorCol + velCol * layerOut, anchorRow + velRow * layerOut, r,
                      density * 0.15f,
                      velX * 0.65f, velY * 0.65f, baseHue);
    }

    static void emitThreeJet() {
        // ─── Signal acquisition ────────────────────────────────────
        const float angle0 = move.directional_noise[threeJet.modJet0Angle.modTimer];
        const float angle1 = move.directional_noise[threeJet.modJet1Angle.modTimer];
        const float angle2 = move.directional_noise[threeJet.modJet2Angle.modTimer];

        // ─── Ring geometry ─────────────────────────────────────────
        // Three anchors evenly spaced on a circle. Anchor angles match
        // solver_debug_views.py: 90° (bottom), 210° (top-left), 330° (top-right).
        const float centerRow = (float)HEIGHT * 0.5f;
        const float centerCol = (float)WIDTH  * 0.5f;
        const float ringR     = threeJet.ringRadius;

        // Pre-computed sin/cos of 90°, 210°, 330° (in screen coords:
        // sin = vertical from center, cos = horizontal from center).
        // 90° → row = +1*r, col = 0     (bottom)
        // 210° → row = -0.5*r, col = -0.866*r  (top-left)
        // 330° → row = -0.5*r, col = +0.866*r  (top-right)
        const float bottomRow    = centerRow + 1.0f       * ringR;
        const float bottomCol    = centerCol + 0.0f       * ringR;
        const float topLeftRow   = centerRow + (-0.5f)    * ringR;
        const float topLeftCol   = centerCol + (-0.86603f)* ringR;
        const float topRightRow  = centerRow + (-0.5f)    * ringR;
        const float topRightCol  = centerCol + (+0.86603f)* ringR;

        const float baseHue = fmodPos(t * threeJet.hueSpeed, 1.0f);
        const float density = threeJet.density;
        const float force   = threeJet.force;
        const float radius  = threeJet.radius;

        // Per-jet hue (depends on color mode).
        const float hue0 = threeJetHueForJet(0, baseHue);
        const float hue1 = threeJetHueForJet(1, baseHue);
        const float hue2 = threeJetHueForJet(2, baseHue);

        // Scale angle noise by modLevel — modLevel=0 disables rotation, jet
        // points purely outward; modLevel=1 lets it wander a full ±π.
        emitOneRingJet(bottomRow,   bottomCol,
                       angle0 * threeJet.modJet0Angle.modLevel, hue0,
                       density, force, radius);
        emitOneRingJet(topLeftRow,  topLeftCol,
                       angle1 * threeJet.modJet1Angle.modLevel, hue1,
                       density, force, radius);
        emitOneRingJet(topRightRow, topRightCol,
                       angle2 * threeJet.modJet2Angle.modLevel, hue2,
                       density, force, radius);
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace fluidSim
