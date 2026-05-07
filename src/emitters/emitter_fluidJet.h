#pragma once

// ═══════════════════════════════════════════════════════════════════
//  FLUID JET EMITTER — emitter_fluidJet.h
// ═══════════════════════════════════════════════════════════════════
//
//  Injects dye (RGB) and momentum (u,v) at a fixed bottom-center
//  position via a 3-layered Gaussian splat.  Designed to be paired
//  with flow_fluid; injects momentum directly into the velocity field.
//
//  Ported from FluidApp.emit_stationary_source() in
//  colorTrailsOrig/navier_stokes_1.py.

#include "fluidSimTypes.h"
#include "modulators.h"
#include "../flows/flow_fluid.h"   // u, v arrays + fluidAddVelocity

namespace fluidSim {
    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    struct FluidJetParams {
        // Density and force are grid-independent (forces, not sizes).
        float jetDensity   = 50.0f;     // dye magnitude (per layer-weighted)
        float jetForce     = 0.25f;     // velocity magnitude
        // Radius and spread scale with grid so plume looks proportional
        // across boards (22→1.8, 32→2.7, 48→4.0, 64→5.3 cells radius).
        float jetRadius    = (float)MIN_DIMENSION / 12.0f;
        float jetSpread    = (float)MIN_DIMENSION / 22.0f;
        float jetAngle     = 0.0f;      // base direction (radians; 0 = straight up)
        float jetHueSpeed  = 0.7f;      // hue rotation rate (Hz)

        ModConfig modJetForce = {EMITTER_SLOT_BASE + 0, 0.3f, 0.1f};   // modTimer, modRate, modLevel
        ModConfig modAngle    = {EMITTER_SLOT_BASE + 1, 0.3f, 2.0f};   // modLevel: 0 = no movement, 2 = full ±90°
    };

    FluidJetParams fluidJet;

    // 4x4 Bayer matrix, values normalized to [-0.5, +0.5].
    // Used for per-cell hue dithering to break uint8 banding from single-color splats.
    static const float bayerHueDither[4][4] = {
        { -7.5f / 16.0f, +1.5f / 16.0f, -5.5f / 16.0f, +3.5f / 16.0f },
        { +5.5f / 16.0f, -3.5f / 16.0f, +7.5f / 16.0f, -1.5f / 16.0f },
        { -4.5f / 16.0f, +4.5f / 16.0f, -6.5f / 16.0f, +2.5f / 16.0f },
        { +6.5f / 16.0f, -2.5f / 16.0f, +4.5f / 16.0f, -0.5f / 16.0f }
    };

    // Hue dither magnitude. Max per-cell hue offset = scale * 0.5.
    // 0.002 → ±0.001 hue cycle → ~1.5 RGB units max delta — enough to break bands.
    static constexpr float HUE_DITHER_SCALE = 0.002f;

    // 2D Gaussian splat: writes dye to gR/gG/gB and momentum to u/v.
    // Per-cell hue dither ensures adjacent cells get distinct uint8 values
    // at LED output, breaking the banding inherent to single-color splats.
    static void fluidJetSplat(float cx, float cy, float radius,
                              float densityMag,
                              float velX, float velY) {
        const float r2  = radius * radius * 0.6f;
        const float invR2 = 1.0f / r2;
        int x0 = max(0,           (int)fl::floorf(cx - radius));
        int x1 = min(WIDTH  - 1,  (int)fl::ceilf (cx + radius));
        int y0 = max(0,           (int)fl::floorf(cy - radius));
        int y1 = min(HEIGHT - 1,  (int)fl::ceilf (cy + radius));

        const float densityScale = densityMag * (1.0f / 255.0f);

        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                float dx = (x + 0.5f) - cx;
                float dy = (y + 0.5f) - cy;
                float d2 = dx * dx + dy * dy;
                // exp(-d2/r2) approximated via fastpow(e, -d2/r2)
                float w = fastpow(2.71828183f, -d2 * invR2);
                if (w < 0.005f) continue;

                // Per-cell hue offset from Bayer matrix — deterministic spatial
                // pattern, so the same cell gets the same offset across frames
                // (no temporal flicker).
                const float hueOffset = bayerHueDither[y & 3][x & 3] * HUE_DITHER_SCALE;
                ColorF c = rainbow(t, fluidJet.jetHueSpeed, hueOffset);

                const float wScale = w * densityScale;
                gR[y][x] += c.r * wScale;
                gG[y][x] += c.g * wScale;
                gB[y][x] += c.b * wScale;
                u[y][x]  += velX * w;
                v[y][x]  += velY * w;
            }
        }
    }

    // Phase 1 of frame: write this component's timer slot ratios.
    static void emitterPrepareModulators() {
        timings.ratio[fluidJet.modJetForce.modTimer] = 0.0004f  * fluidJet.modJetForce.modRate;
        timings.ratio[fluidJet.modAngle.modTimer]    = 0.00045f * fluidJet.modAngle.modRate;
    }

    static void emitFluidJet() {
        const ModConfig& forceMod = fluidJet.modJetForce;
        const ModConfig& angleMod = fluidJet.modAngle;

        // ─── 2) Signal acquisition ─────────────────────────────────
        //const float forceSignal = move.directional_noise[forceMod.modTimer];
        const float forceSignal = move.normalized_noise[forceMod.modTimer];
        const float angleSignal = move.directional_noise[angleMod.modTimer];

        // ─── 3) Artistic application ───────────────────────────────
        // Force: orbitalDots-style bipolar modulation
        const float currentForce = fluidJet.jetForce * (1.0f + forceSignal * 0.4f);
            //((1.0f - forceMod.modLevel) + forceMod.modLevel * forceSignal);

        // Angle: noise-based offset around base direction.
        // Coefficient π/4 per modLevel unit → modLevel=2 reaches full ±π/2 (±90°).
        constexpr float ANGLE_SCALE = CT_2PI * 0.125f;   // π/4
        const float angleOffset = angleMod.modLevel * ANGLE_SCALE * angleSignal;

        // Wrap final angle to [0, 2π) for sincos_fast (UB for negative inputs).
        constexpr float INV_2PI = 1.0f / CT_2PI;
        float angle = fluidJet.jetAngle + angleOffset;
        angle -= fl::floorf(angle * INV_2PI) * CT_2PI;

        // Direction decomposition: angle 0 = straight up (negative y)
        SinCosResult sc = sincos_fast(angle);
        const float dirX =  sc.sin_val;
        const float dirY = -sc.cos_val;
        const float velX = dirX * currentForce;
        const float velY = dirY * currentForce;

        // Color is now computed per-cell inside the splat (with hue dither),
        // so we just pass a per-layer density magnitude.
        const float density = fluidJet.jetDensity;

        // Jet position: bottom-center
        const float jx = (float)WIDTH * 0.5f;
        const float jy = (float)HEIGHT - 3.0f;

        // ─── 4) 3-layered Gaussian splat ──────────────────────────
        // Each layer is shifted along the jet axis. Offsets scale with
        // jetRadius so plume structure stays proportional to its core
        // size across grid sizes (was fixed 1.2/2.2 cells, calibrated
        // for a small grid).
        const float r = fluidJet.jetRadius;
        const float layerMid  = r * 0.6f;   // ns_3 had 1.2 at r≈2 → 0.6 * r
        const float layerOut  = r * 1.1f;   // ns_3 had 2.2 at r≈2 → 1.1 * r
        // Core layer: 55% density, 100% velocity
        fluidJetSplat(jx, jy, r,
                      density * 0.55f,
                      velX,         velY);
        // Middle layer: 30% density, 82% velocity, shifted along jet
        fluidJetSplat(jx + dirX * layerMid, jy + dirY * layerMid, r,
                      density * 0.30f,
                      velX * 0.82f, velY * 0.82f);
        // Outer layer: 15% density, 65% velocity, shifted further
        fluidJetSplat(jx + dirX * layerOut, jy + dirY * layerOut, r,
                      density * 0.15f,
                      velX * 0.65f, velY * 0.65f);

        // ─── 5) Side injections (lateral push outward) ────────────
        if (fluidJet.jetSpread > 0.0f) {
            // Perpendicular to jet axis: rotate (dirX,dirY) by 90°: (-dirY, dirX)
            const float perpX = -dirY;
            const float perpY =  dirX;
            const float side = fluidJet.jetSpread;
            // Side offset also scales with jetRadius (was fixed 1.5).
            const float sideOff = r * 0.75f;
            // Left side: push left (negative perp)
            fluidJetSplat(jx - perpX * sideOff, jy - perpY * sideOff, r * 0.7f,
                          density * 0.15f,
                          -perpX * side * 0.35f, -perpY * side * 0.35f);
            // Right side: push right (positive perp)
            fluidJetSplat(jx + perpX * sideOff, jy + perpY * sideOff, r * 0.7f,
                          density * 0.15f,
                           perpX * side * 0.35f,  perpY * side * 0.35f);
        }
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace fluidSim
