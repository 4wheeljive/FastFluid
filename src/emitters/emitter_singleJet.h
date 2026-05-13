#pragma once

// ═══════════════════════════════════════════════════════════════════
//  SINGLE JET EMITTER — emitter_jet.h
// ═══════════════════════════════════════════════════════════════════
//  Injects dye (RGB) and momentum (u,v) directly into the velocity field
//  at a fixed bottom-center or sliding bottom position via a 3-layered Gaussian splat.
//
//  Ported from FluidApp.emit_stationary_source() in
//  colorTrailsOrig/navier_stokes_1.py.

#include "fastFluidTypes.h"
#include "modulators.h"
#include "jets.h"
#include "../emitters.h"


namespace fastFluid {
namespace singleJet {

    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    JetParams jet = {
        .radius = (float)MIN_DIMENSION * 0.25,
        .size = (float)MIN_DIMENSION / 12.0f,
        .direction = 0.0f,
        .density = 30.0f,
        .force = 0.25f,
        .hueSpeed = 0.7f,
        .spread = (float)MIN_DIMENSION / 22.0f,
        .slideRange = (float)MIN_DIMENSION / 8.0f,
        .modDirection = {1, 0.3f, 2.0f},
        .modForce = {0, 0.3f, 0.1f},
        .modSlideRange = {2, 0.3f, 0.0f}
    };

    static constexpr ModConfig JetParams::* SINGLE_JET_MODS[] = {
        &JetParams::modForce,
        &JetParams::modDirection,
        &JetParams::modSlideRange
    };



    // Phase 1 of frame: write this component's timer slot ratios.
    static void prepEmitterMods() {
        timings.ratio[jet.modForce.modTimer] = 0.0004f  * jet.modForce.modRate;
        timings.ratio[jet.modDirection.modTimer]    = 0.00045f * jet.modDirection.modRate;
        timings.ratio[jet.modSlideRange.modTimer] = 0.0004f  * jet.modSlideRange.modRate;
    }

    static void runEmitter() {
        const ModConfig& directionMod = jet.modDirection;
        const ModConfig& forceMod = jet.modForce;
        const ModConfig& slideMod = jet.modSlideRange;

        // ─── Signal acquisition ────────────────────────────────────
        const float forceSignal = move.normalized_noise[forceMod.modTimer];
        const float directionSignal = move.directional_noise[directionMod.modTimer];
        const float slideSignal = move.directional_noise[slideMod.modTimer];

        // ─── Artistic application ──────────────────────────────────
        const float currentForce = jet.force * (1.0f + forceSignal * 0.4f);

        // Direction: noise-based offset around base direction.
        // Coefficient π/4 per modLevel unit → modLevel=2 reaches full ±π/2 (±90°).
        constexpr float DIRECTION_SCALE = FF_2PI * 0.125f;   // π/4
        const float directionOffset = directionMod.modLevel * DIRECTION_SCALE * directionSignal;

        // Wrap final angle to [0, 2π) for sincos_fast (UB for negative inputs).
        constexpr float INV_2PI = 1.0f / FF_2PI;
        float direction = jet.direction + directionOffset;
        direction -= fl::floorf(direction * INV_2PI) * FF_2PI;

        // Direction decomposition: angle 0 = straight up (negative y)
        SinCosResult sc = sincos_fast(direction);
        const float dirX =  sc.sin_val;
        const float dirY = -sc.cos_val;
        const float velX = dirX * currentForce;
        const float velY = dirY * currentForce;

        const float density = jet.density;

        // Per-frame base hue. Each splat call dithers around this in the cell loop.
        const float baseHue = fmodPos(t * jet.hueSpeed, 1.0f);

        // Lateral position slide: plume slides side-to-side around base position.
        // Independent of direction modulation — both can run simultaneously, or
        // either alone. modLevel=0 disables slide without affecting direction.
        const float slideOffset = slideSignal * slideMod.modLevel * jet.slideRange;

        // Jet position: bottom-center, offset horizontally by slide.
        const float jx = (float)WIDTH * 0.5f + slideOffset;
        const float jy = (float)HEIGHT - 2.0f;

        // ─── 3-layered Gaussian splat ──────────────────────────────
        // Each layer is shifted along the jet axis. Offsets scale with
        // size so plume structure stays proportional to its core
        // size across grid sizes (was fixed 1.2/2.2 cells, calibrated
        // for a small grid).
        const float r = jet.size;
        const float layerMid  = r * 0.6f;
        const float layerOut  = r * 1.1f;
        // Core layer: 55% density, 100% velocity
        jetSplat(jx, jy, r,
                      density * 0.55f,
                      velX,         velY,
                      baseHue);
        // Middle layer: 30% density, 82% velocity, shifted along jet
        jetSplat(jx + dirX * layerMid, jy + dirY * layerMid, r,
                      density * 0.30f,
                      velX * 0.82f, velY * 0.82f,
                      baseHue);
        // Outer layer: 15% density, 65% velocity, shifted further
        jetSplat(jx + dirX * layerOut, jy + dirY * layerOut, r,
                      density * 0.15f,
                      velX * 0.65f, velY * 0.65f,
                      baseHue);

        // ─── Side injections (lateral push outward) ────────────────
        if (jet.spread > 0.0f) {
            // Perpendicular to jet axis: rotate (dirX,dirY) by 90°: (-dirY, dirX)
            const float perpX = -dirY;
            const float perpY =  dirX;
            const float side = jet.spread;
            const float sideOff = r * 0.75f;
            // Left side: push left (negative perp)
            jetSplat(jx - perpX * sideOff, jy - perpY * sideOff, r * 0.7f,
                          density * 0.15f,
                          -perpX * side * 0.35f, -perpY * side * 0.35f,
                          baseHue);
            // Right side: push right (positive perp)
            jetSplat(jx + perpX * sideOff, jy + perpY * sideOff, r * 0.7f,
                          density * 0.15f,
                           perpX * side * 0.35f,  perpY * side * 0.35f,
                          baseHue);
        }
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace singleJet
} // namespace fastFluid
