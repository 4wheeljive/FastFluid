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
        .size = (float)MIN_DIMENSION * 0.1f,
        .direction = 0.0f,
        .density = 25.0f,
        .force = 0.25f,
        .hueSpeed = 0.25f,
        .spread = (float)MIN_DIMENSION * 0.05f,
        .slideRange = (float)WIDTH * 0.5f,
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

    static inline void resolveJetPose(float& anchorCol, float& anchorRow,
                                      float& dirCol, float& dirRow) {
        const ModConfig& directionMod = jet.modDirection;
        const ModConfig& slideMod = jet.modSlideRange;

        const float directionSignal = move.directional_noise[directionMod.modTimer];
        const float slideSignal = move.directional_noise[slideMod.modTimer];

        constexpr float DIRECTION_SCALE = FF_2PI * 0.125f;
        const float directionOffset = directionMod.modLevel * directionSignal * DIRECTION_SCALE;

        constexpr float INV_2PI = 1.0f / FF_2PI;
        float direction = jet.direction + directionOffset;
        direction -= fl::floorf(direction * INV_2PI) * FF_2PI;

        SinCosResult sc = sincos_fast(direction);
        dirCol = sc.sin_val;
        dirRow = -sc.cos_val;

        const float slideOffset = slideSignal * slideMod.modLevel * jet.slideRange;
        anchorCol = (float)WIDTH * 0.5f + slideOffset;
        anchorRow = (float)HEIGHT - 1.0f;
    }

    static void runEmitter() {
        const ModConfig& forceMod = jet.modForce;

        // ─── Signal acquisition ────────────────────────────────────
        const float forceSignal = move.normalized_noise[forceMod.modTimer];

        // ─── Artistic application ──────────────────────────────────
        const float currentForce = jet.force * (1.0f + forceSignal * 0.4f);

        float jx;
        float jy;
        float dirX;
        float dirY;
        resolveJetPose(jx, jy, dirX, dirY);

        const float velX = dirX * currentForce;
        const float velY = dirY * currentForce;

        const float density = jet.density;

        // Per-frame base hue. Each splat call dithers around this in the cell loop.
        const float baseHue = fmodPos(t * jet.hueSpeed, 1.0f);

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
