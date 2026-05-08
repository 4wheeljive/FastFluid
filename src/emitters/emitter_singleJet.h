#pragma once

// ═══════════════════════════════════════════════════════════════════
//  SINGLE JET EMITTER — emitter_singleJet.h
// ═══════════════════════════════════════════════════════════════════
//
//  Injects dye (RGB) and momentum (u,v) at a fixed bottom-center
//  position via a 3-layered Gaussian splat. Designed to be paired
//  with flow_fluid; injects momentum directly into the velocity field.
//
//  Ported from FluidApp.emit_stationary_source() in
//  colorTrailsOrig/navier_stokes_1.py.

#include "fastFluidTypes.h"
#include "modulators.h"
#include "../emitters.h"

namespace fastFluid {
    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    struct SingleJetParams {
        // Density and force are grid-independent (forces, not sizes).
        float jetDensity    = 50.0f;     // dye magnitude (per layer-weighted)
        float jetForce      = 0.25f;     // velocity magnitude
        // Radius and spread scale with grid so plume looks proportional
        // across boards (22→1.8, 32→2.7, 48→4.0, 64→5.3 cells radius).
        float jetRadius     = (float)MIN_DIMENSION / 12.0f;
        float jetSpread     = (float)MIN_DIMENSION / 22.0f;
        float jetAngle      = 0.0f;      // base direction (radians; 0 = straight up)
        float jetHueSpeed   = 0.7f;      // hue rotation rate (Hz)
        // Lateral position swing: plume slides side-to-side along the wall
        // it emits from. Grid-aware max amplitude (22→2.75, 48→6, 64→8 cells).
        // Off by default — modJetSwing.modLevel = 0. Set modLevel > 0 to enable.
        float jetSwingRange = (float)MIN_DIMENSION / 8.0f;

        ModConfig modJetForce = {0, 0.3f, 0.1f};   // modTimer, modRate, modLevel
        ModConfig modAngle    = {1, 0.3f, 2.0f};   // modLevel: 0 = no movement, 2 = full ±90°
        ModConfig modJetSwing = {2, 0.3f, 0.0f};   // modLevel: 0 = no swing, 1 = full range
    };

    static constexpr ModConfig SingleJetParams::* SINGLE_JET_MODS[] = {
        &SingleJetParams::modJetForce,
        &SingleJetParams::modAngle,
        &SingleJetParams::modJetSwing
    };

    SingleJetParams singleJet;

    // Phase 1 of frame: write this component's timer slot ratios.
    static void singleJetPrepareModulators() {
        timings.ratio[singleJet.modJetForce.modTimer] = 0.0004f  * singleJet.modJetForce.modRate;
        timings.ratio[singleJet.modAngle.modTimer]    = 0.00045f * singleJet.modAngle.modRate;
        timings.ratio[singleJet.modJetSwing.modTimer] = 0.0004f  * singleJet.modJetSwing.modRate;
    }

    static void emitSingleJet() {
        const ModConfig& forceMod = singleJet.modJetForce;
        const ModConfig& angleMod = singleJet.modAngle;
        const ModConfig& swingMod = singleJet.modJetSwing;

        // ─── Signal acquisition ────────────────────────────────────
        const float forceSignal = move.normalized_noise[forceMod.modTimer];
        const float angleSignal = move.directional_noise[angleMod.modTimer];
        const float swingSignal = move.directional_noise[swingMod.modTimer];

        // ─── Artistic application ──────────────────────────────────
        // Force: orbitalDots-style bipolar modulation
        const float currentForce = singleJet.jetForce * (1.0f + forceSignal * 0.4f);

        // Angle: noise-based offset around base direction.
        // Coefficient π/4 per modLevel unit → modLevel=2 reaches full ±π/2 (±90°).
        constexpr float ANGLE_SCALE = CT_2PI * 0.125f;   // π/4
        const float angleOffset = angleMod.modLevel * ANGLE_SCALE * angleSignal;

        // Wrap final angle to [0, 2π) for sincos_fast (UB for negative inputs).
        constexpr float INV_2PI = 1.0f / CT_2PI;
        float angle = singleJet.jetAngle + angleOffset;
        angle -= fl::floorf(angle * INV_2PI) * CT_2PI;

        // Direction decomposition: angle 0 = straight up (negative y)
        SinCosResult sc = sincos_fast(angle);
        const float dirX =  sc.sin_val;
        const float dirY = -sc.cos_val;
        const float velX = dirX * currentForce;
        const float velY = dirY * currentForce;

        const float density = singleJet.jetDensity;

        // Per-frame base hue. Each splat call dithers around this in the cell loop.
        const float baseHue = fmodPos(t * singleJet.jetHueSpeed, 1.0f);

        // Lateral position swing: plume slides side-to-side around base position.
        // Independent of angle modulation — both can run simultaneously, or
        // either alone. modLevel=0 disables swing without affecting angle.
        const float swingOffset = swingSignal * swingMod.modLevel * singleJet.jetSwingRange;

        // Jet position: bottom-center, offset horizontally by swing.
        const float jx = (float)WIDTH * 0.5f + swingOffset;
        const float jy = (float)HEIGHT - 2.0f;

        // ─── 3-layered Gaussian splat ──────────────────────────────
        // Each layer is shifted along the jet axis. Offsets scale with
        // jetRadius so plume structure stays proportional to its core
        // size across grid sizes (was fixed 1.2/2.2 cells, calibrated
        // for a small grid).
        const float r = singleJet.jetRadius;
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
        if (singleJet.jetSpread > 0.0f) {
            // Perpendicular to jet axis: rotate (dirX,dirY) by 90°: (-dirY, dirX)
            const float perpX = -dirY;
            const float perpY =  dirX;
            const float side = singleJet.jetSpread;
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

} // namespace fastFluid
