#pragma once

// ═══════════════════════════════════════════════════════════════════
//  PADDLES OBSTACLE — obstacle_paddles.h
// ═══════════════════════════════════════════════════════════════════
//
//  Paddle-pair obstacle: two solid horizontal segments at the vertical
//  midline with a centered gap. Slides side-to-side via Perlin-noise
//  modulation (modSlide) for organic motion.
//
//  Self-contained — owns PaddleParams (all paddle-specific user knobs)
//  and generatePaddleObstacle(). Uses the generic state arrays and
//  apply functions defined in obstacles.h.
//
//  Must be included AFTER obstacles.h.
//
//  Ported from colorTrailsOrig/navier_stokes_3.py (set_obstacle +
//  update_obstacle), with sin-driven swing replaced by noise-driven
//  slide.

#include "../obstacles.h"
#include "../modulators.h"

namespace fluidSim {
    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    static constexpr int PADDLE_THICKNESS = 2;   // matches ns_3 OBSTACLE_THICKNESS

    struct PaddleParams {
        // Common bits (mirrored into obstacleCommon each frame).
        bool  enable     = true;
        bool  overlay    = true;
        float colorR     = 220.0f;    // overlay color (0..255)
        float colorG     = 220.0f;
        float colorB     = 240.0f;

        // Paddle-specific.
        // Width scales with grid (~55% of WIDTH): 22→12, 38→21, 48→26, 64→35.
        float width      = (float)WIDTH * 0.55f;
        float softEdge   = 0.22f;     // inner-corner soft-mask intensity

        // Sliding motion: noise-driven horizontal position.
        //   modRate  → how fast the noise evolves (slide pace)
        //   modLevel → amplitude as fraction of travel range (0=center, 1=full)
        ModConfig modSlide = {OBSTACLE_SLOT_BASE + 0, 0.3f, 0.85f};   // {modTimer, modRate, modLevel}
    };

    PaddleParams paddles;

    static inline int clampi(int v, int lo, int hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    // Phase 1 of frame: write this component's timer slot ratios.
    // Always writes (regardless of paddles.enable) so the slot is in a
    // known state — cost is one slot's noise calc that nobody reads when
    // disabled. Trivial.
    static void paddlesPrepareModulators() {
        timings.ratio[paddles.modSlide.modTimer] = 0.0004f * paddles.modSlide.modRate;
    }

    // Generate paddle geometry into the generic obstacle state arrays.
    // t is animation time (already scaled by globalSpeed).
    static void generatePaddleObstacle(float t) {
        // Mirror common bits into obstacleCommon so generic apply
        // functions can read them.
        obstacleCommon.enable  = paddles.enable;
        obstacleCommon.overlay = paddles.overlay;
        obstacleCommon.colorR  = paddles.colorR;
        obstacleCommon.colorG  = paddles.colorG;
        obstacleCommon.colorB  = paddles.colorB;

        // Clear mask state for this frame.
        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                obstacleMaskArr[y][xc] = false;
                obstacleSoftMaskArr[y][xc] = 0.0f;
            }
        }
        obstacleSegmentCount = 0;
        obstacleHas = false;

        // Disabled? Skip generation entirely; apply functions become no-ops
        // because obstacleHas stays false.
        if (!paddles.enable) return;

        // ─── Modulator-driven slide ────────────────────────────────
        // Perlin-noise drives a smooth, organic horizontal position.
        // Ratio was written by paddlesPrepareModulators(); calculate_modulators
        // already ran at the engine level — we just read the output here.
        const ModConfig& slideMod = paddles.modSlide;
        const float slideSignal = move.directional_noise[slideMod.modTimer];   // [-1, +1]

        // ─── Geometry (float-precision throughout) ─────────────────
        int width = (int)(paddles.width + 0.5f);
        if (width < 1)     width = 1;
        if (width > WIDTH) width = WIDTH;

        const int rowCenter = HEIGHT / 2;
        const int thickness = PADDLE_THICKNESS;

        // Travel keeps the paddle clear of the side walls by ≥2 cells.
        const float travel = (float)((WIDTH - width - 4) / 2);
        const float travelClamped = (travel < 0.0f) ? 0.0f : travel;

        // Float center column. Single quantization at the mask-write step.
        const float centerColF = (float)WIDTH * 0.5f
                               + slideSignal * slideMod.modLevel * travelClamped;
        const int centerCol = (int)fl::floorf(centerColF + 0.5f);

        const int halfWidth     = width / 2;
        const int halfThickness = thickness / 2;
        const int row0 = clampi(rowCenter - halfThickness, 0, HEIGHT - 1);
        const int row1 = clampi(rowCenter + (thickness - halfThickness - 1 < 0
                                             ? 0
                                             : thickness - halfThickness - 1),
                                0, HEIGHT - 1);
        const int col0 = clampi(centerCol - halfWidth, 0, WIDTH - 1);
        const int col1 = clampi(centerCol + (width - halfWidth - 1 < 0
                                             ? 0
                                             : width - halfWidth - 1),
                                0, WIDTH - 1);

        int gapWidth = (int)((float)width / 4.0f + 0.5f);
        if (gapWidth < 1) gapWidth = 1;
        const int solidTotal = (width - gapWidth < 0) ? 0 : width - gapWidth;
        const int leftWidth  = solidTotal / 2;
        const int rightWidth = solidTotal - leftWidth;
        const int gapStart = col0 + leftWidth;
        const int gapEnd   = (col1 < gapStart + gapWidth - 1)
                           ? col1
                           : gapStart + gapWidth - 1;

        // Left segment.
        if (leftWidth > 0) {
            const int leftCol1 = (col1 < gapStart - 1) ? col1 : gapStart - 1;
            for (int y = row0; y <= row1; y++) {
                for (int xc = col0; xc <= leftCol1; xc++) {
                    obstacleMaskArr[y][xc] = true;
                }
            }
            obstacleSegments[obstacleSegmentCount][0] = row0;
            obstacleSegments[obstacleSegmentCount][1] = row1;
            obstacleSegments[obstacleSegmentCount][2] = col0;
            obstacleSegments[obstacleSegmentCount][3] = leftCol1;
            obstacleSegmentCount++;
        }
        // Right segment.
        if (rightWidth > 0) {
            const int rightCol0 = (col0 > gapEnd + 1) ? col0 : gapEnd + 1;
            for (int y = row0; y <= row1; y++) {
                for (int xc = rightCol0; xc <= col1; xc++) {
                    obstacleMaskArr[y][xc] = true;
                }
            }
            obstacleSegments[obstacleSegmentCount][0] = row0;
            obstacleSegments[obstacleSegmentCount][1] = row1;
            obstacleSegments[obstacleSegmentCount][2] = rightCol0;
            obstacleSegments[obstacleSegmentCount][3] = col1;
            obstacleSegmentCount++;
        }

        // Soft mask along the inner gap edges. ns_3 uses 0.18 within the
        // segment rows and 0.22 in the row above/below; we scale to softEdge
        // (which the user tunes) keeping the same relative weighting.
        const float softInner = paddles.softEdge * (0.18f / 0.22f);
        const float softOuter = paddles.softEdge;
        const int innerRow0 = (row0 - 1 < 0)         ? 0          : row0 - 1;
        const int innerRow1 = (row1 + 1 >= HEIGHT)   ? HEIGHT - 1 : row1 + 1;

        if (leftWidth > 0) {
            const int innerLeftCol = gapStart - 1;
            if (innerLeftCol >= 0 && innerLeftCol < WIDTH) {
                for (int y = row0; y <= row1; y++) {
                    if (obstacleSoftMaskArr[y][innerLeftCol] < softInner)
                        obstacleSoftMaskArr[y][innerLeftCol] = softInner;
                }
                for (int y = innerRow0; y <= innerRow1; y++) {
                    if (obstacleSoftMaskArr[y][innerLeftCol] < softOuter)
                        obstacleSoftMaskArr[y][innerLeftCol] = softOuter;
                }
            }
        }
        if (rightWidth > 0) {
            const int innerRightCol = gapEnd + 1;
            if (innerRightCol >= 0 && innerRightCol < WIDTH) {
                for (int y = row0; y <= row1; y++) {
                    if (obstacleSoftMaskArr[y][innerRightCol] < softInner)
                        obstacleSoftMaskArr[y][innerRightCol] = softInner;
                }
                for (int y = innerRow0; y <= innerRow1; y++) {
                    if (obstacleSoftMaskArr[y][innerRightCol] < softOuter)
                        obstacleSoftMaskArr[y][innerRightCol] = softOuter;
                }
            }
        }

        obstacleBounds[0] = row0;
        obstacleBounds[1] = row1;
        obstacleBounds[2] = col0;
        obstacleBounds[3] = col1;
        obstacleHas = (obstacleSegmentCount > 0);
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace fluidSim
