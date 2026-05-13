#pragma once

// ═══════════════════════════════════════════════════════════════════
//  OBSTACLES — obstacles.h
// ═══════════════════════════════════════════════════════════════════
//
//  Generic obstacle infrastructure shared by all obstacle generators.
//  An "obstacle" is a set of axis-aligned solid-cell rectangles that
//  fluid velocity and dye must flow around.
//
//  The mask + segments + apply functions defined here are
//  generator-agnostic — any specific obstacle file (obstacles/
//  obstacle_paddles.h, future obstacle_bitmap.h, etc.) populates the
//  mask each frame, then the engine calls the apply functions to
//  enforce the boundary conditions.
//
//  Per-generator user-facing params (paddle width, slide, color, etc.)
//  live in the specific obstacle_*.h file. The generator copies the
//  small common subset (enable / overlay / overlay color) into
//  obstacleCommon each frame so the generic apply functions can read
//  them without knowing which generator owns the geometry.
//
//  This header references the velocity/dye fields (u, v, uPrev, vPrev,
//  gR/gG/gB, tR/tG/tB) defined in flow_fluid.h / fastFluidTypes.h, so
//  it must be included AFTER flow_fluid.h.
//
//  Ported from colorTrailsOrig/navier_stokes_3.py
//  (apply_obstacle_boundary).

#include "flows/flow_smoke.h"

namespace fastFluid {
    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    // ───────────────────────────────────────────────────────────────
    //  Generic state — populated by the active obstacle generator
    //  each frame, read by applyObstacle* functions below.
    // ───────────────────────────────────────────────────────────────

    static bool  obstacleMaskArr[HEIGHT][WIDTH];
    static float obstacleSoftMaskArr[HEIGHT][WIDTH];

    // Up to 2 axis-aligned solid rectangles. Each: {row0, row1, col0, col1}.
    // (Two is enough for a paddle pair with a centered gap; future bitmap
    //  obstacles may want more, in which case bump the first dimension.)
    static int   obstacleSegments[2][4];
    static int   obstacleSegmentCount = 0;
    static int   obstacleBounds[4] = {0, 0, 0, 0};   // {row0, row1, col0, col1}
    bool         obstacleHas = false;                // matches `extern` decl in flow_fluid.h

    // Cross-generator knobs — the small set of common bits any obstacle
    // has. Each generator mirrors its own user-facing values into here
    // each frame; the apply functions read them without knowing which
    // generator owns them.
    struct ObstacleCommonParams {
        bool  enable  = true;
        bool  overlay = true;
        float colorR  = 220.0f;
        float colorG  = 220.0f;
        float colorB  = 220.0f;
    };
    ObstacleCommonParams obstacleCommon;

    // ───────────────────────────────────────────────────────────────
    //  Solver hook: zero velocity inside solid cells, clamp velocity
    //  at perimeter cells to prevent flow into the obstacle, then
    //  apply soft-mask blend across the bbox. Mirrors ns_3
    //  apply_obstacle_boundary.
    //
    //  C++ axes: u = horizontal (x, +right), v = vertical (y, +down).
    //  ns_3 uses opposite axis convention (u=row, v=col); the per-axis
    //  assignments below are the C++ equivalent.
    // ───────────────────────────────────────────────────────────────
    static void applyObstacleVelocity() {
        if (!obstacleCommon.enable || !obstacleHas) return;

        // Hard stop inside each solid segment.
        for (int s = 0; s < obstacleSegmentCount; s++) {
            const int r0 = obstacleSegments[s][0];
            const int r1 = obstacleSegments[s][1];
            const int c0 = obstacleSegments[s][2];
            const int c1 = obstacleSegments[s][3];

            for (int y = r0; y <= r1; y++) {
                for (int xc = c0; xc <= c1; xc++) {
                    if (!obstacleMaskArr[y][xc]) continue;
                    smoke::u[y][xc]     = 0.0f;
                    smoke::v[y][xc]     = 0.0f;
                    smoke::uPrev[y][xc] = 0.0f;
                    smoke::vPrev[y][xc] = 0.0f;
                    gR[y][xc]    = 0.0f;
                    gG[y][xc]    = 0.0f;
                    gB[y][xc]    = 0.0f;
                    tR[y][xc]    = 0.0f;
                    tG[y][xc]    = 0.0f;
                    tB[y][xc]    = 0.0f;
                }
            }

            // Perimeter velocity clamping — prevent flow from entering the segment.
            const int top    = (r0 - 1 < 0)         ? 0          : r0 - 1;
            const int bottom = (r1 + 1 >= HEIGHT)   ? HEIGHT - 1 : r1 + 1;
            const int left   = (c0 - 1 < 0)         ? 0          : c0 - 1;
            const int right  = (c1 + 1 >= WIDTH)    ? WIDTH - 1  : c1 + 1;

            // Top row: no downward (positive v) flow into obstacle.
            for (int xc = c0; xc <= c1; xc++) {
                if (smoke::v[top][xc] > 0.0f) smoke::v[top][xc] = 0.0f;
            }
            // Bottom row: no upward (negative v) flow into obstacle.
            for (int xc = c0; xc <= c1; xc++) {
                if (smoke::v[bottom][xc] < 0.0f) smoke::v[bottom][xc] = 0.0f;
            }
            // Left col: no rightward (positive u) flow into obstacle.
            for (int y = r0; y <= r1; y++) {
                if (smoke::u[y][left] > 0.0f) smoke::u[y][left] = 0.0f;
            }
            // Right col: no leftward (negative u) flow into obstacle.
            for (int y = r0; y <= r1; y++) {
                if (smoke::u[y][right] < 0.0f) smoke::u[y][right] = 0.0f;
            }
        }

        // Soft-mask blend across the bbox — softens the inner gap corners
        // so flow through the slot doesn't look unnaturally pixel-perfect.
        const int br0 = obstacleBounds[0];
        const int br1 = obstacleBounds[1];
        const int bc0 = obstacleBounds[2];
        const int bc1 = obstacleBounds[3];
        for (int y = br0; y <= br1; y++) {
            for (int xc = bc0; xc <= bc1; xc++) {
                const float sm = obstacleSoftMaskArr[y][xc];
                if (sm <= 0.0f) continue;
                const float invSoft = 1.0f - sm;
                const float dyeFade = 1.0f - sm * 0.35f;
                smoke::u[y][xc]     *= invSoft;
                smoke::v[y][xc]     *= invSoft;
                smoke::uPrev[y][xc] *= invSoft;
                smoke::vPrev[y][xc] *= invSoft;
                gR[y][xc]    *= dyeFade;
                gG[y][xc]    *= dyeFade;
                gB[y][xc]    *= dyeFade;
                tR[y][xc]    *= dyeFade;
                tG[y][xc]    *= dyeFade;
                tB[y][xc]    *= dyeFade;
            }
        }

        smoke::setBnd(1, smoke::u);
        smoke::setBnd(2, smoke::v);
    }

    // Generic per-channel hard-clear inside solid segments.
    // Reused by future temperature/heat field — same interface.
    static void applyObstacleField(float (*grid)[WIDTH]) {
        if (!obstacleCommon.enable || !obstacleHas) return;
        for (int s = 0; s < obstacleSegmentCount; s++) {
            const int r0 = obstacleSegments[s][0];
            const int r1 = obstacleSegments[s][1];
            const int c0 = obstacleSegments[s][2];
            const int c1 = obstacleSegments[s][3];
            for (int y = r0; y <= r1; y++) {
                for (int xc = c0; xc <= c1; xc++) {
                    if (obstacleMaskArr[y][xc]) grid[y][xc] = 0.0f;
                }
            }
        }
    }

    // Generic LED overlay: write the obstacle color to LEDs at solid
    // cells. Called from the engine after renderFluidToLeds. Solver
    // enforce (`enable`) and overlay draw (`overlay`) are independent
    // so a user can verify dye actually deflects vs. is just hidden.
    static void applyObstacleOverlay() {
        if (!obstacleCommon.overlay || !obstacleHas) return;
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                if (!obstacleMaskArr[y][x]) continue;
                uint16_t idx = xyFunc((uint8_t)x, (uint8_t)y);
                if (idx >= NUM_LEDS) continue;
                leds[idx].r = f2u8d(obstacleCommon.colorR, x, y);
                leds[idx].g = f2u8d(obstacleCommon.colorG, x, y);
                leds[idx].b = f2u8d(obstacleCommon.colorB, x, y);
            }
        }
    }

    /*// Forward decl — body lives in obstacles/obstacle_paddles.h. When a
    // second generator is added, this becomes a dispatch (or stays a
    // direct call if only one is active at a time).
    static void generatePaddleObstacle();

    // Regenerate mask from the active generator + apply BC.
    // Called from runfastFluid() each frame.
    static void updateObstacle() {
        generatePaddleObstacle();
        applyObstacleVelocity();
    }*/

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace fastFluid
