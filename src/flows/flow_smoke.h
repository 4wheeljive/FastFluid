#pragma once

// ═══════════════════════════════════════════════════════════════════
//  SMOKE (NAVIER-STOKES) SOLVER — flow_smoke.h
// ═══════════════════════════════════════════════════════════════════
//
//  Stable-fluids simulation (Jos Stam, 1999).  Maintains a persistent
//  velocity field (u, v) that advects RGB dye.  Each frame:
//    - velocity diffuse → project → self-advect → project
//    - optional vorticity confinement → project
//    - dye diffuse + advect through final velocity field
//    - per-frame dissipation
//
//  Ported from colorTrailsOrig/navier_stokes_1.py.

#include "fastFluidTypes.h"
#include "modulators.h"

namespace fastFluid {

    extern bool obstacleHas;
    static void applyObstacleVelocity();
    static void applyObstacleField(float (*grid)[WIDTH]);

namespace smoke {

    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    struct SmokeParams {
        float viscosity           = 0.0f;     // velocity diffusion coefficient
        float diffusion           = 0.0f;     // dye diffusion coefficient
        float velocityDissipation = 0.5f;     // per-second velocity decay (0..1, 1=no decay)
        float dyeDissipation      = 0.4f;     // per-second dye decay (0..1, 1=no decay)
        float vorticity           = 0.0f;     // confinement strength (0 = disabled)
        // Directional gravity: applied as a uniform force inside the velocity step.
        // Angle convention matches jetAngle: 0°=up, 90°=right, 180°=down, 270°=left.
        float gravityForce        = 0.0f;     // intensity (0 = disabled)
        float gravityAngle        = 180.0f;   // direction in degrees (default = down)
        // Split iteration counts. Diffuse is well-conditioned and converges
        // in a handful of passes; pressure projection (incompressibility)
        // benefits from many more. Decoupling lets you crank projection
        // without paying for it on diffuse when viscosity > 0.
        uint8_t diffuseIterations = 6;        // Jacobi passes for diffuse
        uint8_t projectIterations = 10;       // Jacobi passes for pressure projection

        ModConfig modVelDissip = {0, 0.5f, 0.0f};   // modTimer, modRate, modLevel
        ModConfig modDyeDissip = {1, 0.5f, 0.0f};
    };

    static constexpr ModConfig SmokeParams::* SMOKE_MODS[] = {
        &SmokeParams::modVelDissip,
        &SmokeParams::modDyeDissip
    };

    SmokeParams smoke;

    // Working values prepared each frame by smokePrepare()
    static float workVelDissip = 0.5f;
    static float workDyeDissip = 0.5f;

    // Persistent simulation state (survives across frames)
    static float u[HEIGHT][WIDTH], v[HEIGHT][WIDTH];
    static float uPrev[HEIGHT][WIDTH], vPrev[HEIGHT][WIDTH];
    static float pressure[HEIGHT][WIDTH], divergence[HEIGHT][WIDTH];

    // Internal "size" parameter for the solver (scales velocity-to-cells conversion).
    // Stam's algorithm assumes a square grid; we pick a single representative size.
    static constexpr float SIM_SIZE = (float)MIN_DIMENSION;

    // ───────────────────────────────────────────────────────────────
    //  Boundary conditions
    //    b == 0: scalar (dye, pressure) — no enforcement (relies on clamp in samplers)
    //    b == 1: u-velocity — zero at left/right walls (no penetration)
    //    b == 2: v-velocity — zero at top/bottom walls (no penetration)
    // ───────────────────────────────────────────────────────────────
    static void setBnd(int b, float (*x)[WIDTH]) {
        if (b == 1) {
            for (int y = 0; y < HEIGHT; y++) {
                x[y][0]         = 0.0f;
                x[y][WIDTH - 1] = 0.0f;
            }
        } else if (b == 2) {
            for (int xc = 0; xc < WIDTH; xc++) {
                x[0][xc]          = 0.0f;
                x[HEIGHT - 1][xc] = 0.0f;
            }
        }
    }

    // Sample with edge clamping (mirror behavior of Python's set_bnd for scalars)
    static inline float sampleClamped(float (*x)[WIDTH], int yi, int xi) {
        if (yi < 0) yi = 0; else if (yi >= HEIGHT) yi = HEIGHT - 1;
        if (xi < 0) xi = 0; else if (xi >= WIDTH)  xi = WIDTH  - 1;
        return x[yi][xi];
    }

    // ───────────────────────────────────────────────────────────────
    //  Linear solver (Jacobi iteration)
    //    x[i,j] = (x0[i,j] + a*(x[i-1,j] + x[i+1,j] + x[i,j-1] + x[i,j+1])) / c
    // ───────────────────────────────────────────────────────────────
    static void linSolve(int b, float (*x)[WIDTH], float (*x0)[WIDTH], float a, float c, int iter) {
        const float invC = 1.0f / c;
        for (int k = 0; k < iter; k++) {
            for (int y = 0; y < HEIGHT; y++) {
                for (int xc = 0; xc < WIDTH; xc++) {
                    float xm = (xc > 0)         ? x[y][xc - 1] : x[y][xc];
                    float xp = (xc < WIDTH - 1) ? x[y][xc + 1] : x[y][xc];
                    float ym = (y  > 0)          ? x[y - 1][xc] : x[y][xc];
                    float yp = (y  < HEIGHT - 1) ? x[y + 1][xc] : x[y][xc];
                    x[y][xc] = (x0[y][xc] + a * (xm + xp + ym + yp)) * invC;
                }
            }
            setBnd(b, x);
        }
    }

    // Fast-diffusion threshold (matches ns_3's `fast_diffusion_threshold = 1e-8`).
    // Skip the linSolve when the effective coefficient is negligible — covers
    // diff==0 and tiny-dt paused-frame cases without a separate guard.
    static constexpr float FAST_DIFFUSION_THRESHOLD = 1e-8f;

    static void diffuse(int b, float (*x)[WIDTH], float (*x0)[WIDTH], float diff, float dt_) {
        const float a = dt_ * diff * SIM_SIZE * SIM_SIZE;
        if (a <= FAST_DIFFUSION_THRESHOLD) {
            fl::memcpy(x, x0, sizeof(float) * HEIGHT * WIDTH);
            setBnd(b, x);
            return;
        }
        linSolve(b, x, x0, a, 1.0f + 4.0f * a, smoke.diffuseIterations);
    }

    // Semi-Lagrangian advection: backtrace each cell along velocity field, bilinearly sample source
    static void advectField(int b, float (*d)[WIDTH], float (*d0)[WIDTH],
                            float (*velU)[WIDTH], float (*velV)[WIDTH], float dt_) {
        const float dt0 = dt_ * SIM_SIZE;
        const float maxX = (float)WIDTH  - 1.5f;
        const float maxY = (float)HEIGHT - 1.5f;

        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                float sx = (float)xc - dt0 * velU[y][xc];
                float sy = (float)y  - dt0 * velV[y][xc];
                sx = clampf(sx, 0.5f, maxX);
                sy = clampf(sy, 0.5f, maxY);

                int   ix0 = (int)sx;
                int   iy0 = (int)sy;
                int   ix1 = ix0 + 1;
                int   iy1 = iy0 + 1;
                if (ix1 >= WIDTH)  ix1 = WIDTH  - 1;
                if (iy1 >= HEIGHT) iy1 = HEIGHT - 1;

                float fx = sx - ix0;
                float fy = sy - iy0;

                float top = d0[iy0][ix0] * (1.0f - fx) + d0[iy0][ix1] * fx;
                float bot = d0[iy1][ix0] * (1.0f - fx) + d0[iy1][ix1] * fx;
                d[y][xc] = top * (1.0f - fy) + bot * fy;
            }
        }
        setBnd(b, d);
    }

    // Combined advection for fields sharing the same backtrace. This keeps
    // the generic scalar advectField() available for future one-off fields,
    // but avoids recomputing coordinates and bilinear weights for velocity
    // and RGB dye every frame.
    static void advectVelocityPair(float dt_) {
        const float dt0 = dt_ * SIM_SIZE;
        const float maxX = (float)WIDTH  - 1.5f;
        const float maxY = (float)HEIGHT - 1.5f;

        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                float sx = (float)xc - dt0 * uPrev[y][xc];
                float sy = (float)y  - dt0 * vPrev[y][xc];
                sx = clampf(sx, 0.5f, maxX);
                sy = clampf(sy, 0.5f, maxY);

                int ix0 = (int)sx;
                int iy0 = (int)sy;
                int ix1 = ix0 + 1;
                int iy1 = iy0 + 1;
                if (ix1 >= WIDTH)  ix1 = WIDTH  - 1;
                if (iy1 >= HEIGHT) iy1 = HEIGHT - 1;

                const float fx = sx - ix0;
                const float fy = sy - iy0;
                const float wx0 = 1.0f - fx;
                const float wy0 = 1.0f - fy;
                const float w00 = wx0 * wy0;
                const float w10 = fx  * wy0;
                const float w01 = wx0 * fy;
                const float w11 = fx  * fy;

                u[y][xc] = uPrev[iy0][ix0] * w00 + uPrev[iy0][ix1] * w10 +
                           uPrev[iy1][ix0] * w01 + uPrev[iy1][ix1] * w11;
                v[y][xc] = vPrev[iy0][ix0] * w00 + vPrev[iy0][ix1] * w10 +
                           vPrev[iy1][ix0] * w01 + vPrev[iy1][ix1] * w11;
            }
        }
        setBnd(1, u);
        setBnd(2, v);
    }

    static void advectDyeRGB(float dt_) {
        const float dt0 = dt_ * SIM_SIZE;
        const float maxX = (float)WIDTH  - 1.5f;
        const float maxY = (float)HEIGHT - 1.5f;

        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                float sx = (float)xc - dt0 * u[y][xc];
                float sy = (float)y  - dt0 * v[y][xc];
                sx = clampf(sx, 0.5f, maxX);
                sy = clampf(sy, 0.5f, maxY);

                int ix0 = (int)sx;
                int iy0 = (int)sy;
                int ix1 = ix0 + 1;
                int iy1 = iy0 + 1;
                if (ix1 >= WIDTH)  ix1 = WIDTH  - 1;
                if (iy1 >= HEIGHT) iy1 = HEIGHT - 1;

                const float fx = sx - ix0;
                const float fy = sy - iy0;
                const float wx0 = 1.0f - fx;
                const float wy0 = 1.0f - fy;
                const float w00 = wx0 * wy0;
                const float w10 = fx  * wy0;
                const float w01 = wx0 * fy;
                const float w11 = fx  * fy;

                gR[y][xc] = tR[iy0][ix0] * w00 + tR[iy0][ix1] * w10 +
                            tR[iy1][ix0] * w01 + tR[iy1][ix1] * w11;
                gG[y][xc] = tG[iy0][ix0] * w00 + tG[iy0][ix1] * w10 +
                            tG[iy1][ix0] * w01 + tG[iy1][ix1] * w11;
                gB[y][xc] = tB[iy0][ix0] * w00 + tB[iy0][ix1] * w10 +
                            tB[iy1][ix0] * w01 + tB[iy1][ix1] * w11;
            }
        }
        setBnd(0, gR);
        setBnd(0, gG);
        setBnd(0, gB);
    }

    // Hodge projection: subtract pressure gradient to make velocity divergence-free
    static void project() {
        const float h = 1.0f / SIM_SIZE;

        // 1. Compute divergence of velocity field
        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                float uxp = (xc < WIDTH  - 1) ? u[y][xc + 1] : u[y][xc];
                float uxm = (xc > 0)          ? u[y][xc - 1] : u[y][xc];
                float vyp = (y  < HEIGHT - 1) ? v[y + 1][xc] : v[y][xc];
                float vym = (y  > 0)          ? v[y - 1][xc] : v[y][xc];
                divergence[y][xc] = -0.5f * h * (uxp - uxm + vyp - vym);
                pressure[y][xc]   = 0.0f;
            }
        }
        setBnd(0, divergence);
        setBnd(0, pressure);

        // 2. Solve Poisson equation for pressure
        linSolve(0, pressure, divergence, 1.0f, 4.0f, smoke.projectIterations);

        // 3. Subtract pressure gradient from velocity
        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                float pxp = (xc < WIDTH  - 1) ? pressure[y][xc + 1] : pressure[y][xc];
                float pxm = (xc > 0)          ? pressure[y][xc - 1] : pressure[y][xc];
                float pyp = (y  < HEIGHT - 1) ? pressure[y + 1][xc] : pressure[y][xc];
                float pym = (y  > 0)          ? pressure[y - 1][xc] : pressure[y][xc];
                u[y][xc] -= 0.5f * (pxp - pxm) * SIM_SIZE;
                v[y][xc] -= 0.5f * (pyp - pym) * SIM_SIZE;
            }
        }
        setBnd(1, u);
        setBnd(2, v);
    }

    // Vorticity confinement: re-add small swirls lost to numerical diffusion.
    // Uses `divergence` as scratch for the curl field.
    static void applyVorticityConfinement(float dt_) {
        // 1. Compute curl (scalar 2D vorticity) into divergence buffer
        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                float vxp = (xc < WIDTH  - 1) ? v[y][xc + 1] : v[y][xc];
                float vxm = (xc > 0)          ? v[y][xc - 1] : v[y][xc];
                float uyp = (y  < HEIGHT - 1) ? u[y + 1][xc] : u[y][xc];
                float uym = (y  > 0)          ? u[y - 1][xc] : u[y][xc];
                divergence[y][xc] = 0.5f * (vxp - vxm - (uyp - uym));
            }
        }

        // 2. Compute gradient of |curl|, normalize, apply force
        const float strength = smoke.vorticity;
        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                float gxp = fl::fabsf(sampleClamped(divergence, y,     xc + 1));
                float gxm = fl::fabsf(sampleClamped(divergence, y,     xc - 1));
                float gyp = fl::fabsf(sampleClamped(divergence, y + 1, xc));
                float gym = fl::fabsf(sampleClamped(divergence, y - 1, xc));
                float gx = 0.5f * (gxp - gxm);
                float gy = 0.5f * (gyp - gym);
                float len = fl::sqrtf(gx * gx + gy * gy) + 1e-5f;
                gx /= len;
                gy /= len;
                float w = divergence[y][xc];
                u[y][xc] += dt_ * strength *  gy * w;
                v[y][xc] += dt_ * strength * -gx * w;
            }
        }
        setBnd(1, u);
        setBnd(2, v);
    }

    // ───────────────────────────────────────────────────────────────
    //  Public emitter-side hooks
    // ───────────────────────────────────────────────────────────────
    static inline void smokeAddVelocity(int xc, int yc, float du, float dv) {
        if (xc < 0 || xc >= WIDTH || yc < 0 || yc >= HEIGHT) return;
        u[yc][xc] += du;
        v[yc][xc] += dv;
    }

    // ───────────────────────────────────────────────────────────────
    //  Pipeline entry points
    // ───────────────────────────────────────────────────────────────
    // Phase 1 of frame: write this component's timer slot ratios.
    // Engine collects all components' ratios, then runs ONE central
    // calculate_modulators call before any component reads `move[*]`.
    static void prepFlowMods() {
        timings.ratio[smoke.modVelDissip.modTimer] = 0.0004f  * smoke.modVelDissip.modRate;
        timings.ratio[smoke.modDyeDissip.modTimer] = 0.00045f * smoke.modDyeDissip.modRate;
    }

    // Phase 3 of frame: read modulator output and compute work values.
    // Called AFTER calculate_modulators has run.
    static void prepFlow() {
        const ModConfig& velMod = smoke.modVelDissip;
        const ModConfig& dyeMod = smoke.modDyeDissip;

        // Signal acquisition: bipolar [-1, 1]
        const float velSignal = move.directional_noise[velMod.modTimer];
        const float dySignal  = move.directional_noise[dyeMod.modTimer];

        // Artistic application: orbitalDots-style bipolar modulation,
        // clamped to [0.01, 1.0] to keep dissipation values stable.
        workVelDissip = smoke.velocityDissipation *
            ((1.0f - velMod.modLevel) + velMod.modLevel * velSignal);
        workVelDissip = fmaxf(0.01f, fminf(1.0f, workVelDissip));

        workDyeDissip = smoke.dyeDissipation *
            ((1.0f - dyeMod.modLevel) + dyeMod.modLevel * dySignal);
        workDyeDissip = fmaxf(0.01f, fminf(1.0f, workDyeDissip));
    }

    static void advectFlow() {
        // Decompose 2D gravity from intensity + angle.
        // Angle convention matches jetAngle: 0°=up, 90°=right, 180°=down, 270°=left.
        //   u (horizontal, +x = right):  sin(angle) * force
        //   v (vertical,   +y = down ): -cos(angle) * force
        float gravityU = 0.0f;
        float gravityV = 0.0f;
        if (smoke.gravityForce != 0.0f) {
            const float angleRad = smoke.gravityAngle * (FF_PI / 180.0f);
            SinCosResult sc = sincos_fast(angleRad);
            gravityU =  sc.sin_val * smoke.gravityForce;
            gravityV = -sc.cos_val * smoke.gravityForce;
        }

        // ─── VELOCITY STEP ─────────────────────────────────────────
        const float viscosityA = dt * smoke.viscosity * SIM_SIZE * SIM_SIZE;
        if (viscosityA <= FAST_DIFFUSION_THRESHOLD) {
            setBnd(1, u);
            setBnd(2, v);
        } else {
            fl::memcpy(uPrev, u, sizeof(u));
            fl::memcpy(vPrev, v, sizeof(v));
            diffuse(1, u, uPrev, smoke.viscosity, dt);
            diffuse(2, v, vPrev, smoke.viscosity, dt);
        }

        // Gravity applied between diffuse and project (matches ns_3 step()).
        if (gravityU != 0.0f || gravityV != 0.0f) {
            const float dgu = dt * gravityU;
            const float dgv = dt * gravityV;
            for (int y = 0; y < HEIGHT; y++) {
                for (int xc = 0; xc < WIDTH; xc++) {
                    u[y][xc] += dgu;
                    v[y][xc] += dgv;
                }
            }
            setBnd(1, u);
            setBnd(2, v);
        }

        applyObstacleVelocity();   // hook 1: after diffuse + gravity
        project();

        fl::memcpy(uPrev, u, sizeof(u));
        fl::memcpy(vPrev, v, sizeof(v));
        advectVelocityPair(dt);
        applyObstacleVelocity();   // hook 2: after self-advect
        project();

        if (smoke.vorticity > 0.0f) {
            applyVorticityConfinement(dt);
            applyObstacleVelocity(); // hook 3: after vorticity confinement
            project();
        }

        // ─── DYE STEP ──────────────────────────────────────────────
        // For each channel: optionally diffuse, then advect through u,v.
        // Use tR/tG/tB as the previous-frame buffer.
        if (smoke.diffusion > 0.0f) {
            fl::memcpy(tR, gR, sizeof(gR));
            fl::memcpy(tG, gG, sizeof(gG));
            fl::memcpy(tB, gB, sizeof(gB));
            diffuse(0, gR, tR, smoke.diffusion, dt);
            diffuse(0, gG, tG, smoke.diffusion, dt);
            diffuse(0, gB, tB, smoke.diffusion, dt);
        }

        fl::memcpy(tR, gR, sizeof(gR));
        fl::memcpy(tG, gG, sizeof(gG));
        fl::memcpy(tB, gB, sizeof(gB));
        advectDyeRGB(dt);
        // Dye hard-clear in solid cells — keeps the paddle crisp after advect.
        applyObstacleField(gR);
        applyObstacleField(gG);
        applyObstacleField(gB);

        // ─── DISSIPATION ───────────────────────────────────────────
        const float fadeVel = fl::powf(workVelDissip, dt);
        const float fadeDye = fl::powf(workDyeDissip, dt);
        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                u[y][xc]  *= fadeVel;
                v[y][xc]  *= fadeVel;
                gR[y][xc] *= fadeDye;
                gG[y][xc] *= fadeDye;
                gB[y][xc] *= fadeDye;
            }
        }

        applyObstacleVelocity();   // hook 4: end of step
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} //namespace fastFluid::smoke
} // namespace fastFluid
