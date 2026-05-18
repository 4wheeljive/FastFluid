#pragma once

// ═══════════════════════════════════════════════════════════════════
//  EMITTERS — emitters.h
// ═══════════════════════════════════════════════════════════════════
//
//  Shared emitter primitives. Generic helpers used by all
//  emitter_*.h files live here. Each specific emitter file owns its
//  own params struct + emit/prepare functions; this header is for
//  building blocks they all reuse.
//
//  Currently: Gaussian dye splat with per-cell Bayer hue dither.
//
//  This header references velocity (u, v from flow_fluid.h) and dye
//  fields (gR/gG/gB from fastFluidTypes.h), so it must be included
//  after flow_fluid.h.

#include "fastFluidTypes.h"

namespace fastFluid {
    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    // 4×4 Bayer matrix, values normalized to [-0.5, +0.5].
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

    static inline void buildHueDitherColorTable(float baseHue, ColorF colors[4][4]) {
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                const float hueOffset = bayerHueDither[y][x] * HUE_DITHER_SCALE;
                const float dithered = fmodPos(baseHue + hueOffset, 1.0f);
                colors[y][x] = rainbow(t, 0.0f, dithered);
            }
        }
    }

    // 2D Gaussian splat: writes dye to gR/gG/gB and momentum to u/v.
    // The color table holds the 4x4 Bayer hue offsets for a base hue, so
    // palette/rainbow conversion happens once per hue instead of per cell.
    static void jetSplatWithColors(float cx, float cy, float radius,
                                   float densityMag,
                                   float velX, float velY,
                                   const ColorF colors[4][4]) {
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

                const ColorF& c = colors[y & 3][x & 3];

                const float wScale = w * densityScale;
                gR[y][x] += c.r * wScale;
                gG[y][x] += c.g * wScale;
                gB[y][x] += c.b * wScale;
                u[y][x] += velX * w;
                v[y][x] += velY * w;
            }
        }
    }

    // baseHue selects the splat color (passed by caller, computed from
    // per-emitter hue logic). Per-cell hue dither is applied so adjacent cells
    // get distinct uint8 values at LED output without temporal flicker.
    static void jetSplat(float cx, float cy, float radius,
                         float densityMag,
                         float velX, float velY,
                         float baseHue) {
        ColorF colors[4][4];
        buildHueDitherColorTable(baseHue, colors);
        jetSplatWithColors(cx, cy, radius, densityMag, velX, velY, colors);
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace fastFluid
