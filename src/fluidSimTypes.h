#pragma once

// ═══════════════════════════════════════════════════════════════════
//  fluidSimTypes.h — Anchor header for the FluidSim system.
//  All shared types, constants, global instances, math helpers,
//  and drawing primitives live here.
//  Sub-headers (flow_fluid.h, emitter_fluidJet.h) include this.
// ═══════════════════════════════════════════════════════════════════

#include <FastLED.h>
#include "componentEnums.h"

namespace fluidSim {

    constexpr float CT_PI = 3.14159265358979f;
    constexpr float CT_2PI = 6.28318530717958f;

    // ═══════════════════════════════════════════════════════════════════
    //  GRID STATE & TIMING
    // ═══════════════════════════════════════════════════════════════════

    bool fluidSimInstance = false;
    uint16_t (*xyFunc)(uint8_t x, uint8_t y);

    // Floating-point RGB grid, row-major [y][x].
    // Two copies: g* is the live buffer, t* is scratch for advection.
    static float gR[HEIGHT][WIDTH], gG[HEIGHT][WIDTH], gB[HEIGHT][WIDTH];
    static float tR[HEIGHT][WIDTH], tG[HEIGHT][WIDTH], tB[HEIGHT][WIDTH];

    static unsigned long lastFrameMs;
    uint8_t lastEmitter = 255;  // force initial setup on first frame
    uint8_t lastFlow = 255;  // force initial setup on first frame
    bool useRainbow = false;  // false = spectrum (even HSV), true = FastLED rainbow character

    // Shared frame timing — set once per frame in runFluidSim(), read by all emitters and flows.
    // Scaled by globalSpeed so all time-based behavior respects the master clock.
    static float t  = 0.0f;   // virtual elapsed time (seconds), accumulated each frame
    static float dt = 0.0f;   // virtual frame delta (seconds), already scaled by globalSpeed
    float globalSpeed = 0.5f;  // master clock multiplier
    uint8_t paletteBlendRate = 16;


    // ═══════════════════════════════════════════════════════════════════
    //  MATH HELPERS
    // ═══════════════════════════════════════════════════════════════════

    // Non-negative float modulo (matches Python's % for positive m).
    static inline float fmodPos(float x, float m) {
        float r = fl::fmodf(x, m);
        return r < 0.0f ? r + m : r;
    }

    static inline float clampf(float v, float lo, float hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    struct ColorF { float r, g, b; };

    static inline uint8_t f2u8(float v) {
        if (v <= 0.0f)   return 0;
        if (v >= 255.0f) return 255;
        // Round to nearest to reduce low-end bias vs truncation.
        int i = (int)(v + 0.5f);
        return (uint8_t)i;
    }

    // 4x4 Bayer matrix for ordered output dithering. Values are centered at 0
    // with range ~[-0.47, +0.47] (16 distinct values, 1/16 spacing).
    static const float bayerOutputDither[4][4] = {
        { -7.5f / 16.0f, +0.5f / 16.0f, -5.5f / 16.0f, +2.5f / 16.0f },
        { +4.5f / 16.0f, -3.5f / 16.0f, +6.5f / 16.0f, -1.5f / 16.0f },
        { -4.5f / 16.0f, +3.5f / 16.0f, -6.5f / 16.0f, +1.5f / 16.0f },
        { +7.5f / 16.0f, -0.5f / 16.0f, +5.5f / 16.0f, -2.5f / 16.0f }
    };

    // Dithered float→uint8: shifts the rounding boundary spatially per pixel.
    // Adjacent cells with similar float values quantize to different uint8s,
    // breaking the visible bands at the LED hardware boundary.
    static inline uint8_t f2u8d(float v, int x, int y) {
        return f2u8(v + bayerOutputDither[y & 3][x & 3]);
    }

    // Wrapper functions that take radians and return float (-1.0 to 1.0)
    // Using FastLED's sin32/cos32 approximations for better performance
    constexpr float RADIANS_TO_SIN32 = 2671177.0f;  // 16777216 / (2*PI)
    constexpr float SIN32_TO_FLOAT = 1.0f / 2147418112.0f;  // reciprocal for multiply instead of divide

    inline float sin_fast(float angle_radians) {
        uint32_t angle_sin32 = (uint32_t)(angle_radians * RADIANS_TO_SIN32);
        return fl::sin32(angle_sin32) * SIN32_TO_FLOAT;
    }

    inline float cos_fast(float angle_radians) {
        uint32_t angle_cos32 = (uint32_t)(angle_radians * RADIANS_TO_SIN32);
        return fl::cos32(angle_cos32) * SIN32_TO_FLOAT;
    }

    // Combined sin+cos from a single LUT pass — one radians->uint32 conversion,
    // shared table lookups. Used in render_value where both are needed for the
    // same angle.
    struct SinCosResult { float sin_val; float cos_val; };

    inline SinCosResult sincos_fast(float angle_radians) {
        uint32_t angle = (uint32_t)(angle_radians * RADIANS_TO_SIN32);
        fl::SinCos32 sc = fl::sincos32(angle);
        return { sc.sin_val * SIN32_TO_FLOAT, sc.cos_val * SIN32_TO_FLOAT };
    }

    #define FL_SIN_F(x) sin_fast(x)
    #define FL_COS_F(x) cos_fast(x)

    // IEEE 754 bit-trick fast power for base in [0,1]. ~5% error, 10-20x faster than powf.
    inline float fastpow(float base, float exp) {
        union { float f; int32_t i; } v = { base };
        v.i = (int32_t)(exp * (v.i - 1065353216) + 1065353216);
        return v.f;
    }

    // ═══════════════════════════════════════════════════════════════════
    //  COLOR MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════

    // Spectrum: standard HSV with even 60° sectors.
    static ColorF hsvSpectrum(float hue) {
        float h6 = hue * 6.0f;
        int sector = (int)h6;
        float frac = h6 - sector;
        float r, g, b;
        switch (sector % 6) {
            case 0: r = 1.0f;        g = frac;        b = 0.0f;        break;
            case 1: r = 1.0f - frac; g = 1.0f;        b = 0.0f;        break;
            case 2: r = 0.0f;        g = 1.0f;        b = frac;        break;
            case 3: r = 0.0f;        g = 1.0f - frac; b = 1.0f;        break;
            case 4: r = frac;        g = 0.0f;        b = 1.0f;        break;
            case 5: r = 1.0f;        g = 0.0f;        b = 1.0f - frac; break;
            default: r = g = b = 0.0f; break;
        }
        return ColorF{r * 255.0f, g * 255.0f, b * 255.0f};
    }

    // FastLED rainbow character in float precision (no uint8 banding).
    // 8-section piecewise curve: compressed yellow, expanded red/blue/purple.
    // Derived from FastLED's hsv2rgb_rainbow (Y1 mode).
    static ColorF hsvRainbow(float hue) {
        float h8 = hue * 8.0f;
        int section = (int)h8;
        float frac = h8 - section;
        float third = frac * 85.0f;
        float twothirds = frac * 170.0f;
        float r, g, b;
        switch (section % 8) {
            case 0: r = 255.0f - third; g = third;            b = 0.0f;              break; // R → O
            case 1: r = 171.0f;         g = 85.0f + third;    b = 0.0f;              break; // O → Y
            case 2: r = 171.0f - twothirds; g = 170.0f + third; b = 0.0f;            break; // Y → G
            case 3: r = 0.0f;           g = 255.0f - third;   b = third;             break; // G → A
            case 4: r = 0.0f;           g = 171.0f - twothirds; b = 85.0f + twothirds; break; // A → B
            case 5: r = third;          g = 0.0f;             b = 255.0f - third;    break; // B → P
            case 6: r = 85.0f + third;  g = 0.0f;             b = 171.0f - third;    break; // P → K
            case 7: r = 170.0f + third; g = 0.0f;             b = 85.0f - third;     break; // K → R
            default: r = g = b = 0.0f; break;
        }
        return ColorF{r, g, b};
    }

    // Full-saturation, full-brightness rainbow from a continuous hue.
    // Float-precision HSV→RGB eliminates banding from uint8 hue quantization.
    // useRainbow toggles between even spectrum and FastLED rainbow character.
    static ColorF paletteColor(float hue) {
        uint16_t index = (uint16_t)(fmodPos(hue, 1.0f) * 65535.0f + 0.5f);
        fl::CRGB16 c = fl::ColorFromPaletteHD(gCurrentPalette, index, 255, LINEARBLEND_NOWRAP);
        return ColorF{
            c.r.raw() * (1.0f / 256.0f),
            c.g.raw() * (1.0f / 256.0f),
            c.b.raw() * (1.0f / 256.0f)
        };
    }

    static ColorF rainbow(float t, float speed, float phase) {
        float hue = fmodPos(t * speed + phase, 1.0f);
        if (cPaletteMode) {
            return paletteColor(hue);
        }
        return useRainbow ? hsvRainbow(hue) : hsvSpectrum(hue);
    }

    inline void startingPalette() {
        if (gGradientPaletteCount == 0) {return;}
        gCurrentPaletteNumber = random(0, gGradientPaletteCount);
        gTargetPaletteNumber = gCurrentPaletteNumber;
        gCurrentPalette = gGradientPalettes[gCurrentPaletteNumber];
        gTargetPalette = gCurrentPalette;
    }

    inline void setTargetPalette(uint8_t paletteNumber) {
        if (gGradientPaletteCount == 0) {return;}
        gTargetPaletteNumber = paletteNumber % gGradientPaletteCount;
        gTargetPalette = gGradientPalettes[gTargetPaletteNumber];
    }

    inline void nblendPalette32TowardPalette32(
        fl::CRGBPalette32& current,
        const fl::CRGBPalette32& target,
        fl::u8 maxChanges
    ) {
        fl::u8* currentBytes = reinterpret_cast<fl::u8*>(current.entries);
        const fl::u8* targetBytes = reinterpret_cast<const fl::u8*>(target.entries);
        fl::u8 changes = 0;

        for (uint16_t i = 0; i < sizeof(fl::CRGBPalette32); i++) {
            if (currentBytes[i] == targetBytes[i]) {
                continue;
            }

            if (currentBytes[i] < targetBytes[i]) {
                currentBytes[i]++;
                changes++;
            }

            if (currentBytes[i] > targetBytes[i]) {
                currentBytes[i]--;
                changes++;
                if (currentBytes[i] > targetBytes[i]) {
                    currentBytes[i]--;
                }
            }

            if (changes >= maxChanges) {
                break;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    //  DRAWING PRIMITIVES
    // ═══════════════════════════════════════════════════════════════════

    // Draw an anti-aliased sub-pixel dot into the float grid.
    static void drawDot(float cx, float cy, float diam,
                            float cr, float cg, float cb) {
        float rad = diam * 0.5f;
        int x0 = max(0,               (int)fl::floorf(cx - rad - 1.0f));
        int x1 = min((int)WIDTH  - 1, (int)fl::ceilf (cx + rad + 1.0f));
        int y0 = max(0,               (int)fl::floorf(cy - rad - 1.0f));
        int y1 = min((int)HEIGHT - 1, (int)fl::ceilf (cy + rad + 1.0f));

        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                float dx   = (x + 0.5f) - cx;
                float dy   = (y + 0.5f) - cy;
                float dist = fl::sqrtf(dx * dx + dy * dy);
                float cov  = clampf(rad + 0.5f - dist, 0.0f, 1.0f);
                if (cov <= 0.0f) continue;
                float inv = 1.0f - cov;
                gR[y][x] = gR[y][x] * inv + cr * cov;
                gG[y][x] = gG[y][x] * inv + cg * cov;
                gB[y][x] = gB[y][x] * inv + cb * cov;
            }
        }
    }

    // Blend a single pixel with weighted alpha (used by line and disc drawing).
    static void blendPixelWeighted(int px, int py,
                                    float cr, float cg, float cb,
                                    float w) {
        if (px < 0 || px >= WIDTH || py < 0 || py >= HEIGHT) return;
        w = clampf(w, 0.0f, 1.0f);
        if (w <= 0.0f) return;
        float inv = 1.0f - w;
        gR[py][px] = gR[py][px] * inv + cr * w;
        gG[py][px] = gG[py][px] * inv + cg * w;
        gB[py][px] = gB[py][px] * inv + cb * w;
    }

    // Anti-aliased disc at a sub-pixel position (for line endpoints).
    static void drawAAEndpointDisc(float cx, float cy,
                                    float cr, float cg, float cb,
                                    float radius = 0.85f) {
        int x0 = max(0,               (int)fl::floorf(cx - radius - 1.0f));
        int x1 = min((int)WIDTH  - 1, (int)fl::ceilf (cx + radius + 1.0f));
        int y0 = max(0,               (int)fl::floorf(cy - radius - 1.0f));
        int y1 = min((int)HEIGHT - 1, (int)fl::ceilf (cy + radius + 1.0f));
        for (int py = y0; py <= y1; py++) {
            for (int px = x0; px <= x1; px++) {
                float dx   = (px + 0.5f) - cx;
                float dy   = (py + 0.5f) - cy;
                float dist = fl::sqrtf(dx * dx + dy * dy);
                float w    = clampf(radius + 0.5f - dist, 0.0f, 1.0f);
                blendPixelWeighted(px, py, cr, cg, cb, w);
            }
        }
    }

    // Anti-aliased sub-pixel line with rainbow color varying along its length.
    static void drawAASubpixelLine(float x0, float y0, float x1, float y1,
                                    float t, float colorShift) {
        float dx = x1 - x0;
        float dy = y1 - y0;
        float maxd = fl::fabsf(dx) > fl::fabsf(dy) ? fl::fabsf(dx) : fl::fabsf(dy);
        int steps = max(1, (int)(maxd * 3.0f));
        for (int i = 0; i <= steps; i++) {
            float u  = (float)i / (float)steps;
            float x  = x0 + dx * u;
            float y  = y0 + dy * u;
            int   xi = (int)fl::floorf(x);
            int   yi = (int)fl::floorf(y);
            float fx = x - xi;
            float fy = y - yi;
            ColorF c = rainbow(t, colorShift, u);
            blendPixelWeighted(xi,     yi,     c.r, c.g, c.b, (1.0f - fx) * (1.0f - fy));
            blendPixelWeighted(xi + 1, yi,     c.r, c.g, c.b, fx * (1.0f - fy));
            blendPixelWeighted(xi,     yi + 1, c.r, c.g, c.b, (1.0f - fx) * fy);
            blendPixelWeighted(xi + 1, yi + 1, c.r, c.g, c.b, fx * fy);
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    //  COMPONENT TYPES & ENUMS
    // ═══════════════════════════════════════════════════════════════════

    // Emitter and Flow enums defined in componentEnums.h

    // Function pointer types for dispatch (read shared t/dt from namespace)
    using EmitterFn     = void(*)();
    using FlowPrepFn    = void(*)();
    using FlowAdvectFn  = void(*)();

    // ═══════════════════════════════════════════════════════════════════
    //  MODULATION TYPES
    // ═══════════════════════════════════════════════════════════════════

    struct ModConfig {
        // Hardcoded by developer — architectural choices, set on the instance in the emitter/flow/obstacle file
        uint8_t modTimer = 0;          // which timer index to read from (0 to num_timers)

        // UI-tunable via cVars — struct values are defaults, overwritten by syncFromCVars()
        float   modRate  = 0.0f;       // UI adjustment to timings.ratio[timer] (developer uses in formula)
        float   modLevel = 0.0f;       // modulation depth (0 = mod off)
    };

    template <typename Params, size_t N>
    inline uint8_t assignModSlots(Params& params, ModConfig Params::* const (&mods)[N], uint8_t baseSlot) {
        for (uint8_t i = 0; i < N; i++) {
            (params.*mods[i]).modTimer = baseSlot + i;
        }
        return baseSlot + N;
    }

    template <typename Params, size_t N>
    constexpr uint8_t modCount(ModConfig Params::* const (&)[N]) {
        return static_cast<uint8_t>(N);
    }

    // ═══════════════════════════════════════════════════════════════════
    //  GLOBAL CONFIG
    // ═══════════════════════════════════════════════════════════════════

    Emitter activeEmitter = EMITTER_THREEJET;
    Flow    activeFlow    = FLOW_FLUID;

    uint8_t activeEmitterTimers = 0;
    uint8_t activeFlowTimers = 0;
    uint8_t activeObstacleTimers = 0;

} // namespace fluidSim
