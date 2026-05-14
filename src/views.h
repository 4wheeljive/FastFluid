#pragma once

// ═══════════════════════════════════════════════════════════════════
//  VIEWS — views.h
// ═══════════════════════════════════════════════════════════════════

namespace fastFluid {

    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    enum DebugView : uint8_t {
        DEBUG_VIEW_COLOR = 0,
        DEBUG_VIEW_VELOCITY = 1,
        DEBUG_VIEW_VORTICITY = 2,
        DEBUG_VIEW_PRESSURE = 3,
        DEBUG_VIEW_DIVERGENCE = 4,
        DEBUG_VIEW_DIVERGENCE_SIGNED = 5,
        DEBUG_VIEW_DYE_DENSITY = 6,
        DEBUG_VIEW_EMITTER_OVERLAY = 7
    };

    static inline void writeDebugPixel(uint8_t xc, uint8_t y, float r, float g, float b) {
        const uint16_t idx = xyFunc(xc, y);
        if (idx >= NUM_LEDS) return;

        leds[idx].r = f2u8d(clampf(r, 0.0f, 1.0f) * 255.0f, xc, y);
        leds[idx].g = f2u8d(clampf(g, 0.0f, 1.0f) * 255.0f, xc, y);
        leds[idx].b = f2u8d(clampf(b, 0.0f, 1.0f) * 255.0f, xc, y);
    }

    static inline float debugVelocityMagnitude(uint8_t xc, uint8_t y) {
        const float vu = u[y][xc];
        const float vv = v[y][xc];
        return fl::sqrtf(vu * vu + vv * vv);
    }

    static inline float debugDyeDensity(uint8_t xc, uint8_t y) {
        return gR[y][xc] + gG[y][xc] + gB[y][xc];
    }

    static inline float debugVorticity(uint8_t xc, uint8_t y) {
        const float vxp = (xc < WIDTH  - 1) ? v[y][xc + 1] : v[y][xc];
        const float vxm = (xc > 0)          ? v[y][xc - 1] : v[y][xc];
        const float uyp = (y  < HEIGHT - 1) ? u[y + 1][xc] : u[y][xc];
        const float uym = (y  > 0)          ? u[y - 1][xc] : u[y][xc];
        return 0.5f * (vxp - vxm - (uyp - uym));
    }

    static inline float debugDivergence(uint8_t xc, uint8_t y) {
        const float h = 1.0f / SIM_SIZE;
        const float uxp = (xc < WIDTH  - 1) ? u[y][xc + 1] : u[y][xc];
        const float uxm = (xc > 0)          ? u[y][xc - 1] : u[y][xc];
        const float vyp = (y  < HEIGHT - 1) ? v[y + 1][xc] : v[y][xc];
        const float vym = (y  > 0)          ? v[y - 1][xc] : v[y][xc];
        return -0.5f * h * (uxp - uxm + vyp - vym);
    }

    static inline float debugScalarValue(uint8_t view, uint8_t xc, uint8_t y) {
        switch (view) {
            case DEBUG_VIEW_VELOCITY: return debugVelocityMagnitude(xc, y);
            case DEBUG_VIEW_VORTICITY: return debugVorticity(xc, y);
            case DEBUG_VIEW_PRESSURE: return pressure[y][xc];
            case DEBUG_VIEW_DIVERGENCE:
            case DEBUG_VIEW_DIVERGENCE_SIGNED: return debugDivergence(xc, y);
            case DEBUG_VIEW_DYE_DENSITY: return debugDyeDensity(xc, y);
            default: return 0.0f;
        }
    }

    static inline bool debugViewIsSigned(uint8_t view) {
        return view == DEBUG_VIEW_VORTICITY ||
               view == DEBUG_VIEW_PRESSURE ||
               view == DEBUG_VIEW_DIVERGENCE_SIGNED;
    }

    static inline void blendOverlayPixel(int px, int py,
                                         float r, float g, float b,
                                         float alpha) {
        if (px < 0 || px >= WIDTH || py < 0 || py >= HEIGHT) return;
        alpha = clampf(alpha, 0.0f, 1.0f);
        if (alpha <= 0.0f) return;

        const uint16_t idx = xyFunc((uint8_t)px, (uint8_t)py);
        if (idx >= NUM_LEDS) return;

        const float inv = 1.0f - alpha;
        leds[idx].r = f2u8d((float)leds[idx].r * inv + r * 255.0f * alpha, px, py);
        leds[idx].g = f2u8d((float)leds[idx].g * inv + g * 255.0f * alpha, px, py);
        leds[idx].b = f2u8d((float)leds[idx].b * inv + b * 255.0f * alpha, px, py);
    }

    static void drawOverlayDisc(float cx, float cy,
                                float r, float g, float b,
                                float radius) {
        const int x0 = max(0,               (int)fl::floorf(cx - radius - 1.0f));
        const int x1 = min((int)WIDTH  - 1, (int)fl::ceilf (cx + radius + 1.0f));
        const int y0 = max(0,               (int)fl::floorf(cy - radius - 1.0f));
        const int y1 = min((int)HEIGHT - 1, (int)fl::ceilf (cy + radius + 1.0f));

        for (int py = y0; py <= y1; py++) {
            for (int px = x0; px <= x1; px++) {
                const float dx = (px + 0.5f) - cx;
                const float dy = (py + 0.5f) - cy;
                const float dist = fl::sqrtf(dx * dx + dy * dy);
                const float alpha = clampf(radius + 0.5f - dist, 0.0f, 1.0f);
                blendOverlayPixel(px, py, r, g, b, alpha);
            }
        }
    }

    static void drawOverlayLine(float x0, float y0, float x1, float y1,
                                float r, float g, float b) {
        const float dx = x1 - x0;
        const float dy = y1 - y0;
        const float maxd = fl::fabsf(dx) > fl::fabsf(dy) ? fl::fabsf(dx) : fl::fabsf(dy);
        const int steps = max(1, (int)(maxd * 2.5f));

        for (int i = 0; i <= steps; i++) {
            const float u_ = (float)i / (float)steps;
            const float x = x0 + dx * u_;
            const float y = y0 + dy * u_;
            const int xi = (int)fl::floorf(x);
            const int yi = (int)fl::floorf(y);
            const float fx = x - xi;
            const float fy = y - yi;
            blendOverlayPixel(xi,     yi,     r, g, b, (1.0f - fx) * (1.0f - fy));
            blendOverlayPixel(xi + 1, yi,     r, g, b, fx * (1.0f - fy));
            blendOverlayPixel(xi,     yi + 1, r, g, b, (1.0f - fx) * fy);
            blendOverlayPixel(xi + 1, yi + 1, r, g, b, fx * fy);
        }
    }

    static void drawEmitterDebugOverlay() {
        if (activeEmitter != EMITTER_MULTIJET) return;

        const uint8_t count = multiJet::multiJetCount();
        if (count == 0) return;

        const float arrowLength = clampf(multiJet::jetPack.size * 2.0f, 3.0f, (float)MIN_DIMENSION * 0.28f);
        for (uint8_t i = 0; i < count; i++) {
            const JetParams& thisJet = multiJet::jet[i];
            if (!thisJet.enabled) continue;

            float anchorCol;
            float anchorRow;
            multiJet::resolveMultiJetAnchor(i, count, thisJet, anchorCol, anchorRow);

            float dirCol;
            float dirRow;
            multiJet::resolveMultiJetDirection(i, thisJet, anchorCol, anchorRow, dirCol, dirRow);

            const float tipCol = anchorCol + dirCol * arrowLength;
            const float tipRow = anchorRow + dirRow * arrowLength;
            drawOverlayLine(anchorCol, anchorRow, tipCol, tipRow, 0.1f, 0.85f, 1.0f);
            drawOverlayDisc(tipCol, tipRow, 0.1f, 0.85f, 1.0f, 0.85f);
            drawOverlayDisc(anchorCol, anchorRow, 1.0f, 0.95f, 0.1f, 1.35f);
        }
    }

    static void renderDebugView(uint8_t view) {
        if (view > DEBUG_VIEW_EMITTER_OVERLAY) view = DEBUG_VIEW_COLOR;
        if (view == DEBUG_VIEW_COLOR || view == DEBUG_VIEW_EMITTER_OVERLAY) {
            renderFluidToLeds();
            return;
        }

        const bool signedView = debugViewIsSigned(view);
        float maxValue = 1e-6f;

        for (uint8_t y = 0; y < HEIGHT; y++) {
            for (uint8_t xc = 0; xc < WIDTH; xc++) {
                float value = debugScalarValue(view, xc, y);
                if (view == DEBUG_VIEW_DIVERGENCE) value = fl::fabsf(value);
                if (signedView) value = fl::fabsf(value);
                if (value > maxValue) maxValue = value;
            }
        }

        const float invMax = 1.0f / maxValue;
        for (uint8_t y = 0; y < HEIGHT; y++) {
            for (uint8_t xc = 0; xc < WIDTH; xc++) {
                float value = debugScalarValue(view, xc, y);

                if (signedView) {
                    const float mag = clampf(fl::fabsf(value) * invMax, 0.0f, 1.0f);
                    if (value >= 0.0f) {
                        writeDebugPixel(xc, y, mag, mag * 0.18f, 0.0f);
                    } else {
                        writeDebugPixel(xc, y, 0.0f, mag * 0.35f, mag);
                    }
                } else {
                    if (view == DEBUG_VIEW_DIVERGENCE) value = fl::fabsf(value);
                    const float mag = clampf(value * invMax, 0.0f, 1.0f);
                    writeDebugPixel(xc, y, mag, mag, mag);
                }
            }
        }
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace FastFluid
