#pragma once

#include "parameterSchema.h"
#include "fastFluidTypes.h"
#include "noise.h"
#include "modulators.h"
#include "emitters.h"
#include "obstacles.h"
#include "emitters/emitter_singleJet.h"
#include "emitters/emitter_multiJet.h"
#include "flows/flow_smoke.h"
#include "obstacles/obstacle_paddles.h"

namespace fastFluid {

    // ═══════════════════════════════════════════════════════════════════
    //  DISPLAY PIPELINE
    // ═══════════════════════════════════════════════════════════════════
    //  Post-simulation render: dye + velocity → color-graded RGB → LEDs.
    //  Stages (per ns_3 draw()):
    //    1. velocity-magnitude additive glow into base RGB
    //    2. flow saturation/brightness boost around Rec.709 luminance
    //    3. highlight saturation second-pass on bright pixels
    //    4. 5-tap blur glow on (channel - 0.55) regions
    //    5. black-point compression
    //    6. gamma (Color Contrast)
    //    7. Bayer-dither quantization to LEDs
    //
    //  Buffer reuse (post-fluidAdvect, all of these are scratch):
    //    tR/tG/tB:           working RGB (normalized [0,1])
    //    pressure/divergence: glow scratch (init / blurred), reused per channel

    struct RenderParams {
        float colorContrast = 1.0f;
        float blackPoint    = 0.105f;
        float flowSat       = 2.0f;
        float flowBright    = 0.75f;
        float glowStrength  = 0.0f; // was 0.24
        float highlightSat  = 2.0f;
    };

    RenderParams render;

    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    static void renderFluidToLeds();

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

        const uint8_t count = multiJetCount();
        if (count == 0) return;

        const float arrowLength = clampf(jetPack.size * 2.0f, 3.0f, (float)MIN_DIMENSION * 0.28f);
        for (uint8_t i = 0; i < count; i++) {
            const JetParams& thisJet = jet[i];
            if (!thisJet.enabled) continue;

            float anchorCol;
            float anchorRow;
            resolveMultiJetAnchor(i, count, thisJet, anchorCol, anchorRow);

            float dirCol;
            float dirRow;
            resolveMultiJetDirection(thisJet, anchorCol, anchorRow, dirCol, dirRow);

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

    static void renderFluidToLeds() {
        const float invBlack = 1.0f / fmaxf(1e-3f, 1.0f - render.blackPoint);
        const float gamma    = 1.0f / fmaxf(0.2f, render.colorContrast);
        const bool applyGamma = fl::fabsf(gamma - 1.0f) > 1e-6f;

        // ─── Stage 1+2+3: base RGB → flow sat/bright → highlight sat ───
        for (int y = 0; y < HEIGHT; y++) {
            for (int xc = 0; xc < WIDTH; xc++) {
                const float vu = u[y][xc];
                const float vv = v[y][xc];
                float vmag = fl::sqrtf(vu * vu + vv * vv) * 1.8f;
                if (vmag > 255.0f) vmag = 255.0f;

                // Normalize base RGB to [0,1] with velocity-glow add
                float r = (gR[y][xc] + vmag * 0.08f) * (1.0f / 255.0f);
                float g = (gG[y][xc] + vmag * 0.08f) * (1.0f / 255.0f);
                float b = (gB[y][xc] + vmag * 0.08f) * (1.0f / 255.0f);
                r = clampf(r, 0.0f, 1.0f);
                g = clampf(g, 0.0f, 1.0f);
                b = clampf(b, 0.0f, 1.0f);

                // Velocity influence: normalize then sqrt (matches ns_3 draw())
                float vn = fl::sqrtf(vmag * (1.0f / 255.0f));

                // Flow saturation + brightness around Rec.709 luminance.
                // 0.94 base brightness intentionally slightly < 1.0 — at zero velocity
                // the image is mildly muted, fast regions push it brighter.
                float lum = r * 0.2126f + g * 0.7152f + b * 0.0722f;
                float satBoost    = 1.0f + vn * render.flowSat;
                float brightBoost = 0.94f + vn * render.flowBright;
                r = (lum + (r - lum) * satBoost) * brightBoost;
                g = (lum + (g - lum) * satBoost) * brightBoost;
                b = (lum + (b - lum) * satBoost) * brightBoost;

                // Highlight pass: second sat boost on bright pixels (max-channel > 0.42)
                float maxChan = r;
                if (g > maxChan) maxChan = g;
                if (b > maxChan) maxChan = b;
                float highlight = clampf((maxChan - 0.42f) * (1.0f / 0.58f), 0.0f, 1.0f);
                float hsBoost = 1.0f + highlight * render.highlightSat;
                lum = r * 0.2126f + g * 0.7152f + b * 0.0722f;
                tR[y][xc] = lum + (r - lum) * hsBoost;
                tG[y][xc] = lum + (g - lum) * hsBoost;
                tB[y][xc] = lum + (b - lum) * hsBoost;
            }
        }

        // ─── Stage 4: per-channel glow (5-tap blur of values > 0.55) ───
        // Process each channel using pressure as glow_init, divergence as blurred.
        const float gs = render.glowStrength;
        if (gs > 0.0f) {
            float (*channels[3])[WIDTH] = { tR, tG, tB };
            for (int ch = 0; ch < 3; ch++) {
                float (*chan)[WIDTH] = channels[ch];

                // glow_init = max(chan - 0.55, 0)
                for (int y = 0; y < HEIGHT; y++) {
                    for (int xc = 0; xc < WIDTH; xc++) {
                        float v_ = chan[y][xc] - 0.55f;
                        pressure[y][xc] = (v_ < 0.0f) ? 0.0f : v_;
                    }
                }
                // 5-tap blur (0.42 center, 0.145 cardinal neighbors). Edge cells
                // sample the center for the missing neighbor (clamp-to-edge).
                for (int y = 0; y < HEIGHT; y++) {
                    for (int xc = 0; xc < WIDTH; xc++) {
                        float c  = pressure[y][xc];
                        float n  = (y > 0)          ? pressure[y - 1][xc] : c;
                        float s  = (y < HEIGHT - 1) ? pressure[y + 1][xc] : c;
                        float w_ = (xc > 0)         ? pressure[y][xc - 1] : c;
                        float e  = (xc < WIDTH - 1) ? pressure[y][xc + 1] : c;
                        divergence[y][xc] = c * 0.42f + (n + s + w_ + e) * 0.145f;
                    }
                }
                // Add blurred glow * strength back to channel
                for (int y = 0; y < HEIGHT; y++) {
                    for (int xc = 0; xc < WIDTH; xc++) {
                        chan[y][xc] += divergence[y][xc] * gs;
                    }
                }
            }
        }

        // ─── Stage 5+6+7: black point → gamma → Bayer dither → LEDs ───
        for (uint8_t y = 0; y < HEIGHT; y++) {
            for (uint8_t xc = 0; xc < WIDTH; xc++) {
                uint16_t idx = xyFunc(xc, y);
                if (idx >= NUM_LEDS) continue;

                float r = clampf((tR[y][xc] - render.blackPoint) * invBlack, 0.0f, 1.0f);
                float g = clampf((tG[y][xc] - render.blackPoint) * invBlack, 0.0f, 1.0f);
                float b = clampf((tB[y][xc] - render.blackPoint) * invBlack, 0.0f, 1.0f);

                if (applyGamma) {
                    // Gamma curve via fastpow (~5% error, base in [0,1] which is satisfied here).
                    r = fastpow(r, gamma);
                    g = fastpow(g, gamma);
                    b = fastpow(b, gamma);
                }

                leds[idx].r = f2u8d(r * 255.0f, xc, y);
                leds[idx].g = f2u8d(g * 255.0f, xc, y);
                leds[idx].b = f2u8d(b * 255.0f, xc, y);
            }
        }
    } // renderFluidToLeds()

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

    // ═══════════════════════════════════════════════════════════════════
    //  DISPATCH TABLES
    // ═══════════════════════════════════════════════════════════════════
    //
    //  Each emitter exposes a prepare-modulators function (Phase 1 of
    //  frame: writes timer slot ratios) and an emit function (Phase 3:
    //  reads modulator output, splats dye/velocity). Indexed by
    //  Emitter enum from componentEnums.h. Order MUST match the enum.
    //
    //  Flow and obstacle dispatch tables can be added the same way
    //  when additional flows/obstacles arrive.

    const EmitterFn EMITTER_PREPARE_MOD[] = {
        singleJetPrepareModulators,    // EMITTER_SINGLEJET = 0
        multiJetPrepareModulators,     // EMITTER_MULTIJET = 1
    };

    const EmitterFn EMITTER_RUN[] = {
        emitSingleJet,                 // EMITTER_SINGLEJET = 0
        emitMultiJet,                  // EMITTER_MULTIJET = 1
    };

    static_assert(sizeof(EMITTER_RUN) / sizeof(EMITTER_RUN[0]) == EMITTER_COUNT,
                  "EMITTER_RUN size must match EMITTER_COUNT");
    static_assert(sizeof(EMITTER_PREPARE_MOD) / sizeof(EMITTER_PREPARE_MOD[0]) == EMITTER_COUNT,
                  "EMITTER_PREPARE_MOD size must match EMITTER_COUNT");

    static uint8_t configureActiveModulatorSlots() {
        uint8_t nextSlot = 0;

        activeEmitterTimers = 0;
        activeFlowTimers = 0;
        activeObstacleTimers = 0;

        switch (activeEmitter) {
            case EMITTER_SINGLEJET:
                nextSlot = assignModSlots(singleJet, SINGLE_JET_MODS, nextSlot);
                activeEmitterTimers = modCount(SINGLE_JET_MODS);
                break;
            case EMITTER_MULTIJET:
                nextSlot = assignModSlots(jetPack, MULTIJET_MODS, nextSlot);
                activeEmitterTimers = modCount(MULTIJET_MODS);
                break;
            default:
                break;
        }

        switch (activeFlow) {
            case FLOW_SMOKE:
                nextSlot = assignModSlots(smoke, SMOKE_MODS, nextSlot);
                activeFlowTimers = modCount(SMOKE_MODS);
                break;
            default:
                break;
        }

        switch (activeObstacle) {
            case OBSTACLE_PADDLES:
                if (paddles.enable) {
                    nextSlot = assignModSlots(paddles, PADDLES_MODS, nextSlot);
                    activeObstacleTimers = modCount(PADDLES_MODS);
                }
            default:
                break;
        }

        return nextSlot;
    }

    // ═══════════════════════════════════════════════════════════════════
    //  INIT & MAIN LOOP
    // ═══════════════════════════════════════════════════════════════════

    void initfastFluid(uint16_t (*xy_func)(uint8_t, uint8_t)) {
        fastFluidInstance = true;
        xyFunc = xy_func;

        for (int y = 0; y < HEIGHT; y++)
            for (int x = 0; x < WIDTH; x++)
                gR[y][x] = gG[y][x] = gB[y][x] = 0.0f;

        lastFrameMs = fl::millis();
        lastEmitter = 255;
        lastFlow = 255;
        lastObstacle = 255;

        // Modulator system needs the noise generator
        noiseX.init(42);

        timings = timers();
        move = modulators();
        startingPalette();

    }

    // ═══════════════════════════════════════════════════════════════════
    //  cVAR BRIDGE
    // ═══════════════════════════════════════════════════════════════════

    static void pushGlobalDefaultsToCVars() {
        cGlobalSpeed = globalSpeed;
        cPaletteBlendRate = paletteBlendRate;
        cPaletteFloor = paletteFloor;
        render = RenderParams{};
        cColorContrast = render.colorContrast;
        cBlackPoint    = render.blackPoint;
        cFlowSat       = render.flowSat;
        cFlowBright    = render.flowBright;
        cGlowStrength  = render.glowStrength;
        cHighlightSat  = render.highlightSat;
    }

    // Push the active emitter's struct defaults into its cVars. Called on
    // emitter change. Mirrors pushFlowDefaultsToCVars in structure.
    static void pushEmitterDefaultsToCVars() {
        switch (activeEmitter) {
            case EMITTER_SINGLEJET: {
                singleJet = SingleJetParams{};
                cJetDensity = singleJet.jetDensity;
                cJetForce = singleJet.jetForce;
                cJetRadius = singleJet.jetRadius;
                cJetSpread = singleJet.jetSpread;
                cJetAngle = singleJet.jetAngle;
                cJetHueSpeed = singleJet.jetHueSpeed;
                cJetSwingRange = singleJet.jetSwingRange;
                cModJetForceRate = singleJet.modJetForce.modRate;
                cModJetForceLevel = singleJet.modJetForce.modLevel;
                cModJetAngleRate = singleJet.modJetAngle.modRate;
                cModJetAngleLevel = singleJet.modJetAngle.modLevel;
                cModJetSwingRate = singleJet.modJetSwing.modRate;
                cModJetSwingLevel = singleJet.modJetSwing.modLevel;
                break;
            }
            case EMITTER_MULTIJET: {
                resetMultiJetDefaults();
                cNumJets             = jetPack.numJets;
                cDirectionMode       = jetPack.directionMode;
                cColorMode           = jetPack.colorMode;
                cRadius          = jetPack.radius;
                cRadialAngleBase         = jetPack.radialAngleBase;
                cSize           = jetPack.size;
                cDensity             = jetPack.density;
                cForce               = jetPack.force;
                cDirection      = jetPack.direction;
                cHueSpeed            = jetPack.hueSpeed;
                cHueSpread           = jetPack.hueSpread;
                cVarRadius  = jetPack.varRadius;
                cVarRadialAngle = jetPack.varRadialAngle;
                cVarSize   	= jetPack.varSize;
				cVarDirection   = jetPack.varDirection;
                cVarDensity     = jetPack.varDensity;
                cVarForce       = jetPack.varForce;
                cVarHueSpeed         = jetPack.varHueSpeed;
                cModRadiusRate   = jetPack.modRadius.modRate;
                cModRadiusLevel  = jetPack.modRadius.modLevel;
                cModRadialAngleRate  = jetPack.modRadialAngle.modRate;
                cModRadialAngleLevel = jetPack.modRadialAngle.modLevel;
                cModSizeRate    = jetPack.modSize.modRate;
                cModSizeLevel   = jetPack.modSize.modLevel;
                cModDensityRate      = jetPack.modDensity.modRate;
                cModDensityLevel     = jetPack.modDensity.modLevel;
                cModDirectionRate    = jetPack.modDirection.modRate;
                cModDirectionLevel   = jetPack.modDirection.modLevel;
                cModForceRate        = jetPack.modForce.modRate;
                cModForceLevel       = jetPack.modForce.modLevel;
                cModHueSpeedRate          = jetPack.modHueSpeed.modRate;
                cModHueSpeedLevel         = jetPack.modHueSpeed.modLevel;
                break;
            }
            default: break;
        }
    }
    
    static void pushFlowDefaultsToCVars() {
        smoke = SmokeParams{};
        cViscosity = smoke.viscosity;
        cDiffusion = smoke.diffusion;
        cVelocityDissipation = smoke.velocityDissipation;
        cDyeDissipation = smoke.dyeDissipation;
        cVorticity = smoke.vorticity;
        cGravityForce = smoke.gravityForce;
        cGravityAngle = smoke.gravityAngle;
        cDiffuseIterations = smoke.diffuseIterations;
        cProjectIterations = smoke.projectIterations;
        cModVelDissipRate = smoke.modVelDissip.modRate;
        cModVelDissipLevel = smoke.modVelDissip.modLevel;
        cModDyeDissipRate = smoke.modDyeDissip.modRate;
        cModDyeDissipLevel = smoke.modDyeDissip.modLevel;
    }

    static void pushObstacleDefaultsToCVars() {
        paddles = PaddlesParams{};
        cPaddleEnable     = paddles.enable;
        cPaddleOverlay    = paddles.overlay;
        cPaddleWidth      = paddles.width;
        cPaddleSlideRate  = paddles.modSlide.modRate;
        cPaddleSlideLevel = paddles.modSlide.modLevel;
        cPaddleSoftEdge   = paddles.softEdge;
        cPaddleR          = paddles.colorR;
        cPaddleG          = paddles.colorG;
        cPaddleB          = paddles.colorB;
    }

    // Sync ALL emitters' cVars into their structs every frame. Inactive
    // emitters' state is updated harmlessly — only EMITTER_RUN[activeEmitter]
    // actually consumes them this frame.
    
    static void syncGlobalFromCVars() {
        globalSpeed = cGlobalSpeed;
        paletteBlendRate = cPaletteBlendRate;
        paletteFloor = cPaletteFloor;
        render.colorContrast = cColorContrast;
        render.blackPoint    = cBlackPoint;
        render.flowSat       = cFlowSat;
        render.flowBright    = cFlowBright;
        render.glowStrength  = cGlowStrength;
        render.highlightSat  = cHighlightSat;
    }

    static void syncEmittersFromCVars() {
        // singleJet
        singleJet.jetDensity = cJetDensity;
        singleJet.jetForce = cJetForce;
        singleJet.jetRadius = cJetRadius;
        singleJet.jetSpread = cJetSpread;
        singleJet.jetAngle = cJetAngle;
        singleJet.jetHueSpeed = cJetHueSpeed;
        singleJet.jetSwingRange = cJetSwingRange;
        singleJet.modJetForce.modRate = cModJetForceRate;
        singleJet.modJetForce.modLevel = cModJetForceLevel;
        singleJet.modJetAngle.modRate = cModJetAngleRate;
        singleJet.modJetAngle.modLevel = cModJetAngleLevel;
        singleJet.modJetSwing.modRate = cModJetSwingRate;
        singleJet.modJetSwing.modLevel = cModJetSwingLevel;

        // multiJet
        jetPack.numJets = cNumJets;
        jetPack.directionMode = cDirectionMode;
        jetPack.colorMode = cColorMode;
        jetPack.radialAngleBase = cRadialAngleBase;
        jetPack.density = cDensity;
        jetPack.force = cForce;
        jetPack.size = cSize;
        jetPack.radius = cRadius;
        jetPack.direction = cDirection;
        jetPack.hueSpeed = cHueSpeed;
        jetPack.hueSpread = cHueSpread;
        jetPack.varRadialAngle = cVarRadialAngle;
        jetPack.varRadius = cVarRadius;
        jetPack.varDirection = cVarDirection;
        jetPack.varSize = cVarSize;
        jetPack.varForce = cVarForce;
        jetPack.varDensity = cVarDensity;
        jetPack.varHueSpeed = cVarHueSpeed;
        jetPack.modRadialAngle.modRate = cModRadialAngleRate;
        jetPack.modRadialAngle.modLevel = cModRadialAngleLevel;
        jetPack.modRadius.modRate = cModRadiusRate;
        jetPack.modRadius.modLevel = cModRadiusLevel;
        jetPack.modDirection.modRate = cModDirectionRate;
        jetPack.modDirection.modLevel = cModDirectionLevel;
        jetPack.modSize.modRate = cModSizeRate;
        jetPack.modSize.modLevel = cModSizeLevel;
        jetPack.modForce.modRate = cModForceRate;
        jetPack.modForce.modLevel = cModForceLevel;
        jetPack.modDensity.modRate = cModDensityRate;
        jetPack.modDensity.modLevel = cModDensityLevel;
        jetPack.modHueSpeed.modRate = cModHueSpeedRate;
        jetPack.modHueSpeed.modLevel = cModHueSpeedLevel;
    }

    static void syncFlowFromCVars() {
        smoke.viscosity = cViscosity;
        smoke.diffusion = cDiffusion;
        smoke.velocityDissipation = cVelocityDissipation;
        smoke.dyeDissipation = cDyeDissipation;
        smoke.vorticity = cVorticity;
        smoke.gravityForce = cGravityForce;
        smoke.gravityAngle = cGravityAngle;
        smoke.diffuseIterations = cDiffuseIterations;
        smoke.projectIterations = cProjectIterations;
        smoke.modVelDissip.modRate = cModVelDissipRate;
        smoke.modVelDissip.modLevel = cModVelDissipLevel;
        smoke.modDyeDissip.modRate = cModDyeDissipRate;
        smoke.modDyeDissip.modLevel = cModDyeDissipLevel;
    }

    static void syncObstaclesFromCVars() {
        paddles.enable            = cPaddleEnable;
        paddles.overlay           = cPaddleOverlay;
        paddles.width             = cPaddleWidth;
        paddles.modSlide.modRate  = cPaddleSlideRate;
        paddles.modSlide.modLevel = cPaddleSlideLevel;
        paddles.softEdge          = cPaddleSoftEdge;
        paddles.colorR            = cPaddleR;
        paddles.colorG            = cPaddleG;
        paddles.colorB            = cPaddleB;
    }

    static void updatePaletteState() {
        if (gGradientPaletteCount == 0) {return;}

        if (cPaletteMode && cRotatePalette) {
            EVERY_N_SECONDS(15) {
                gCurrentPaletteNumber = gTargetPaletteNumber;
                gTargetPaletteNumber = addmod8(gTargetPaletteNumber, 1, gGradientPaletteCount);
                gTargetPalette = gGradientPalettes[gTargetPaletteNumber];
            }
        }

        EVERY_N_MILLISECONDS(40) {
            if (gCurrentPalette != gTargetPalette) {
                nblendPalette32TowardPalette32(gCurrentPalette, gTargetPalette, cPaletteBlendRate);
            } else {
                gCurrentPaletteNumber = gTargetPaletteNumber;
            }
        }
    }

    void runfastFluid() {
        unsigned long now = fl::millis();
        float rawDt = (now - lastFrameMs) * 0.001f;
        lastFrameMs = now;
        dt = rawDt * globalSpeed;
        t += dt;

        pushGlobalDefaultsToCVars();

        // First-time setup of emitter/flow state. Only one of each in fastFluid,
        // but keep the trigger pattern to fire defaults push exactly once at start.
        if (EMITTER < EMITTER_COUNT && EMITTER != lastEmitter) {
            activeEmitter = (Emitter)EMITTER;
            lastEmitter = EMITTER;
            pushEmitterDefaultsToCVars();
            sendEmitterState();
        }

        if (FLOW < FLOW_COUNT && FLOW != lastFlow) {
            activeFlow = (Flow)FLOW;
            lastFlow = FLOW;
            pushFlowDefaultsToCVars();
            sendFlowState();
        }

        if (OBSTACLE < OBSTACLE_COUNT && OBSTACLE != lastObstacle) {
            activeObstacle = (Obstacle)OBSTACLE;
            lastObstacle = OBSTACLE;
            pushObstacleDefaultsToCVars();
            sendObstacleState();
        }

        syncGlobalFromCVars();
        syncEmittersFromCVars();
        syncFlowFromCVars();
        syncObstaclesFromCVars();

        updatePaletteState();
        uint8_t totalActiveTimers = configureActiveModulatorSlots();

        // ─── Modulator pipeline (Phase 4.5 consolidation) ──────────
        // 1. Each component writes its slot ratios. 2. Single
        // calculate_modulators pass. 3. Components run, reading move[*].
        // Avoids the previous shared-slot fragility (flow + emitter
        // both writing slots 0+1, working only by capture-before-overwrite).
        EMITTER_PREPARE_MOD[activeEmitter]();
        smokePrepareModulators();
        if (activeObstacleTimers > 0) {
            paddlesPrepareModulators();
        }
        calculate_modulators(timings, totalActiveTimers);

        // Pipeline: obstacle → prepare → emit → advect → render → overlay
        updateObstacle(t);
        smokePrepare();
        PROFILE_START("emitter");
        EMITTER_RUN[activeEmitter]();
        PROFILE_END();

        PROFILE_START("fluidAdvect");
        smokeAdvect();
        PROFILE_END();

        PROFILE_START("render");
        if (cDebugView == DEBUG_VIEW_COLOR) {
            renderFluidToLeds();
        } else {
            renderDebugView(cDebugView);
        }
        PROFILE_END();

        // Obstacle overlay: write color over the rendered dye at solid
        // cells. Generic — reads obstacleCommon (mirrored from the active
        // obstacle generator each frame). Solver-enforce and overlay are
        // independently toggleable so the user can verify dye actually
        // deflects (overlay off, BC on) vs. is just hidden.
        PROFILE_START("applyObstacleOverlay");
        applyObstacleOverlay();
        PROFILE_END();

        if (cDebugView == DEBUG_VIEW_EMITTER_OVERLAY) {
            PROFILE_START("emitterOverlay");
            drawEmitterDebugOverlay();
            PROFILE_END();
        }
    } // runfastFluid() 

} // namespace fastFluid
