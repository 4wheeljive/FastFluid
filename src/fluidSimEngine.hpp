#pragma once

#include "parameterSchema.h"
#include "fluidSimTypes.h"
#include "flows/flow_fluid.h"
#include "obstacles.h"
#include "obstacles/obstacle_paddles.h"
#include "emitters.h"
#include "emitters/emitter_fluidJet.h"
#include "emitters/emitter_threeJet.h"
#include "noise.h"
#include "modulators.h"

namespace fluidSim {

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
        float colorContrast = 0.8f;
        float blackPoint    = 0.0897f;
        float flowSat       = 0.5583f;
        float flowBright    = 0.18f;
        float glowStrength  = 0.0f; // was 0.24
        float highlightSat  = 0.22f;
    };

    RenderParams render;

    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

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
        fluidJetPrepareModulators,    // EMITTER_FLUIDJET = 0
        threeJetPrepareModulators,    // EMITTER_THREEJET = 1
    };

    const EmitterFn EMITTER_RUN[] = {
        emitFluidJet,                 // EMITTER_FLUIDJET = 0
        emitThreeJet,                 // EMITTER_THREEJET = 1
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
            case EMITTER_FLUIDJET:
                nextSlot = assignModSlots(fluidJet, FLUID_JET_MODS, nextSlot);
                activeEmitterTimers = modCount(FLUID_JET_MODS);
                break;
            case EMITTER_THREEJET:
                nextSlot = assignModSlots(threeJet, THREE_JET_MODS, nextSlot);
                activeEmitterTimers = modCount(THREE_JET_MODS);
                break;
            default:
                break;
        }

        switch (activeFlow) {
            case FLOW_FLUID:
                nextSlot = assignModSlots(fluid, FLUID_MODS, nextSlot);
                activeFlowTimers = modCount(FLUID_MODS);
                break;
            default:
                break;
        }

        if (paddles.enable) {
            nextSlot = assignModSlots(paddles, PADDLE_MODS, nextSlot);
            activeObstacleTimers = modCount(PADDLE_MODS);
        }

        return nextSlot;
    }

    // ═══════════════════════════════════════════════════════════════════
    //  INIT & MAIN LOOP
    // ═══════════════════════════════════════════════════════════════════

    void initFluidSim(uint16_t (*xy_func)(uint8_t, uint8_t)) {
        fluidSimInstance = true;
        xyFunc = xy_func;

        for (int y = 0; y < HEIGHT; y++)
            for (int x = 0; x < WIDTH; x++)
                gR[y][x] = gG[y][x] = gB[y][x] = 0.0f;

        lastFrameMs = fl::millis();
        lastEmitter = 255;
        lastFlow = 255;

        // Modulator system needs the noise generator
        noiseX.init(42);

        timings = timers();
        move = modulators();
        startingPalette();

    }

    // ═══════════════════════════════════════════════════════════════════
    //  cVAR BRIDGE
    // ═══════════════════════════════════════════════════════════════════

    static void pushFlowDefaultsToCVars() {
        fluid = FluidParams{};
        render = RenderParams{};
        paddles = PaddleParams{};
        cViscosity = fluid.viscosity;
        cDiffusion = fluid.diffusion;
        cVelocityDissipation = fluid.velocityDissipation;
        cDyeDissipation = fluid.dyeDissipation;
        cVorticity = fluid.vorticity;
        cGravityForce = fluid.gravityForce;
        cGravityAngle = fluid.gravityAngle;
        cDiffuseIterations = (float)fluid.diffuseIterations;
        cProjectIterations = (float)fluid.projectIterations;
        cModVelDissipRate = fluid.modVelDissip.modRate;
        cModVelDissipLevel = fluid.modVelDissip.modLevel;
        cModDyeDissipRate = fluid.modDyeDissip.modRate;
        cModDyeDissipLevel = fluid.modDyeDissip.modLevel;
        cColorContrast = render.colorContrast;
        cBlackPoint    = render.blackPoint;
        cFlowSat       = render.flowSat;
        cFlowBright    = render.flowBright;
        cGlowStrength  = render.glowStrength;
        cHighlightSat  = render.highlightSat;
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

    static void syncFlowFromCVars() {
        fluid.viscosity = cViscosity;
        fluid.diffusion = cDiffusion;
        fluid.velocityDissipation = cVelocityDissipation;
        fluid.dyeDissipation = cDyeDissipation;
        fluid.vorticity = cVorticity;
        fluid.gravityForce = cGravityForce;
        fluid.gravityAngle = cGravityAngle;
        fluid.diffuseIterations = (uint8_t)cDiffuseIterations;
        fluid.projectIterations = (uint8_t)cProjectIterations;
        fluid.modVelDissip.modRate = cModVelDissipRate;
        fluid.modVelDissip.modLevel = cModVelDissipLevel;
        fluid.modDyeDissip.modRate = cModDyeDissipRate;
        fluid.modDyeDissip.modLevel = cModDyeDissipLevel;
        render.colorContrast = cColorContrast;
        render.blackPoint    = cBlackPoint;
        render.flowSat       = cFlowSat;
        render.flowBright    = cFlowBright;
        render.glowStrength  = cGlowStrength;
        render.highlightSat  = cHighlightSat;
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

    // Push the active emitter's struct defaults into its cVars. Called on
    // emitter change. Mirrors pushFlowDefaultsToCVars in structure.
    static void pushDefaultsToCVars() {
        // Universal (always)
        cGlobalSpeed = globalSpeed;
        cPaletteBlendRate = paletteBlendRate;
        //cPersistence = fl::floorf(persistence);
        //cPersistFine = persistence - cPersistence;
        //cColorShift = colorShift;

        // Emitter-specific
        switch (activeEmitter) {
            case EMITTER_FLUIDJET: {
                fluidJet = FluidJetParams{};
                cJetDensity = fluidJet.jetDensity;
                cJetForce = fluidJet.jetForce;
                cJetRadius = fluidJet.jetRadius;
                cJetSpread = fluidJet.jetSpread;
                cJetAngle = fluidJet.jetAngle;
                cJetHueSpeed = fluidJet.jetHueSpeed;
                cJetSwingRange = fluidJet.jetSwingRange;
                cModJetForceRate = fluidJet.modJetForce.modRate;
                cModJetForceLevel = fluidJet.modJetForce.modLevel;
                cModAngleRate = fluidJet.modAngle.modRate;
                cModAngleLevel = fluidJet.modAngle.modLevel;
                cModJetSwingRate = fluidJet.modJetSwing.modRate;
                cModJetSwingLevel = fluidJet.modJetSwing.modLevel;
                break;
            }
            case EMITTER_THREEJET: {
                threeJet = ThreeJetParams{};
                cThreeJetDensity     = threeJet.density;
                cThreeJetForce       = threeJet.force;
                cThreeJetRadius      = threeJet.radius;
                cThreeJetHueSpeed    = threeJet.hueSpeed;
                cThreeJetRingRadius  = threeJet.ringRadius;
                cThreeJetColorMode   = (float)threeJet.colorMode;
                cModJet0AngleRate    = threeJet.modJet0Angle.modRate;
                cModJet0AngleLevel   = threeJet.modJet0Angle.modLevel;
                cModJet1AngleRate    = threeJet.modJet1Angle.modRate;
                cModJet1AngleLevel   = threeJet.modJet1Angle.modLevel;
                cModJet2AngleRate    = threeJet.modJet2Angle.modRate;
                cModJet2AngleLevel   = threeJet.modJet2Angle.modLevel;
                break;
            }
            default: break;
        }
    }

    // Sync ALL emitters' cVars into their structs every frame. Inactive
    // emitters' state is updated harmlessly — only EMITTER_RUN[activeEmitter]
    // actually consumes them this frame.
    static void syncFromCVars() {
        globalSpeed = cGlobalSpeed;
        //persistence = cPersistence + cPersistFine;
        //colorShift = cColorShift;
        useRainbow = cUseRainbow;
        paletteBlendRate = cPaletteBlendRate;

        // fluidJet
        fluidJet.jetDensity = cJetDensity;
        fluidJet.jetForce = cJetForce;
        fluidJet.jetRadius = cJetRadius;
        fluidJet.jetSpread = cJetSpread;
        fluidJet.jetAngle = cJetAngle;
        fluidJet.jetHueSpeed = cJetHueSpeed;
        fluidJet.jetSwingRange = cJetSwingRange;
        fluidJet.modJetForce.modRate = cModJetForceRate;
        fluidJet.modJetForce.modLevel = cModJetForceLevel;
        fluidJet.modAngle.modRate = cModAngleRate;
        fluidJet.modAngle.modLevel = cModAngleLevel;
        fluidJet.modJetSwing.modRate = cModJetSwingRate;
        fluidJet.modJetSwing.modLevel = cModJetSwingLevel;

        // threeJet
        threeJet.density    = cThreeJetDensity;
        threeJet.force      = cThreeJetForce;
        threeJet.radius     = cThreeJetRadius;
        threeJet.hueSpeed   = cThreeJetHueSpeed;
        threeJet.ringRadius = cThreeJetRingRadius;
        threeJet.colorMode  = (uint8_t)cThreeJetColorMode;
        threeJet.modJet0Angle.modRate  = cModJet0AngleRate;
        threeJet.modJet0Angle.modLevel = cModJet0AngleLevel;
        threeJet.modJet1Angle.modRate  = cModJet1AngleRate;
        threeJet.modJet1Angle.modLevel = cModJet1AngleLevel;
        threeJet.modJet2Angle.modRate  = cModJet2AngleRate;
        threeJet.modJet2Angle.modLevel = cModJet2AngleLevel;

        syncFlowFromCVars();
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

        /*//int maxChanges = (int)(cPaletteBlendRate + 0.5f);
        int maxChanges = cPaletteBlendRate;
        if (maxChanges < 1) {
            maxChanges = 1;
        } else if (maxChanges > 255) {
            maxChanges = 255;
        }*/

        EVERY_N_MILLISECONDS(40) {
            if (gCurrentPalette != gTargetPalette) {
                nblendPalette32TowardPalette32(gCurrentPalette, gTargetPalette, cPaletteBlendRate);
            } else {
                gCurrentPaletteNumber = gTargetPaletteNumber;
            }
        }
    }

    void runFluidSim() {
        unsigned long now = fl::millis();
        float rawDt = (now - lastFrameMs) * 0.001f;
        lastFrameMs = now;
        dt = rawDt * globalSpeed;
        t += dt;

        // First-time setup of emitter/flow state. Only one of each in FluidSim,
        // but keep the trigger pattern to fire defaults push exactly once at start.
        if (EMITTER < EMITTER_COUNT && EMITTER != lastEmitter) {
            activeEmitter = (Emitter)EMITTER;
            lastEmitter = EMITTER;
            pushDefaultsToCVars();
            sendEmitterState();
        }

        if (FLOW < FLOW_COUNT && FLOW != lastFlow) {
            activeFlow = (Flow)FLOW;
            lastFlow = FLOW;
            pushFlowDefaultsToCVars();
            sendFlowState();
        }

        syncFromCVars();
        updatePaletteState();
        uint8_t totalActiveTimers = configureActiveModulatorSlots();

        // ─── Modulator pipeline (Phase 4.5 consolidation) ──────────
        // 1. Each component writes its slot ratios. 2. Single
        // calculate_modulators pass. 3. Components run, reading move[*].
        // Avoids the previous shared-slot fragility (flow + emitter
        // both writing slots 0+1, working only by capture-before-overwrite).
        EMITTER_PREPARE_MOD[activeEmitter]();
        fluidPrepareModulators();
        if (activeObstacleTimers > 0) {
            paddlesPrepareModulators();
        }
        calculate_modulators(timings, totalActiveTimers);

        // Pipeline: obstacle → prepare → emit → advect → render → overlay
        updateObstacle(t);
        fluidPrepare();
        PROFILE_START("emitter");
        EMITTER_RUN[activeEmitter]();
        PROFILE_END();

        PROFILE_START("fluidAdvect");
        fluidAdvect();
        PROFILE_END();

        PROFILE_START("renderFluidToLeds");
        renderFluidToLeds();
        PROFILE_END();

        // Obstacle overlay: write color over the rendered dye at solid
        // cells. Generic — reads obstacleCommon (mirrored from the active
        // obstacle generator each frame). Solver-enforce and overlay are
        // independently toggleable so the user can verify dye actually
        // deflects (overlay off, BC on) vs. is just hidden.
        PROFILE_START("applyObstacleOverlay");
        applyObstacleOverlay();
        PROFILE_END();
    } // runFluidSim() 

} // namespace fluidSim
