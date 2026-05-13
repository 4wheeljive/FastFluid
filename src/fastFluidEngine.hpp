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
    static void renderFluidToLeds();
}

#include "views.h"

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
        float glowStrength  = 0.0f;
        float highlightSat  = 2.0f;
    };

    RenderParams render;

    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    static void renderFluidToLeds();



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

    const EmitterPrepFn EMITTER_PREP_MODS[] = {
        singleJet::prepEmitterMods,          // EMITTER_SINGLEJET = 0
        multiJet::prepEmitterMods          // EMITTER_MULTIJET = 1
    };

    const EmitterRunFn EMITTER_RUN[] = {
        singleJet::runEmitter,           // EMITTER_SINGLEJET = 0
        multiJet::runEmitter            // EMITTER_MULTIJET = 1
    };

    const FlowPrepModsFn FLOW_PREP_MODS[] = {
        smoke::prepFlowMods              // FLOW_SMOKE = 0
    };

    const FlowPrepFn FLOW_PREP[] = {
        smoke::prepFlow
    };

    const FlowAdvectFn FLOW_ADVECT[] = {
        smoke::advectFlow
    };

    const ObstaclePrepModsFn OBSTACLE_PREP_MODS[] = {
        paddles::prepObstacleMods
    };

    const ObstacleApplyFn OBSTACLE_APPLY[] = {
        paddles::updateObstacle
    };

    static_assert(sizeof(EMITTER_RUN) / sizeof(EMITTER_RUN[0]) == EMITTER_COUNT,
                  "EMITTER_RUN size must match EMITTER_COUNT");
    static_assert(sizeof(EMITTER_PREP_MODS) / sizeof(EMITTER_PREP_MODS[0]) == EMITTER_COUNT,
                  "EMITTER_PREP size must match EMITTER_COUNT");
    //TODO: add additional

    static uint8_t configureActiveModulatorSlots() {
        uint8_t nextSlot = 0;

        activeEmitterTimers = 0;
        activeFlowTimers = 0;
        activeObstacleTimers = 0;

        switch (activeEmitter) {
            case EMITTER_SINGLEJET:
                nextSlot = assignModSlots(singleJet::jet, singleJet::SINGLE_JET_MODS, nextSlot);
                activeEmitterTimers = modCount(singleJet::SINGLE_JET_MODS);
                break;
            case EMITTER_MULTIJET:
                nextSlot = assignModSlots(multiJet::jetPack, multiJet::MULTIJET_MODS, nextSlot);
                activeEmitterTimers = modCount(multiJet::MULTIJET_MODS);
                break;
            default:
                break;
        }

        switch (activeFlow) {
            case FLOW_SMOKE:
                nextSlot = assignModSlots(smoke::smoke, smoke::SMOKE_MODS, nextSlot);
                activeFlowTimers = modCount(smoke::SMOKE_MODS);
                break;
            default:
                break;
        }

        switch (activeObstacle) {
            case OBSTACLE_PADDLES:
                if (paddles::paddles.enable) {
                    nextSlot = assignModSlots(paddles::paddles, paddles::PADDLES_MODS, nextSlot);
                    activeObstacleTimers = modCount(paddles::PADDLES_MODS);
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
                JetParams& jet = singleJet::jet;
                cDensity = jet.density;
                cForce = jet.force;
                cRadius = jet.size;
                cSpread = jet.spread;
                cDirection = jet.direction;
                cHueSpeed = jet.hueSpeed;
                cSlideRange = jet.slideRange;
                cModForceRate = jet.modForce.modRate;
                cModForceLevel = jet.modForce.modLevel;
                cModDirectionRate = jet.modDirection.modRate;
                cModDirectionLevel = jet.modDirection.modLevel;
                cModSlideRate = jet.modSlideRange.modRate;
                cModSlideLevel = jet.modSlideRange.modLevel;
                break;
            }
            case EMITTER_MULTIJET: {
                multiJet::resetMultiJetDefaults();
                JetPackParams& pack = multiJet::jetPack;
                cNumJets = pack.numJets;
                cDirectionMode = pack.directionMode;
                cColorMode = pack.colorMode;
                cRadius = pack.radius;
                cRadialAngleBase = pack.radialAngleBase;
                cSize = pack.size;
                cDensity = pack.density;
                cForce = pack.force;
                cDirection = pack.direction;
                cHueSpeed = pack.hueSpeed;
                cHueSpread = pack.hueSpread;
                cVarRadius = pack.varRadius;
                cVarRadialAngle = pack.varRadialAngle;
                cVarSize = pack.varSize;
                cVarDirection = pack.varDirection;
                cVarDensity = pack.varDensity;
                cVarForce = pack.varForce;
                cVarHueSpeed = pack.varHueSpeed;
                cModRadiusRate = pack.modRadius.modRate;
                cModRadiusLevel = pack.modRadius.modLevel;
                cModRadialAngleRate = pack.modRadialAngle.modRate;
                cModRadialAngleLevel = pack.modRadialAngle.modLevel;
                cModSizeRate = pack.modSize.modRate;
                cModSizeLevel = pack.modSize.modLevel;
                cModDensityRate = pack.modDensity.modRate;
                cModDensityLevel = pack.modDensity.modLevel;
                cModDirectionRate = pack.modDirection.modRate;
                cModDirectionLevel = pack.modDirection.modLevel;
                cModForceRate = pack.modForce.modRate;
                cModForceLevel = pack.modForce.modLevel;
                cModHueSpeedRate = pack.modHueSpeed.modRate;
                cModHueSpeedLevel = pack.modHueSpeed.modLevel;
                break;
            }
            default: break;
        }
    }
    
    static void pushFlowDefaultsToCVars() {
        smoke::SmokeParams& smoke = smoke::smoke;
        smoke::smoke = smoke::SmokeParams{};
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
        paddles::paddles = paddles::PaddlesParams{};
        cPaddleEnable     = paddles::paddles.enable;
        cPaddleOverlay    = paddles::paddles.overlay;
        cPaddleWidth      = paddles::paddles.width;
        cPaddleSlideRate  = paddles::paddles.modSlide.modRate;
        cPaddleSlideLevel = paddles::paddles.modSlide.modLevel;
        cPaddleSoftEdge   = paddles::paddles.softEdge;
        cPaddleR          = paddles::paddles.colorR;
        cPaddleG          = paddles::paddles.colorG;
        cPaddleB          = paddles::paddles.colorB;
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
        singleJet::jet.density = cDensity;
        singleJet::jet.force = cForce;
        singleJet::jet.size = cRadius;
        singleJet::jet.spread = cSpread;
        singleJet::jet.direction = cDirection;
        singleJet::jet.hueSpeed = cHueSpeed;
        singleJet::jet.slideRange = cSlideRange;
        singleJet::jet.modForce.modRate = cModForceRate;
        singleJet::jet.modForce.modLevel = cModForceLevel;
        singleJet::jet.modDirection.modRate = cModDirectionRate;
        singleJet::jet.modDirection.modLevel = cModDirectionLevel;
        singleJet::jet.modSlideRange.modRate = cModSlideRate;
        singleJet::jet.modSlideRange.modLevel = cModSlideLevel;

        // multiJet
        multiJet::jetPack.numJets = cNumJets;
        multiJet::jetPack.directionMode = cDirectionMode;
        multiJet::jetPack.colorMode = cColorMode;
        multiJet::jetPack.radialAngleBase = cRadialAngleBase;
        multiJet::jetPack.density = cDensity;
        multiJet::jetPack.force = cForce;
        multiJet::jetPack.size = cSize;
        multiJet::jetPack.radius = cRadius;
        multiJet::jetPack.direction = cDirection;
        multiJet::jetPack.hueSpeed = cHueSpeed;
        multiJet::jetPack.hueSpread = cHueSpread;
        multiJet::jetPack.varRadialAngle = cVarRadialAngle;
        multiJet::jetPack.varRadius = cVarRadius;
        multiJet::jetPack.varDirection = cVarDirection;
        multiJet::jetPack.varSize = cVarSize;
        multiJet::jetPack.varForce = cVarForce;
        multiJet::jetPack.varDensity = cVarDensity;
        multiJet::jetPack.varHueSpeed = cVarHueSpeed;
        multiJet::jetPack.modRadialAngle.modRate = cModRadialAngleRate;
        multiJet::jetPack.modRadialAngle.modLevel = cModRadialAngleLevel;
        multiJet::jetPack.modRadius.modRate = cModRadiusRate;
        multiJet::jetPack.modRadius.modLevel = cModRadiusLevel;
        multiJet::jetPack.modDirection.modRate = cModDirectionRate;
        multiJet::jetPack.modDirection.modLevel = cModDirectionLevel;
        multiJet::jetPack.modSize.modRate = cModSizeRate;
        multiJet::jetPack.modSize.modLevel = cModSizeLevel;
        multiJet::jetPack.modForce.modRate = cModForceRate;
        multiJet::jetPack.modForce.modLevel = cModForceLevel;
        multiJet::jetPack.modDensity.modRate = cModDensityRate;
        multiJet::jetPack.modDensity.modLevel = cModDensityLevel;
        multiJet::jetPack.modHueSpeed.modRate = cModHueSpeedRate;
        multiJet::jetPack.modHueSpeed.modLevel = cModHueSpeedLevel;
    }

    static void syncFlowFromCVars() {
        smoke::smoke.viscosity = cViscosity;
        smoke::smoke.diffusion = cDiffusion;
        smoke::smoke.velocityDissipation = cVelocityDissipation;
        smoke::smoke.dyeDissipation = cDyeDissipation;
        smoke::smoke.vorticity = cVorticity;
        smoke::smoke.gravityForce = cGravityForce;
        smoke::smoke.gravityAngle = cGravityAngle;
        smoke::smoke.diffuseIterations = cDiffuseIterations;
        smoke::smoke.projectIterations = cProjectIterations;
        smoke::smoke.modVelDissip.modRate = cModVelDissipRate;
        smoke::smoke.modVelDissip.modLevel = cModVelDissipLevel;
        smoke::smoke.modDyeDissip.modRate = cModDyeDissipRate;
        smoke::smoke.modDyeDissip.modLevel = cModDyeDissipLevel;
    }

    static void syncObstaclesFromCVars() {
        paddles::paddles.enable            = cPaddleEnable;
        paddles::paddles.overlay           = cPaddleOverlay;
        paddles::paddles.width             = cPaddleWidth;
        paddles::paddles.modSlide.modRate  = cPaddleSlideRate;
        paddles::paddles.modSlide.modLevel = cPaddleSlideLevel;
        paddles::paddles.softEdge          = cPaddleSoftEdge;
        paddles::paddles.colorR            = cPaddleR;
        paddles::paddles.colorG            = cPaddleG;
        paddles::paddles.colorB            = cPaddleB;
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
        dt = rawDt * globalSpeed * 0.5f; // change takes effect one frame late
        t += dt;

        // consider way to do this only upon change
        //pushGlobalDefaultsToCVars();

        // First-time setup of emitter/flow/obstacle state. Only one of each in fastFluid,
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

        // Pipeline: Prepare and calculate modulators
        EMITTER_PREP_MODS[activeEmitter]();
        FLOW_PREP_MODS[activeFlow]();
        OBSTACLE_PREP_MODS[activeObstacle]();
        calculate_modulators(timings, totalActiveTimers);

        // Pipeline: obstacle → prepare flow → emit → advect flow → render → overlay
        OBSTACLE_APPLY[activeObstacle](); // updateObstacle()
        FLOW_PREP[activeFlow]();    //smokePrepare();

        PROFILE_START("emitter");
        EMITTER_RUN[activeEmitter]();  // runMultiJet();
        PROFILE_END();

        PROFILE_START("flowAdvect");
        FLOW_ADVECT[activeFlow]();  //smokeAdvect();
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
