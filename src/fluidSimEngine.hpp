#pragma once

#include "parameterSchema.h"
#include "fluidSimTypes.h"
#include "flows/flow_fluid.h"
#include "emitters/emitter_fluidJet.h"
#include "modulators.h"

namespace fluidSim {

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
    }

    // ═══════════════════════════════════════════════════════════════════
    //  cVAR BRIDGE
    // ═══════════════════════════════════════════════════════════════════

    static void pushFlowDefaultsToCVars() {
        fluid = FluidParams{};
        cViscosity = fluid.viscosity;
        cDiffusion = fluid.diffusion;
        cVelocityDissipation = fluid.velocityDissipation;
        cDyeDissipation = fluid.dyeDissipation;
        cVorticity = fluid.vorticity;
        cGravity = fluid.gravity;
        cSolverIterations = (float)fluid.solverIterations;
        cModVelDissipRate = fluid.modVelDissip.modRate;
        cModVelDissipLevel = fluid.modVelDissip.modLevel;
        cModDyeDissipRate = fluid.modDyeDissip.modRate;
        cModDyeDissipLevel = fluid.modDyeDissip.modLevel;
    }

    static void syncFlowFromCVars() {
        fluid.viscosity = cViscosity;
        fluid.diffusion = cDiffusion;
        fluid.velocityDissipation = cVelocityDissipation;
        fluid.dyeDissipation = cDyeDissipation;
        fluid.vorticity = cVorticity;
        fluid.gravity = cGravity;
        fluid.solverIterations = (uint8_t)cSolverIterations;
        fluid.modVelDissip.modRate = cModVelDissipRate;
        fluid.modVelDissip.modLevel = cModVelDissipLevel;
        fluid.modDyeDissip.modRate = cModDyeDissipRate;
        fluid.modDyeDissip.modLevel = cModDyeDissipLevel;
    }

    static void pushDefaultsToCVars() {
        // Universal
        cGlobalSpeed = globalSpeed;
        cPersistence = floorf(persistence);
        cPersistFine = persistence - cPersistence;
        cColorShift = colorShift;
        // Emitter: fluidJet
        cJetDensity = fluidJet.jetDensity;
        cJetForce = fluidJet.jetForce;
        cJetRadius = fluidJet.jetRadius;
        cJetSpread = fluidJet.jetSpread;
        cJetAngle = fluidJet.jetAngle;
        cJetHueSpeed = fluidJet.jetHueSpeed;
        cModJetForceRate = fluidJet.modJetForce.modRate;
        cModJetForceLevel = fluidJet.modJetForce.modLevel;
        cModAngleRate = fluidJet.modAngle.modRate;
        cModAngleLevel = fluidJet.modAngle.modLevel;
    }

    static void syncFromCVars() {
        globalSpeed = cGlobalSpeed;
        persistence = cPersistence + cPersistFine;
        colorShift = cColorShift;

        fluidJet.jetDensity = cJetDensity;
        fluidJet.jetForce = cJetForce;
        fluidJet.jetRadius = cJetRadius;
        fluidJet.jetSpread = cJetSpread;
        fluidJet.jetAngle = cJetAngle;
        fluidJet.jetHueSpeed = cJetHueSpeed;
        fluidJet.modJetForce.modRate = cModJetForceRate;
        fluidJet.modJetForce.modLevel = cModJetForceLevel;
        fluidJet.modAngle.modRate = cModAngleRate;
        fluidJet.modAngle.modLevel = cModAngleLevel;

        syncFlowFromCVars();
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

        // Pipeline: prepare → emit → advect → copy out
        fluidPrepare();
        emitFluidJet();
        fluidAdvect();

        // Float grid → LED array (Bayer-dithered to break uint8 banding)
        for (uint8_t y = 0; y < HEIGHT; y++) {
            for (uint8_t x = 0; x < WIDTH; x++) {
                uint16_t idx = xyFunc(x, y);
                if (idx >= NUM_LEDS) continue;
                leds[idx].r = f2u8d(gR[y][x], x, y);
                leds[idx].g = f2u8d(gG[y][x], x, y);
                leds[idx].b = f2u8d(gB[y][x], x, y);
            }
        }
    }

} // namespace fluidSim
