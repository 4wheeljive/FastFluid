#pragma once

// ============================================================================
//  MULTI-JET EMITTER - emitter_jet.h
// ============================================================================

#include "fastFluidTypes.h"
#include "modulators.h"
#include "../emitters.h"
#include "../jets.h"

namespace fastFluid {
namespace multiJet {

    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    static constexpr uint8_t MAX_NUM_JETS = 5;
    static constexpr float MULTIJET_INV_2PI = 1.0f / FF_2PI;

    float radiusSignal = 0.0f;
    float directionSignal = 0.0f;
    float forceSignal = 0.0f;
    float hueSpeedSignal = 0.0f;
    float fxNoiseSignal = 0.0f;
    float fxSineSignal = 0.0f;
    float fxDelaySignal = 0.0f;

    static constexpr uint16_t FX_NOISE_HISTORY_MS = 20000;
    static constexpr uint16_t FX_NOISE_HISTORY_SAMPLE_MS = 20;
    static constexpr uint16_t FX_NOISE_HISTORY_SAMPLES =
        (FX_NOISE_HISTORY_MS / FX_NOISE_HISTORY_SAMPLE_MS) + 1;
    float fxNoiseHistory[FX_NOISE_HISTORY_SAMPLES];
    uint16_t fxNoiseHistoryWrite = 0;
    uint16_t fxNoiseHistoryCount = 0;
    unsigned long fxNoiseHistoryLastMs = 0;
    bool fxNoiseHistorySeeded = false;

    static inline JetParams makeMultiJetParams(float radiusModScale,
                                               float directionModScale,
                                               float forceModScale) {
        JetParams params{};
        params.radiusModScale = radiusModScale;
        params.directionModScale = directionModScale;
        params.forceModScale = forceModScale;
        return params;
    }

    JetParams jet[MAX_NUM_JETS] = {
        makeMultiJetParams(1.00f, 1.00f, 1.00f),
        makeMultiJetParams(0.80f, 1.30f, 0.70f),
        makeMultiJetParams(1.20f, 0.80f, 1.15f),
        makeMultiJetParams(0.90f, 1.20f, 0.85f),
        makeMultiJetParams(1.40f, 0.60f, 1.30f)
    };

    /*
        makeMultiJetParams(1.00f, 1.00f, 1.00f),
        makeMultiJetParams(1.00f, 1.00f, 1.00f),
        makeMultiJetParams(1.00f, 1.00f, 1.00f),
        makeMultiJetParams(1.00f, 1.00f, 1.00f),
        makeMultiJetParams(1.00f, 1.00f, 1.00f)
    */

    JetPackParams jetPack;

    static constexpr ModConfig JetPackParams::* MULTIJET_MODS[] = {
        &JetPackParams::modRadius,
        &JetPackParams::modDirection,
        &JetPackParams::modForce,
        &JetPackParams::modHueSpeed,
        &JetPackParams::fxNoise,
        //&JetPackParams::fxSine,
    };
    
    static inline float wrapRad(float angle) {
        angle -= fl::floorf(angle * MULTIJET_INV_2PI) * FF_2PI;
        return angle;
    }
   
    static inline void rotateVector(float inCol, float inRow, float angle,
                                    float& outCol, float& outRow) {
        SinCosResult sc = sincos_fast(wrapRad(angle));
        outCol = inCol * sc.cos_val - inRow * sc.sin_val;
        outRow = inCol * sc.sin_val + inRow * sc.cos_val;
    }

    static void resetMultiJetDefaults() {
        jetPack = JetPackParams{};
    }

    static void prepEmitterMods() {
        timings.ratio[jetPack.modRadius.modTimer] = 0.00049f * jetPack.modRadius.modRate;
        timings.ratio[jetPack.modDirection.modTimer] = 0.0006f * jetPack.modDirection.modRate;
        //timings.ratio[jetPack.modForce.modTimer] = 0.00037f * jetPack.modForce.modRate;
        timings.ratio[jetPack.modHueSpeed.modTimer] = 0.00035f * jetPack.modHueSpeed.modRate;
        timings.ratio[jetPack.fxNoise.modTimer] = 0.0001f * jetPack.fxNoise.modRate;
		//timings.ratio[jetPack.fxSine.modTimer] = 0.0002f * jetPack.fxSine.modRate;
    }

    static inline void pushFxNoiseHistory(float sample) {
        fxNoiseHistory[fxNoiseHistoryWrite] = sample;
        fxNoiseHistoryWrite = (fxNoiseHistoryWrite + 1) % FX_NOISE_HISTORY_SAMPLES;
        if (fxNoiseHistoryCount < FX_NOISE_HISTORY_SAMPLES) {
            fxNoiseHistoryCount++;
        }
    }

    static inline void seedFxNoiseHistory(float sample, unsigned long nowMs) {
        for (uint16_t i = 0; i < FX_NOISE_HISTORY_SAMPLES; i++) {
            fxNoiseHistory[i] = sample;
        }
        fxNoiseHistoryWrite = 0;
        fxNoiseHistoryCount = 0;
        fxNoiseHistoryLastMs = nowMs - FX_NOISE_HISTORY_SAMPLE_MS;
        fxNoiseHistorySeeded = true;
    }

    static inline void updateFxNoiseHistory(float sample) {
        const unsigned long nowMs = fl::millis();
        if (!fxNoiseHistorySeeded) {
            seedFxNoiseHistory(sample, nowMs);
        }

        while ((unsigned long)(nowMs - fxNoiseHistoryLastMs) >= FX_NOISE_HISTORY_SAMPLE_MS) {
            fxNoiseHistoryLastMs += FX_NOISE_HISTORY_SAMPLE_MS;
            pushFxNoiseHistory(sample);
        }
    }

    static inline float readFxNoiseHistory(uint16_t lookbackMs) {
        if (fxNoiseHistoryCount == 0) return fxNoiseSignal;

        uint16_t slotsBack =
            (lookbackMs + (FX_NOISE_HISTORY_SAMPLE_MS / 2)) / FX_NOISE_HISTORY_SAMPLE_MS;
        const uint16_t maxSlotsBack = fxNoiseHistoryCount - 1;
        if (slotsBack > maxSlotsBack) {
            slotsBack = maxSlotsBack;
        }

        const uint16_t latestIndex =
            (fxNoiseHistoryWrite + FX_NOISE_HISTORY_SAMPLES - 1) % FX_NOISE_HISTORY_SAMPLES;
        const uint16_t readIndex =
            (latestIndex + FX_NOISE_HISTORY_SAMPLES - slotsBack) % FX_NOISE_HISTORY_SAMPLES;
        return fxNoiseHistory[readIndex];
    }

    static void acquireSignals() {
        radiusSignal = move.directional_noise[jetPack.modRadius.modTimer];
        directionSignal = move.directional_noise[jetPack.modDirection.modTimer];
        //forceSignal = move.directional_noise[jetPack.modForce.modTimer];
        hueSpeedSignal = move.directional_noise[jetPack.modHueSpeed.modTimer];
        fxNoiseSignal = move.directional_noise[jetPack.fxNoise.modTimer];
        updateFxNoiseHistory(fxNoiseSignal);
		fxSineSignal = move.directional_sine[jetPack.fxSine.modTimer];
    }

    float jetNoiseSignal(uint8_t jetIndex) {
        uint16_t noiseLookback = 5000;
        noiseLookback *= jetIndex;
        return readFxNoiseHistory(noiseLookback);
    }

    float jetSineSignal(uint8_t jetIndex) {
        return 0.0f; // placeholder until later
    }

    float jetDelaySignal(uint8_t jetIndex) {
        uint16_t delayLookback = 40;
        delayLookback *= jetIndex;
        return readFxNoiseHistory(delayLookback);
    }

    static float diffSignal(uint8_t fxMode, uint8_t jetIndex) {
		switch (fxMode) {
            case SYNCHRONOUS:  return 0.0f; break;
            case NOISE:        return jetNoiseSignal(jetIndex); break;
            case SINE:         return jetSineSignal(jetIndex); break;
            case DELAY:        return jetDelaySignal(jetIndex); break;
            default:	       return 0.0f; break;
        }
    }

    static inline uint8_t multiJetCount() {
        return jetPack.numJets > MAX_NUM_JETS ? MAX_NUM_JETS : jetPack.numJets;
    }

    static inline void resolveMultiJetAnchor(uint8_t jetIndex, uint8_t count,
                                             const JetParams& thisJet,
                                             float& anchorCol, float& anchorRow) {
        const float rawAngle = jetPack.radialAngleBase +
                               FF_2PI * ((float)jetIndex / (float)count);

        const float angle = rawAngle +
                            //angleSignal *
                            jetPack.modRadialAngle.modLevel *
                            jetPack.varRadialAngle *
                            thisJet.radialAngleModScale;


		float radiusFxDepth = 0.9f;   // how much we want each jet's radius to vary from other jets based on diffSignal  		
		float radiusFxFactor = 1.0f + diffSignal(jetPack.fxRadius, jetIndex) * radiusFxDepth; 

        float radius = jetPack.radius * 
                       (1.0f + radiusSignal * jetPack.modRadius.modLevel *
                        thisJet.radiusModScale * radiusFxFactor);
        radius = fmaxf(0.0f, radius);
        radius = clampf(radius, (float)MIN_DIMENSION * 0.2f, (float)MIN_DIMENSION * 0.45f);

        SinCosResult sc = sincos_fast(wrapRad(angle));
        anchorCol = jetPack.centerCol + sc.cos_val * radius;
        anchorRow = jetPack.centerRow + sc.sin_val * radius;
    
    } // resolveMultiJetAnchor()

    static inline void resolveMultiJetDirection(uint8_t jetIndex, const JetParams& thisJet,
                                                float anchorCol, float anchorRow,
                                                float& dirCol, float& dirRow) {
        float radialCol = anchorCol - jetPack.centerCol;
        float radialRow = anchorRow - jetPack.centerRow;
        const float length = fl::sqrtf(radialCol * radialCol + radialRow * radialRow);
        if (length > 1e-6f) {
            radialCol /= length;
            radialRow /= length;
        } else {
            radialCol = 0.0f;
            radialRow = -1.0f;
        }

        switch (jetPack.directionMode) {
            case MULTIJET_DIR_RADIAL_IN:
            case MULTIJET_DIR_AIM_CENTER:
                dirCol = -radialCol;
                dirRow = -radialRow;
                break;
            case MULTIJET_DIR_TANGENT_CW:
                dirCol = -radialRow;
                dirRow = radialCol;
                break;
            case MULTIJET_DIR_TANGENT_CCW:
                dirCol = radialRow;
                dirRow = -radialCol;
                break;
            case MULTIJET_DIR_ABSOLUTE: {
                const float direction = jetPack.direction;
                SinCosResult sc = sincos_fast(wrapRad(direction));
                dirCol = sc.sin_val;
                dirRow = -sc.cos_val;
                break;
            }
            case MULTIJET_DIR_RADIAL_OUT:
            default:
                dirCol = radialCol;
                dirRow = radialRow;
                break;
        }

        float directionFxDepth = 0.9f;   // how much we want each jet's direction to vary from other jets based on diffSignal 
        float directionFxFactor = 1.0f + diffSignal(jetPack.fxDirection, jetIndex) * directionFxDepth; 
        
        const float directionModulation = directionSignal *
                                          jetPack.modDirection.modLevel *
                                          jetPack.varDirection *
                                          thisJet.directionModScale * 
                                          directionFxFactor;
        const float rotation = (jetPack.directionMode == MULTIJET_DIR_ABSOLUTE)
            ? directionModulation
            : jetPack.direction + directionModulation;
        if (rotation != 0.0f) {
            float rotatedCol;
            float rotatedRow;
            rotateVector(dirCol, dirRow, rotation, rotatedCol, rotatedRow);
            dirCol = rotatedCol;
            dirRow = rotatedRow;
        }
    
    } // resolveMultiJetDirection()

    static inline float multiJetHueForJet(uint8_t jetIndex, uint8_t count,
                                          const JetParams& thisJet) {
        const float hueShift = hueSpeedSignal *
                               jetPack.modHueSpeed.modLevel *
                               jetPack.varHueSpeed *
                               thisJet.hueSpeedModScale;

        const float baseHue = fmodPos(t * jetPack.hueSpeed, 1.0f);

        float hueOffset = 0.0f;

        switch (jetPack.colorMode) {
            case MULTIJET_COLOR_NEAR_OPPOSITE:
                if (jetIndex == 1) hueOffset = 170.0f / 360.0f;
                else if (jetIndex == 2) hueOffset = 190.0f / 360.0f;
                else if (count > 3) hueOffset = (float)jetIndex / (float)count;
                break;
            case MULTIJET_COLOR_PER_JET:
                if (count > 0) hueOffset = (float)jetIndex / (float)count;
                break;
            case MULTIJET_COLOR_HUE_SPREAD:
            default:
                if (count > 1) {
                    hueOffset = jetPack.hueSpread * ((float)jetIndex / (float)count);
                }
                break;
        }

        return fmodPos(baseHue + hueOffset + hueShift, 1.0f);

    } // multiJetHueForJet

    static inline void emitLayeredJet(float anchorCol, float anchorRow,
                                      float dirCol, float dirRow,
                                      float hue, float density,
                                      float force, float radius) {
        if (radius <= 0.0f || density <= 0.0f) return;

        const float velX = dirCol * force;
        const float velY = dirRow * force;
        const float layerMid = radius * 0.6f;
        const float layerOut = radius * 1.1f;

        jetSplat(anchorCol, anchorRow, radius,
                 density * 0.55f,
                 velX, velY, hue);

        jetSplat(anchorCol + dirCol * layerMid, anchorRow + dirRow * layerMid, radius,
                 density * 0.30f,
                 velX * 0.82f, velY * 0.82f, hue);

        jetSplat(anchorCol + dirCol * layerOut, anchorRow + dirRow * layerOut, radius,
                 density * 0.15f,
                 velX * 0.65f, velY * 0.65f, hue);
    
    } // emitLayeredJet()

    static void runEmitter() {
        const uint8_t count = multiJetCount();
        if (count == 0) return;

        acquireSignals();

        for (uint8_t i = 0; i < count; i++) {
            
            const JetParams& thisJet = jet[i];
            if (!thisJet.enabled) continue;

            float anchorCol;
            float anchorRow;
            resolveMultiJetAnchor(i, count, thisJet, anchorCol, anchorRow);

            float dirCol;
            float dirRow;
            resolveMultiJetDirection(i, thisJet, anchorCol, anchorRow, dirCol, dirRow);

            const float hue = multiJetHueForJet(i, count, thisJet);
            
            const float density = jetPack.density;
            
            const float force = jetPack.force;
            //float force = jetPack.force * (1.0f + forceSignal * jetPack.modForce.modLevel * thisJet.forceModScale);
            //force = fmaxf(0.0f, force);
            
            const float size = jetPack.size; 

            emitLayeredJet(anchorCol, anchorRow, dirCol, dirRow,
                           hue, density, force, size);
        }
    } // runEmitter()

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace multiJet
} // namespace fastFluid
