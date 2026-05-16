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

    static constexpr int MAX_NUM_JETS = 5;
    static constexpr float MULTIJET_INV_2PI = 1.0f / FF_2PI;

    float radiusSignal = 0.0f;
    //float wobbleSignal = 0.0f;
    float directionSignal = 0.0f;
    float forceSignal = 0.0f;
    float hueSpeedSignal = 0.0f;

    static inline JetParams makeMultiJetParams(float radiusModScale,
                                               //float wobbleModScale,
                                               float directionModScale,
                                               float forceModScale) {
        JetParams params{};
        params.radiusModScale = radiusModScale;
        //params.wobbleModScale = wobbleModScale;
        params.directionModScale = directionModScale;
        params.forceModScale = forceModScale;
        return params;
    }

    JetParams jet[MAX_NUM_JETS] = {
        makeMultiJetParams(1.00f, 1.00f, 1.00f),
        makeMultiJetParams(1.00f, 1.00f, 1.00f),
        makeMultiJetParams(1.00f, 1.00f, 1.00f),
        makeMultiJetParams(1.00f, 1.00f, 1.00f),
        makeMultiJetParams(1.00f, 1.00f, 1.00f)
    };

    /*
        makeMultiJetParams(1.00f, 1.00f, 1.00f, 1.00f),
        makeMultiJetParams(0.80f, 1.30f, 0.70f, 1.00f),
        makeMultiJetParams(1.20f, 0.80f, 1.15f, 1.00f),
        makeMultiJetParams(0.90f, 1.20f, 0.85f, 1.00f),
        makeMultiJetParams(1.40f, 0.60f, 1.30f, 1.00f)
        
    */

    JetPackParams jetPack;
    
    static inline uint8_t getNumJets() {
        return jetPack.numJets > MAX_NUM_JETS ? MAX_NUM_JETS : jetPack.numJets;
    }

    static constexpr ModConfig JetPackParams::* MULTIJET_MODS[] = {
        &JetPackParams::modRadius,
        &JetPackParams::modDirection,
        &JetPackParams::modHueSpeed,
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

    // Modulation signal processing functions -----------------------------

    static void prepEmitterMods() {
        timings.ratio[jetPack.modRadius.modTimer] = 0.00049f * jetPack.modRadius.modRate;
        timings.ratio[jetPack.modDirection.modTimer] = 0.0006f * jetPack.modDirection.modRate;
        timings.ratio[jetPack.modHueSpeed.modTimer] = 0.00035f * jetPack.modHueSpeed.modRate;
    }

    static void acquireSignals() {
        radiusSignal = move.directional_noise[jetPack.modRadius.modTimer];
        directionSignal = move.directional_noise[jetPack.modDirection.modTimer];
        hueSpeedSignal = move.directional_noise[jetPack.modHueSpeed.modTimer];
    }

    uint32_t getStep(int param) {
        switch(param) {
            case RADIUS:     return jetPack.radiusStep; break;
            case DIRECTION:  return jetPack.directionStep; break;
            //case WOBBLE:     return jetPack.diffWobble.modRate; break;  // was the step
            default:         return 0; break;
        }
    }

    float getNoiseSignal(uint8_t jetIndex, param param) {
		uint32_t t = (uint32_t)now * jetPack.modTimeScale; //16u;
        uint32_t offset = (uint32_t)jetIndex * getStep(param);
        float noiseVal = inoise16(t, t + offset) * (2.0f / 65535.0f) - 1.0f;
        return noiseVal;
	}

    static float diffSignal(int fxMode, uint8_t jetIndex, param param) {
		switch (fxMode) {
            case SYNCHRONOUS:  return 0.0f; break;
            case NOISE:        return getNoiseSignal(jetIndex, param); break;
            case SINE:         return 0.0f; break;
            case DELAY:        return 0.0f; break;
            default:	       return 0.0f; break;
        }
    }

    static inline void getJetPlacement(uint8_t jetIndex, uint8_t count,
                                             const JetParams& thisJet,
                                             float& anchorCol, float& anchorRow) {
        const float rawAngle = jetPack.radialAngleBase +
                               FF_2PI * ((float)jetIndex / (float)count);

        const float rotationOffset = t * jetPack.rotationRate * (FF_2PI / 60.0f);
        
        const float rotatedAngle = rawAngle + rotationOffset;

        //float wobbleFxDepth = 1.0f;   // how much we want each jet's wobble to vary from other jets based on diffSignal  		
		//float wobbleFxFactor = 1.0f + diffSignal(jetPack.fxWobble, jetIndex, WOBBLE) * jetPack.varWobble; //wobbleFxDepth;

        const float angle = rotatedAngle; /* +
                            wobbleSignal *                  // common wobble signal
                            jetPack.diffWobble.modLevel *   // jetPack-level magnitude of wobble   
                            //jetPack.varWobble *             // jetPack-level of wobble differentiation among jets    
                            wobbleFxFactor *                // jetPack-level of wobble differentiation among jets                
                            thisJet.wobbleModScale;         // thisJet's specified responsiveness to wobble 
                            */

		//float radiusFxDepth = 1.5f;   // how much we want each jet's radius to vary from other jets based on diffSignal  		
		float radiusFxFactor = 1.0f + diffSignal(jetPack.fxRadius, jetIndex, RADIUS) * jetPack.varRadius; // radiusFxDepth; 

        float radius = jetPack.radius * 
                       (1.0f + radiusSignal * jetPack.modRadius.modLevel *
                        thisJet.radiusModScale * radiusFxFactor);
        radius = fmaxf(0.0f, radius);
        radius = clampf(radius, (float)MIN_DIMENSION * 0.2f, (float)MIN_DIMENSION * 0.45f);

        SinCosResult sc = sincos_fast(wrapRad(angle));
        anchorCol = jetPack.centerCol + sc.cos_val * radius;
        anchorRow = jetPack.centerRow + sc.sin_val * radius;
    
    } // getJetPlacement()

    static inline void getJetDirection(uint8_t jetIndex, const JetParams& thisJet,
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

        float directionFxDepth = 1.5f;   // how much we want each jet's direction to vary from other jets based on diffSignal 
        float directionFxFactor = 1.0f + diffSignal(jetPack.fxDirection, jetIndex, DIRECTION) * directionFxDepth; 
        
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
    
    } // getJetDirection()

    static inline float getJetHue(uint8_t jetIndex, uint8_t count,
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

    } // getJetHue())

    static inline void emitJet(float anchorCol, float anchorRow,
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
    
    } // emitJet()

    // Main runEmitter loop -----------------------------------
    static void runEmitter() {
        const int count = getNumJets();
        if (count == 0) return;

        acquireSignals();

        for (int i = 0; i < count; i++) {
            
            const JetParams& thisJet = jet[i];
            if (!thisJet.enabled) continue;

            float anchorCol;
            float anchorRow;
            getJetPlacement(i, count, thisJet, anchorCol, anchorRow);

            float dirCol;
            float dirRow;
            getJetDirection(i, thisJet, anchorCol, anchorRow, dirCol, dirRow);

            const float hue = getJetHue(i, count, thisJet);
            
            const float density = jetPack.density;
            
            const float force = jetPack.force;
            //float force = jetPack.force * (1.0f + forceSignal * jetPack.modForce.modLevel * thisJet.forceModScale);
            //force = fmaxf(0.0f, force);
            
            const float size = jetPack.size; 

            emitJet(anchorCol, anchorRow, dirCol, dirRow,
                    hue, density, force, size);
        }
    } // runEmitter()

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace multiJet
} // namespace fastFluid
