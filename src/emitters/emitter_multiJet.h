#pragma once

// ============================================================================
//  MULTI-JET EMITTER - emitter_jet.h
// ============================================================================

#include "fastFluidTypes.h"
#include "modulators.h"
#include "../emitters.h"
#include "../jets.h"

namespace fastFluid {
    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    static constexpr uint8_t MAX_NUM_JETS = 5;
    static constexpr float MULTIJET_INV_2PI = 1.0f / CT_2PI;

    JetParams jet[MAX_NUM_JETS];
    JetPackParams jetPack;
 
    static constexpr ModConfig JetPackParams::* MULTIJET_MODS[] = {
        &JetPackParams::modRadialAngle,
        &JetPackParams::modRadius,
        &JetPackParams::modSize,
        &JetPackParams::modDirection,
        &JetPackParams::modDensity,
        &JetPackParams::modForce,
        &JetPackParams::modHueSpeed
    };

    static inline float wrapRad(float angle) {
        angle -= fl::floorf(angle * MULTIJET_INV_2PI) * CT_2PI;
        return angle;
    }

    static inline float multiJetSignal(const ModConfig& mod, float phaseOffset) {
        return clampf(2.0f * noiseX.noise(move.linear[mod.modTimer] + phaseOffset), -1.0f, 1.0f);
    }

    static inline float multiJetScalar(float base, const ModConfig& mod,
                                       float variance, float signal,
                                       float modScale, float minValue) {
        const float depth = fmaxf(0.0f, mod.modLevel) * variance * modScale;
        return fmaxf(minValue, base * (1.0f + depth * signal));
    }

    static inline void rotateVector(float inCol, float inRow, float angle,
                                    float& outCol, float& outRow) {
        SinCosResult sc = sincos_fast(wrapRad(angle));
        outCol = inCol * sc.cos_val - inRow * sc.sin_val;
        outRow = inCol * sc.sin_val + inRow * sc.cos_val;
    }

    // Reset the pack and the jet array together. The default is a generalized
    // version of the old three-jet arrangement: three ring anchors, radial-out
    // direction, hue spread, and shared modulators with per-jet phase offsets.
    static void resetMultiJetDefaults() {
        
        jetPack = JetPackParams{};

        for (uint8_t i = 0; i < MAX_NUM_JETS; i++) {
            jet[i] = JetParams{};
            jet[i].enabled = true;

            const float phase = (float)i * 17.0f;
            jet[i].offsetRadialAngle = phase + 3.0f;
            jet[i].offsetRadius = phase + 7.0f;
            jet[i].offsetSize = phase + 17.0f;
            jet[i].offsetDirection = phase + 13.0f;
            jet[i].offsetDensity = phase + 19.0f;
            jet[i].offsetForce = phase + 23.0f;
            jet[i].offsetHueSpeed = phase + 29.0f;
        }
    } // resetMultiJetDefaults()

    // Phase 1 of frame: write this component's timer slot ratios.
    static void multiJetPrepareModulators() {
        timings.ratio[jetPack.modRadialAngle.modTimer] = 0.00045f * jetPack.modRadialAngle.modRate;
        timings.ratio[jetPack.modRadius.modTimer] = 0.00045f * jetPack.modRadius.modRate;
        timings.ratio[jetPack.modSize.modTimer] = 0.00045f * jetPack.modSize.modRate;
        timings.ratio[jetPack.modDirection.modTimer] = 0.00040f * jetPack.modDirection.modRate;
        timings.ratio[jetPack.modDensity.modTimer] = 0.00050f * jetPack.modDensity.modRate;
        timings.ratio[jetPack.modForce.modTimer] = 0.00050f * jetPack.modForce.modRate;
        timings.ratio[jetPack.modHueSpeed.modTimer] = 0.00035f * jetPack.modHueSpeed.modRate;
    }

    static inline uint8_t multiJetCount() {
        return jetPack.numJets > MAX_NUM_JETS ? MAX_NUM_JETS : jetPack.numJets;
    }

    static inline void resolveMultiJetAnchor(uint8_t jetIndex, uint8_t count,
                                             const JetParams& thisJet,
                                             float& anchorCol, float& anchorRow) {
        const float rawAngle = jetPack.radialAngleBase +
                               CT_2PI * ((float)jetIndex / (float)count);

        const float angleSignal = multiJetSignal(jetPack.modRadialAngle, thisJet.offsetRadialAngle);
        const float angle = rawAngle +
                            angleSignal *
                            jetPack.modRadialAngle.modLevel *
                            jetPack.varRadialAngle *
                            thisJet.radialAngleModScale;

        const float radiusSignal = multiJetSignal(jetPack.modRadius, thisJet.offsetRadius);
        float anchorRadius = multiJetScalar(
            jetPack.radius * thisJet.radiusScale,
            jetPack.modRadius,
            jetPack.varRadius,
            radiusSignal,
            thisJet.radiusModScale,
            1.0f
        );
        anchorRadius = clampf(anchorRadius, (float)MIN_DIMENSION * 0.2f, (float)MIN_DIMENSION * 0.45f);

        SinCosResult sc = sincos_fast(wrapRad(angle));
        anchorCol = jetPack.centerCol + sc.cos_val * anchorRadius;
        anchorRow = jetPack.centerRow + sc.sin_val * anchorRadius;
    }

    static inline void resolveMultiJetDirection(const JetParams& thisJet,
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

        const float directionSignal = multiJetSignal(jetPack.modDirection, thisJet.offsetDirection);
        const float directionModulation = jetPack.modDirection.modLevel *
                                          jetPack.varDirection *
                                          directionSignal *
                                          thisJet.directionModScale;
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
    }

    static inline float multiJetHueForJet(uint8_t jetIndex, uint8_t count,
                                          const JetParams& thisJet) {
        const float hueSignal = multiJetSignal(jetPack.modHueSpeed, thisJet.offsetHueSpeed);
        const float hueShift = jetPack.modHueSpeed.modLevel *
                               jetPack.varHueSpeed *
                               hueSignal *
                               thisJet.hueSpeedModScale;

        if (jetPack.colorMode == MULTIJET_COLOR_WARM_ABSOLUTE) {
            static const float warmHues[3] = {
                0.0f / 360.0f,
                32.0f / 360.0f,
                54.0f / 360.0f
            };
            return fmodPos(warmHues[jetIndex % 3] + hueShift, 1.0f);
        }

        const float baseHue = fmodPos(t * jetPack.hueSpeed * thisJet.hueSpeedScale, 1.0f);
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
    }

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
    }

    static void emitMultiJet() {
        const uint8_t count = multiJetCount();
        if (count == 0) return;

        for (uint8_t i = 0; i < count; i++) {
            const JetParams& thisJet = jet[i];
            if (!thisJet.enabled) continue;

            float anchorCol;
            float anchorRow;
            resolveMultiJetAnchor(i, count, thisJet, anchorCol, anchorRow);

            float dirCol;
            float dirRow;
            resolveMultiJetDirection(thisJet, anchorCol, anchorRow, dirCol, dirRow);

            const float sizeSignal = multiJetSignal(jetPack.modSize, thisJet.offsetSize);
            const float size = multiJetScalar(
                jetPack.size * thisJet.sizeScale,
                jetPack.modSize,
                jetPack.varSize,
                sizeSignal,
                thisJet.sizeModScale,
                0.5f
            );

            const float densitySignal = multiJetSignal(jetPack.modDensity, thisJet.offsetDensity);
            const float density = multiJetScalar(
                jetPack.density * thisJet.densityScale,
                jetPack.modDensity,
                jetPack.varDensity,
                densitySignal,
                thisJet.densityModScale,
                0.0f
            );

            const float forceSignal = multiJetSignal(jetPack.modForce, thisJet.offsetForce);
            const float force = multiJetScalar(
                jetPack.force * thisJet.forceScale,
                jetPack.modForce,
                jetPack.varForce,
                forceSignal,
                thisJet.forceModScale,
                0.05f
            );

            const float hue = multiJetHueForJet(i, count, thisJet);

            emitLayeredJet(anchorCol, anchorRow, dirCol, dirRow,
                           hue, density, force, size);
        }
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace fastFluid
