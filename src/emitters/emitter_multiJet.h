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
    static constexpr float MULTIJET_INV_2PI = 1.0f / FF_2PI;
    static constexpr uint8_t MULTIJET_HISTORY_LEN = 32;
    static constexpr uint8_t MULTIJET_HISTORY_MASK = MULTIJET_HISTORY_LEN - 1;
    static_assert((MULTIJET_HISTORY_LEN & MULTIJET_HISTORY_MASK) == 0,
                  "MULTIJET_HISTORY_LEN must be a power of two");

    enum MultiJetSignalId : uint8_t {
        MULTIJET_SIGNAL_RADIAL_ANGLE = 0,
        MULTIJET_SIGNAL_RADIUS,
        MULTIJET_SIGNAL_SIZE,
        MULTIJET_SIGNAL_DIRECTION,
        MULTIJET_SIGNAL_DENSITY,
        MULTIJET_SIGNAL_FORCE,
        MULTIJET_SIGNAL_HUE_SPEED,
        MULTIJET_SIGNAL_COUNT
    };

    JetParams jet[MAX_NUM_JETS];
    JetPackParams jetPack;
    float multiJetSignalHistory[MULTIJET_SIGNAL_COUNT][MULTIJET_HISTORY_LEN];
    uint8_t multiJetSignalHead = 0;
    bool multiJetSignalPrimed = false;
 
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
        angle -= fl::floorf(angle * MULTIJET_INV_2PI) * FF_2PI;
        return angle;
    }

    static inline void writeMultiJetSignalSamples(uint8_t historyIndex) {
        multiJetSignalHistory[MULTIJET_SIGNAL_RADIAL_ANGLE][historyIndex] =
            move.directional_noise[jetPack.modRadialAngle.modTimer];
        multiJetSignalHistory[MULTIJET_SIGNAL_RADIUS][historyIndex] =
            move.directional_noise[jetPack.modRadius.modTimer];
        multiJetSignalHistory[MULTIJET_SIGNAL_SIZE][historyIndex] =
            move.directional_noise[jetPack.modSize.modTimer];
        multiJetSignalHistory[MULTIJET_SIGNAL_DIRECTION][historyIndex] =
            move.directional_noise[jetPack.modDirection.modTimer];
        multiJetSignalHistory[MULTIJET_SIGNAL_DENSITY][historyIndex] =
            move.directional_noise[jetPack.modDensity.modTimer];
        multiJetSignalHistory[MULTIJET_SIGNAL_FORCE][historyIndex] =
            move.directional_noise[jetPack.modForce.modTimer];
        multiJetSignalHistory[MULTIJET_SIGNAL_HUE_SPEED][historyIndex] =
            move.directional_noise[jetPack.modHueSpeed.modTimer];
    }

    static inline void captureMultiJetSignals() {
        if (!multiJetSignalPrimed) {
            for (uint8_t i = 0; i < MULTIJET_HISTORY_LEN; i++) {
                writeMultiJetSignalSamples(i);
            }
            multiJetSignalHead = 0;
            multiJetSignalPrimed = true;
            return;
        }

        multiJetSignalHead = (multiJetSignalHead + 1) & MULTIJET_HISTORY_MASK;
        writeMultiJetSignalSamples(multiJetSignalHead);
    }

    static inline float multiJetSignal(MultiJetSignalId signalId, float delayOffset) {
        if (!multiJetSignalPrimed) return 0.0f;

        const float absDelay = delayOffset < 0.0f ? -delayOffset : delayOffset;
        const uint8_t delay = ((uint8_t)absDelay) & MULTIJET_HISTORY_MASK;
        const uint8_t historyIndex = (multiJetSignalHead - delay) & MULTIJET_HISTORY_MASK;
        return clampf(multiJetSignalHistory[signalId][historyIndex], -1.0f, 1.0f);
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
    // direction, hue spread, and shared modulators with per-jet delay offsets.
    static void resetMultiJetDefaults() {
        
        jetPack = JetPackParams{};
        multiJetSignalPrimed = false;
        multiJetSignalHead = 0;

        for (uint8_t i = 0; i < MAX_NUM_JETS; i++) {
            jet[i] = JetParams{};
            jet[i].enabled = true;

            const float delayBase = (float)i * 17.0f;
            jet[i].offsetRadialAngle = delayBase + 3.0f;
            jet[i].offsetRadius = delayBase + 7.0f;
            jet[i].offsetSize = delayBase + 17.0f;
            jet[i].offsetDirection = delayBase + 13.0f;
            jet[i].offsetDensity = delayBase + 19.0f;
            jet[i].offsetForce = delayBase + 23.0f;
            jet[i].offsetHueSpeed = delayBase + 29.0f;
        }
    } // resetMultiJetDefaults()

    // Phase 1 of frame: write this component's timer slot ratios.
    static void multiJetPrepareModulators() {
        timings.ratio[jetPack.modRadialAngle.modTimer] = 0.00043f * jetPack.modRadialAngle.modRate;
        timings.ratio[jetPack.modRadius.modTimer] = 0.00049f * jetPack.modRadius.modRate;
        timings.ratio[jetPack.modSize.modTimer] = 0.00036f * jetPack.modSize.modRate;
        timings.ratio[jetPack.modDirection.modTimer] = 0.0006f * jetPack.modDirection.modRate;
        timings.ratio[jetPack.modDensity.modTimer] = 0.00033f * jetPack.modDensity.modRate;
        timings.ratio[jetPack.modForce.modTimer] = 0.00037f * jetPack.modForce.modRate;
        timings.ratio[jetPack.modHueSpeed.modTimer] = 0.00035f * jetPack.modHueSpeed.modRate;
    }

    static inline uint8_t multiJetCount() {
        return jetPack.numJets > MAX_NUM_JETS ? MAX_NUM_JETS : jetPack.numJets;
    }

    static inline void resolveMultiJetAnchor(uint8_t jetIndex, uint8_t count,
                                             const JetParams& thisJet,
                                             float& anchorCol, float& anchorRow) {
        const float rawAngle = jetPack.radialAngleBase +
                               FF_2PI * ((float)jetIndex / (float)count);

        const float angleSignal = multiJetSignal(MULTIJET_SIGNAL_RADIAL_ANGLE, thisJet.offsetRadialAngle);
        const float angle = rawAngle +
                            angleSignal *
                            jetPack.modRadialAngle.modLevel *
                            jetPack.varRadialAngle *
                            thisJet.radialAngleModScale;

        const float radiusSignal = multiJetSignal(MULTIJET_SIGNAL_RADIUS, thisJet.offsetRadius);
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

        const float directionSignal = multiJetSignal(MULTIJET_SIGNAL_DIRECTION, thisJet.offsetDirection);
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
        const float hueSignal = multiJetSignal(MULTIJET_SIGNAL_HUE_SPEED, thisJet.offsetHueSpeed);
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

        captureMultiJetSignals();

        for (uint8_t i = 0; i < count; i++) {
            const JetParams& thisJet = jet[i];
            if (!thisJet.enabled) continue;

            float anchorCol;
            float anchorRow;
            resolveMultiJetAnchor(i, count, thisJet, anchorCol, anchorRow);

            float dirCol;
            float dirRow;
            resolveMultiJetDirection(thisJet, anchorCol, anchorRow, dirCol, dirRow);

            const float sizeSignal = multiJetSignal(MULTIJET_SIGNAL_SIZE, thisJet.offsetSize);
            const float size = multiJetScalar(
                jetPack.size * thisJet.sizeScale,
                jetPack.modSize,
                jetPack.varSize,
                sizeSignal,
                thisJet.sizeModScale,
                0.5f
            );

            const float densitySignal = multiJetSignal(MULTIJET_SIGNAL_DENSITY, thisJet.offsetDensity);
            const float density = multiJetScalar(
                jetPack.density * thisJet.densityScale,
                jetPack.modDensity,
                jetPack.varDensity,
                densitySignal,
                thisJet.densityModScale,
                0.0f
            );

            const float forceSignal = multiJetSignal(MULTIJET_SIGNAL_FORCE, thisJet.offsetForce);
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
