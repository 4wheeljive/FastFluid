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

    enum MultiJetSignal : uint8_t {
        MJ_SIGNAL_RADIUS = 0,
        MJ_SIGNAL_RADIAL_ANGLE,
        MJ_SIGNAL_SIZE,
        MJ_SIGNAL_DIRECTION,
        MJ_SIGNAL_DENSITY,
        MJ_SIGNAL_FORCE,
        MJ_SIGNAL_HUE_SPEED,
        MJ_SIGNAL_COUNT
    };

    float multiJetBaseSignals[MJ_SIGNAL_COUNT];

    JetParams jet[MAX_NUM_JETS];
    JetPackParams jetPack;

    struct MultiJetResolvedSignals {
        float radius = 1.0f;
        float radialAngle = 1.0f;
        float size = 1.0f;
        float direction = 1.0f;
        float density = 1.0f;
        float force = 1.0f;
        float hueSpeed = 1.0f;
    };

    static constexpr float MULTIJET_PRIMARY_SIGNAL_WEIGHT = 0.35f;
    static constexpr float MULTIJET_CROSS_SIGNAL_WEIGHT = 0.65f;
    static constexpr float MULTIJET_MIX_NORMALIZE = 0.42f;
 
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

    // Reset the pack and jet array together. Per-jet modulation personality is
    // resolved per frame in resolveMixedSignals() rather than stored as state.
    static void resetMultiJetDefaults() {
        
        jetPack = JetPackParams{};

        for (uint8_t i = 0; i < MAX_NUM_JETS; i++) {
            jet[i] = JetParams{};
            jet[i].enabled = true;
        }
    } // resetMultiJetDefaults()

    static void multiJetPrepareModulators() {
        timings.ratio[jetPack.modRadialAngle.modTimer] = 0.00043f * jetPack.modRadialAngle.modRate;
        timings.ratio[jetPack.modRadius.modTimer] = 0.00049f * jetPack.modRadius.modRate;
        timings.ratio[jetPack.modSize.modTimer] = 0.00036f * jetPack.modSize.modRate;
        timings.ratio[jetPack.modDirection.modTimer] = 0.0006f * jetPack.modDirection.modRate;
        timings.ratio[jetPack.modDensity.modTimer] = 0.00033f * jetPack.modDensity.modRate;
        timings.ratio[jetPack.modForce.modTimer] = 0.00037f * jetPack.modForce.modRate;
        timings.ratio[jetPack.modHueSpeed.modTimer] = 0.00035f * jetPack.modHueSpeed.modRate;
    }

    static void acquireSignals() {
        multiJetBaseSignals[MJ_SIGNAL_RADIUS] =
            move.directional_noise[jetPack.modRadius.modTimer];
        multiJetBaseSignals[MJ_SIGNAL_RADIAL_ANGLE] =
            move.directional_noise[jetPack.modRadialAngle.modTimer];
        multiJetBaseSignals[MJ_SIGNAL_SIZE] =
            move.directional_noise[jetPack.modSize.modTimer];
        multiJetBaseSignals[MJ_SIGNAL_DIRECTION] =
            move.directional_noise[jetPack.modDirection.modTimer];
        multiJetBaseSignals[MJ_SIGNAL_DENSITY] =
            move.directional_noise[jetPack.modDensity.modTimer];
        multiJetBaseSignals[MJ_SIGNAL_FORCE] =
            move.directional_noise[jetPack.modForce.modTimer];
        multiJetBaseSignals[MJ_SIGNAL_HUE_SPEED] =
            move.directional_noise[jetPack.modHueSpeed.modTimer];
    }

    static inline float mixWeight(uint8_t jetIndex, uint8_t propertyIndex, uint8_t sourceIndex) {
        static constexpr float weights[8] = {
            -0.95f, -0.62f, -0.33f, 0.24f,
             0.41f,  0.68f,  0.87f, -0.48f
        };
        const uint8_t idx = (jetIndex * 13u + propertyIndex * 7u + sourceIndex * 5u) & 7u;
        return weights[idx];
    }

    static inline float crossMixedSignal(uint8_t jetIndex, uint8_t propertyIndex) {
        float sum = 0.0f;
        for (uint8_t source = 0; source < MJ_SIGNAL_COUNT; source++) {
            if (source == propertyIndex) continue;
            sum += mixWeight(jetIndex, propertyIndex, source) * multiJetBaseSignals[source];
        }
        return clampf(sum * MULTIJET_MIX_NORMALIZE, -1.0f, 1.0f);
    }

    static inline float resolveMixedSignal(uint8_t jetIndex, uint8_t propertyIndex, float modScale) {
        const float primary = multiJetBaseSignals[propertyIndex];
        const float cross = crossMixedSignal(jetIndex, propertyIndex);
        const float signal = MULTIJET_PRIMARY_SIGNAL_WEIGHT * primary +
                             MULTIJET_CROSS_SIGNAL_WEIGHT * cross;
        return clampf(signal * modScale, -1.0f, 1.0f);
    }

    static inline MultiJetResolvedSignals resolveMixedSignals(uint8_t jetIndex,
                                                              const JetParams& thisJet) {
        MultiJetResolvedSignals signals;
        signals.radius = resolveMixedSignal(jetIndex, MJ_SIGNAL_RADIUS, thisJet.radiusModScale);
        signals.radialAngle = resolveMixedSignal(
            jetIndex,
            MJ_SIGNAL_RADIAL_ANGLE,
            thisJet.radialAngleModScale
        );
        signals.size = resolveMixedSignal(jetIndex, MJ_SIGNAL_SIZE, thisJet.sizeModScale);
        signals.direction = resolveMixedSignal(
            jetIndex,
            MJ_SIGNAL_DIRECTION,
            thisJet.directionModScale
        );
        signals.density = resolveMixedSignal(jetIndex, MJ_SIGNAL_DENSITY, thisJet.densityModScale);
        signals.force = resolveMixedSignal(jetIndex, MJ_SIGNAL_FORCE, thisJet.forceModScale);
        signals.hueSpeed = resolveMixedSignal(
            jetIndex,
            MJ_SIGNAL_HUE_SPEED,
            thisJet.hueSpeedModScale
        );
        return signals;
    }

    static inline uint8_t multiJetCount() {
        return jetPack.numJets > MAX_NUM_JETS ? MAX_NUM_JETS : jetPack.numJets;
    }

    static inline void resolveMultiJetAnchor(uint8_t jetIndex, uint8_t count,
                                             const JetParams& thisJet,
                                             const MultiJetResolvedSignals& signals,
                                             float& anchorCol, float& anchorRow) {
        const float rawAngle = jetPack.radialAngleBase +
                               FF_2PI * ((float)jetIndex / (float)count);

        const float angle = rawAngle +
                            signals.radialAngle *
                            jetPack.modRadialAngle.modLevel *
                            jetPack.varRadialAngle;

        float anchorRadius = multiJetScalar(
            jetPack.radius * thisJet.radiusScale,
            jetPack.modRadius,
            jetPack.varRadius,
            signals.radius,
            1.0f,
            1.0f
        );
        anchorRadius = clampf(anchorRadius, (float)MIN_DIMENSION * 0.2f, (float)MIN_DIMENSION * 0.45f);

        SinCosResult sc = sincos_fast(wrapRad(angle));
        anchorCol = jetPack.centerCol + sc.cos_val * anchorRadius;
        anchorRow = jetPack.centerRow + sc.sin_val * anchorRadius;
    } // resolveMultiJetAnchor()

    static inline void resolveMultiJetDirection(const JetParams& thisJet,
                                                const MultiJetResolvedSignals& signals,
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

        const float directionModulation = jetPack.modDirection.modLevel *
                                          jetPack.varDirection *
                                          signals.direction;
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
                                          const JetParams& thisJet,
                                          const MultiJetResolvedSignals& signals) {
        const float hueShift = jetPack.modHueSpeed.modLevel *
                               jetPack.varHueSpeed *
                               signals.hueSpeed;

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

    static void runMultiJet() {
        const uint8_t count = multiJetCount();
        if (count == 0) return;

        acquireSignals();

        for (uint8_t i = 0; i < count; i++) {
            const JetParams& thisJet = jet[i];
            if (!thisJet.enabled) continue;
            const MultiJetResolvedSignals signals = resolveMixedSignals(i, thisJet);

            float anchorCol;
            float anchorRow;
            resolveMultiJetAnchor(i, count, thisJet, signals, anchorCol, anchorRow);

            float dirCol;
            float dirRow;
            resolveMultiJetDirection(thisJet, signals, anchorCol, anchorRow, dirCol, dirRow);

            const float size = multiJetScalar(
                jetPack.size * thisJet.sizeScale,
                jetPack.modSize,
                jetPack.varSize,
                signals.size,
                1.0f,
                0.5f
            );

            const float density = multiJetScalar(
                jetPack.density * thisJet.densityScale,
                jetPack.modDensity,
                jetPack.varDensity,
                signals.density,
                1.0f,
                0.0f
            );

            const float force = multiJetScalar(
                jetPack.force * thisJet.forceScale,
                jetPack.modForce,
                jetPack.varForce,
                signals.force,
                1.0f,
                0.05f
            );

            const float hue = multiJetHueForJet(i, count, thisJet, signals);

            emitLayeredJet(anchorCol, anchorRow, dirCol, dirRow,
                           hue, density, force, size);
        }
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace fastFluid
