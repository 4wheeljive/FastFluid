#pragma once

// ============================================================================
//  MULTI-JET EMITTER - emitter_multiJet.h
// ============================================================================

#include "fastFluidTypes.h"
#include "modulators.h"
#include "../emitters.h"
#include "../jets.h"

namespace fastFluid {
    FL_FAST_MATH_BEGIN
    FL_OPTIMIZATION_LEVEL_O3_BEGIN

    static constexpr uint8_t MAX_MULTI_JETS = 5;
    static constexpr float MULTIJET_INV_2PI = 1.0f / CT_2PI;

    JetPack multiJet;
    JetParams multiJets[MAX_MULTI_JETS];

    static constexpr ModConfig JetPack::* MULTI_JET_MODS[] = {
        &JetPack::modRadialAngle,
        &JetPack::modRingRadius,
        &JetPack::modWobble,
        &JetPack::modDirection,
        &JetPack::modJetRadius,
        &JetPack::modDensity,
        &JetPack::modForce,
        &JetPack::modHue
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
        multiJet = JetPack{};

        for (uint8_t i = 0; i < MAX_MULTI_JETS; i++) {
            multiJets[i] = JetParams{};
            multiJets[i].enabled = true;

            const float phase = (float)i * 17.0f;
            multiJets[i].radialAnglePhase = phase + 3.0f;
            multiJets[i].anchorRadiusPhase = phase + 7.0f;
            multiJets[i].wobblePhase = phase + 11.0f;
            multiJets[i].directionPhase = phase + 13.0f;
            multiJets[i].jetRadiusPhase = phase + 17.0f;
            multiJets[i].densityPhase = phase + 19.0f;
            multiJets[i].forcePhase = phase + 23.0f;
            multiJets[i].huePhase = phase + 29.0f;
        }

        multiJets[1].anchorRadiusScale = 0.92f;
        multiJets[2].anchorRadiusScale = 1.08f;
        multiJets[1].forceScale = 0.92f;
        multiJets[2].forceScale = 1.08f;
    }

    // Phase 1 of frame: write this component's timer slot ratios.
    static void multiJetPrepareModulators() {
        timings.ratio[multiJet.modRadialAngle.modTimer] = 0.00045f * multiJet.modRadialAngle.modRate;
        timings.ratio[multiJet.modRingRadius.modTimer] = 0.00045f * multiJet.modRingRadius.modRate;
        timings.ratio[multiJet.modWobble.modTimer] = 0.00045f * multiJet.modWobble.modRate;
        timings.ratio[multiJet.modDirection.modTimer] = 0.00040f * multiJet.modDirection.modRate;
        timings.ratio[multiJet.modJetRadius.modTimer] = 0.00045f * multiJet.modJetRadius.modRate;
        timings.ratio[multiJet.modDensity.modTimer] = 0.00050f * multiJet.modDensity.modRate;
        timings.ratio[multiJet.modForce.modTimer] = 0.00050f * multiJet.modForce.modRate;
        timings.ratio[multiJet.modHue.modTimer] = 0.00035f * multiJet.modHue.modRate;
    }

    static inline uint8_t multiJetCount() {
        return multiJet.numJets > MAX_MULTI_JETS ? MAX_MULTI_JETS : multiJet.numJets;
    }

    static inline void resolveMultiJetAnchor(uint8_t jetIndex, uint8_t count,
                                             const JetParams& jet,
                                             float& anchorCol, float& anchorRow) {
        if (multiJet.layoutMode == MULTIJET_LAYOUT_FREE) {
            anchorCol = jet.anchorCol;
            anchorRow = jet.anchorRow;
        } else {
            float radialAngle = multiJet.radialAngle + jet.radialAngleOffset;
            if (multiJet.layoutMode == MULTIJET_LAYOUT_RING_EVEN && count > 0) {
                radialAngle += CT_2PI * ((float)jetIndex / (float)count);
            }

            const float angleSignal = multiJetSignal(multiJet.modRadialAngle, jet.radialAnglePhase);
            radialAngle += multiJet.modRadialAngle.modLevel *
                           multiJet.varianceRadialAngle *
                           angleSignal *
                           jet.radialAngleModScale;

            const float radiusSignal = multiJetSignal(multiJet.modRingRadius, jet.anchorRadiusPhase);
            float anchorRadius = multiJetScalar(
                multiJet.ringRadius * jet.anchorRadiusScale,
                multiJet.modRingRadius,
                multiJet.varianceRingRadius,
                radiusSignal,
                jet.anchorRadiusModScale,
                1.0f
            );
            anchorRadius = clampf(anchorRadius, 1.0f, (float)MIN_DIMENSION * 0.48f);

            SinCosResult sc = sincos_fast(wrapRad(radialAngle));
            anchorCol = multiJet.centerCol + sc.cos_val * anchorRadius;
            anchorRow = multiJet.centerRow + sc.sin_val * anchorRadius;
        }

        const float wobbleSignal = multiJetSignal(multiJet.modWobble, jet.wobblePhase);
        const float wobble = multiJet.wobble *
                             multiJet.modWobble.modLevel *
                             wobbleSignal *
                             jet.wobbleModScale;
        if (wobble != 0.0f) {
            float radialCol = anchorCol - multiJet.centerCol;
            float radialRow = anchorRow - multiJet.centerRow;
            const float length = fl::sqrtf(radialCol * radialCol + radialRow * radialRow);
            if (length > 1e-6f) {
                radialCol /= length;
                radialRow /= length;
                const float tangentCol = -radialRow;
                const float tangentRow = radialCol;
                anchorCol += (radialCol + tangentCol) * wobble * 0.5f;
                anchorRow += (radialRow + tangentRow) * wobble * 0.5f;
            }
        }
    }

    static inline void resolveMultiJetDirection(const JetParams& jet,
                                                float anchorCol, float anchorRow,
                                                float& dirCol, float& dirRow) {
        float radialCol = anchorCol - multiJet.centerCol;
        float radialRow = anchorRow - multiJet.centerRow;
        const float length = fl::sqrtf(radialCol * radialCol + radialRow * radialRow);
        if (length > 1e-6f) {
            radialCol /= length;
            radialRow /= length;
        } else {
            radialCol = 0.0f;
            radialRow = -1.0f;
        }

        switch (multiJet.directionMode) {
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
                const float angle = jet.directionAngle;
                SinCosResult sc = sincos_fast(wrapRad(angle));
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

        const float directionSignal = multiJetSignal(multiJet.modDirection, jet.directionPhase);
        const float rotation = multiJet.directionAngle +
                               jet.directionOffset +
                               multiJet.modDirection.modLevel *
                               multiJet.varianceDirection *
                               directionSignal *
                               jet.directionModScale;
        if (rotation != 0.0f) {
            float rotatedCol;
            float rotatedRow;
            rotateVector(dirCol, dirRow, rotation, rotatedCol, rotatedRow);
            dirCol = rotatedCol;
            dirRow = rotatedRow;
        }
    }

    static inline float multiJetHueForJet(uint8_t jetIndex, uint8_t count,
                                          const JetParams& jet) {
        const float hueSignal = multiJetSignal(multiJet.modHue, jet.huePhase);
        const float hueShift = multiJet.modHue.modLevel *
                               multiJet.varianceHue *
                               hueSignal *
                               jet.hueModScale;

        if (multiJet.colorMode == MULTIJET_COLOR_WARM_ABSOLUTE) {
            static const float warmHues[3] = {
                0.0f / 360.0f,
                32.0f / 360.0f,
                54.0f / 360.0f
            };
            return fmodPos(warmHues[jetIndex % 3] + jet.hueOffset + hueShift, 1.0f);
        }

        const float baseHue = fmodPos(t * multiJet.hueSpeed * jet.hueSpeedScale, 1.0f);
        float offset = jet.hueOffset;

        switch (multiJet.colorMode) {
            case MULTIJET_COLOR_NEAR_OPPOSITE:
                if (jetIndex == 1) offset += 170.0f / 360.0f;
                else if (jetIndex == 2) offset += 190.0f / 360.0f;
                else if (count > 3) offset += (float)jetIndex / (float)count;
                break;
            case MULTIJET_COLOR_PER_JET:
                break;
            case MULTIJET_COLOR_HUE_SPREAD:
            default:
                if (count > 1) {
                    offset += multiJet.hueSpread * ((float)jetIndex / (float)count);
                }
                break;
        }

        return fmodPos(baseHue + offset + hueShift, 1.0f);
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
            const JetParams& jet = multiJets[i];
            if (!jet.enabled) continue;

            float anchorCol;
            float anchorRow;
            resolveMultiJetAnchor(i, count, jet, anchorCol, anchorRow);

            float dirCol;
            float dirRow;
            resolveMultiJetDirection(jet, anchorCol, anchorRow, dirCol, dirRow);

            const float radiusSignal = multiJetSignal(multiJet.modJetRadius, jet.jetRadiusPhase);
            const float jetRadius = multiJetScalar(
                multiJet.jetRadius * jet.jetRadiusScale,
                multiJet.modJetRadius,
                multiJet.varianceJetRadius,
                radiusSignal,
                jet.jetRadiusModScale,
                0.5f
            );

            const float densitySignal = multiJetSignal(multiJet.modDensity, jet.densityPhase);
            const float density = multiJetScalar(
                multiJet.density * jet.densityScale,
                multiJet.modDensity,
                multiJet.varianceDensity,
                densitySignal,
                jet.densityModScale,
                0.0f
            );

            const float forceSignal = multiJetSignal(multiJet.modForce, jet.forcePhase);
            const float force = multiJetScalar(
                multiJet.force * jet.forceScale,
                multiJet.modForce,
                multiJet.varianceForce,
                forceSignal,
                jet.forceModScale,
                0.05f
            );

            const float hue = multiJetHueForJet(i, count, jet);

            emitLayeredJet(anchorCol, anchorRow, dirCol, dirRow,
                           hue, density, force, jetRadius);
        }
    }

    FL_OPTIMIZATION_LEVEL_O3_END
    FL_FAST_MATH_END

} // namespace fastFluid
