#pragma once

// ═══════════════════════════════════════════════════════════════════
//  cVAR BRIDGE
// ═══════════════════════════════════════════════════════════════════

#include "fastFluidTypes.h"
#include "parameterSchema.h"
#include "emitters/emitter_singleJet.h"
#include "emitters/emitter_multiJet.h"
#include "flows/flow_smoke.h"
#include "obstacles/obstacle_paddles.h"

namespace fastFluid {

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
                cSize = jet.size;
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
                //cVarRadialAngle = pack.varRadialAngle;
                //cVarSize = pack.varSize;
                cVarDirection = pack.varDirection;
                //cVarDensity = pack.varDensity;
                //cVarForce = pack.varForce;
                cVarHueSpeed = pack.varHueSpeed;
                cModRadiusRate = pack.modRadius.modRate;
                cModRadiusLevel = pack.modRadius.modLevel;
                //cModRadialAngleRate = pack.modRadialAngle.modRate;
                //cModRadialAngleLevel = pack.modRadialAngle.modLevel;
                //cModSizeRate = pack.modSize.modRate;
                //cModSizeLevel = pack.modSize.modLevel;
                //cModDensityRate = pack.modDensity.modRate;
                //cModDensityLevel = pack.modDensity.modLevel;
                cModDirectionRate = pack.modDirection.modRate;
                cModDirectionLevel = pack.modDirection.modLevel;
                //cModForceRate = pack.modForce.modRate;
                //cModForceLevel = pack.modForce.modLevel;
                cModHueSpeedRate = pack.modHueSpeed.modRate;
                cModHueSpeedLevel = pack.modHueSpeed.modLevel;
                cRadiusStep = pack.radiusStep;
                cDirectionStep = pack.directionStep;
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
    
        switch (activeEmitter) {
    
            case EMITTER_SINGLEJET: {
                JetParams& jet = singleJet::jet;
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
                break;
            }
            
            case EMITTER_MULTIJET: {
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
                //multiJet::jetPack.varRadialAngle = cVarRadialAngle;
                multiJet::jetPack.varRadius = cVarRadius;
                multiJet::jetPack.varDirection = cVarDirection;
                //multiJet::jetPack.varSize = cVarSize;
                //multiJet::jetPack.varForce = cVarForce;
                //multiJet::jetPack.varDensity = cVarDensity;
                multiJet::jetPack.varHueSpeed = cVarHueSpeed;
                //multiJet::jetPack.modRadialAngle.modRate = cModRadialAngleRate;
                //multiJet::jetPack.modRadialAngle.modLevel = cModRadialAngleLevel;
                multiJet::jetPack.modRadius.modRate = cModRadiusRate;
                multiJet::jetPack.modRadius.modLevel = cModRadiusLevel;
                multiJet::jetPack.modDirection.modRate = cModDirectionRate;
                multiJet::jetPack.modDirection.modLevel = cModDirectionLevel;
                //multiJet::jetPack.modSize.modRate = cModSizeRate;
                //multiJet::jetPack.modSize.modLevel = cModSizeLevel;
                //multiJet::jetPack.modForce.modRate = cModForceRate;
                //multiJet::jetPack.modForce.modLevel = cModForceLevel;
                //multiJet::jetPack.modDensity.modRate = cModDensityRate;
                //multiJet::jetPack.modDensity.modLevel = cModDensityLevel;
                multiJet::jetPack.modHueSpeed.modRate = cModHueSpeedRate;
                multiJet::jetPack.modHueSpeed.modLevel = cModHueSpeedLevel;
                multiJet::jetPack.radiusStep = cRadiusStep;
                multiJet::jetPack.directionStep = cDirectionStep;
                break;
            }

            default: break;
        }
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

} //namespace fastFluid