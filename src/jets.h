#pragma once

// ============================================================================
//  JETS - shared multi-jet data model
// ============================================================================

#include "fastFluidTypes.h"

namespace fastFluid {

    enum MultiJetDirectionMode : uint8_t {
        MULTIJET_DIR_RADIAL_OUT = 0,
        MULTIJET_DIR_RADIAL_IN = 1,
        MULTIJET_DIR_TANGENT_CW = 2,
        MULTIJET_DIR_TANGENT_CCW = 3,
        MULTIJET_DIR_ABSOLUTE = 4,
        MULTIJET_DIR_AIM_CENTER = 5
    };

    enum MultiJetColorMode : uint8_t {
        MULTIJET_COLOR_HUE_SPREAD = 0,
        MULTIJET_COLOR_NEAR_OPPOSITE = 1,
        MULTIJET_COLOR_PER_JET = 2
    };

    struct JetParams {

        bool enabled = true;
        float anchorCol = (float)WIDTH * 0.5f;
        float anchorRow = (float)HEIGHT * 0.5f;

        // parameters used by stand-alone jet
        float radius = (float)MIN_DIMENSION * 0.25f;
        float radialAngle = 0.0f;
        float size = (float)MIN_DIMENSION * 0.1f;
        float direction = 0.0f;
        float density = 25.0f;
        float force = 0.25f;
        float hueSpeed = 0.25f;
        float spread = (float)MIN_DIMENSION * 0.05f;
        float slideRange = (float)MIN_DIMENSION *0.125f;
        ModConfig modRadialAngle = {0, 0.2f, 0.4f};
        ModConfig modRadius = {0, 0.4f, 1.5f};
        ModConfig modDirection = {0, 0.5f, 1.5f};
        ModConfig modSize = {0, 0.1f, 0.0f};
        ModConfig modDensity = {0, 0.1f, 0.0f};
        ModConfig modForce = {0, 0.2f, 0.2f};
        ModConfig modHueSpeed = {0, 0.3f, 0.0f};
        ModConfig modSpread = {0, 0.3f, 0.0f};
        ModConfig modSlideRange = {0, 0.3f, 0.0f};

        // In a multiJet emitter, the above parameters are set by 
        // a shared "jetPack". In that case, the following parameters
        // set individual jet variances     

        /*
        // Static per-jet scaling factors
        float radiusScale = 1.0f;   
        float sizeScale = 1.0f;
        float densityScale = 1.0f;
        float forceScale = 1.0f;
        float hueSpeedScale = 1.0f;
        */

        // Static per-jet modulation strength. Values near 1 keep jets synchronized;
        // small differences create related but non-identical motion.
        float radiusModScale = 1.0f;
        float radialAngleModScale = 1.0f;
        float sizeModScale = 1.0f;
        float directionModScale = 1.0f;
        float densityModScale = 1.0f;
        float forceModScale = 1.0f;
        float hueSpeedModScale = 1.0f;

        /*
        float offsetRadius = 0.0f;
        float offsetRadialAngle = 0.0f;
        float offsetSize = 0.0f;
        float offsetDirection = 0.0f;
        float offsetDensity = 0.0f;
        float offsetForce = 0.0f;
        float offsetHueSpeed = 0.0f;
        */

    };

    struct JetPackParams {
        uint8_t numJets = 3;
        uint8_t directionMode = MULTIJET_DIR_RADIAL_OUT;
        uint8_t colorMode = MULTIJET_COLOR_HUE_SPREAD;

        float centerCol = (float)WIDTH * 0.5f;
        float centerRow = (float)HEIGHT * 0.5f;

        float radialAngleBase = 0.0f;  // Rotates the evenly-spaced ring.
        float radius = (float)MIN_DIMENSION * 0.25f;
        float size = (float)MIN_DIMENSION * 0.1f;
        // Absolute angle in absolute mode; shared rotation offset otherwise.
        float direction = 0.0f;
        float density = 25.0f;
        float force = 0.25f;
        float hueSpeed = 0.25f;
        float hueSpread = 0.5f;

        // OLD CODEX COMMENT:
        //   Modulation amplitudes. Angle fields are radians; scalar fields are
        //   fractional depth around the configured base value.
        // This is wrong. These are intended to set jetPack-level factors for how significantly
        // each modulated parameter affects the individual jets overall. These work in 
        // conjunction with each jet's paramModScale factosr that set how each jet responds to
        // modulations of various parameters.     
        
        float varRadialAngle = 0.8f;
        float varRadius = 0.3f;
        float varSize = 0.3f;
        float varDirection = 0.8f; // was FF_PI;
        float varDensity = 0.3f;
        float varForce = 0.3f;
        float varHueSpeed = 0.3f;

        ModConfig modRadialAngle = {0, 0.2f, 0.4f};
        ModConfig modRadius = {0, 0.4f, 1.5f};
        ModConfig modDirection = {0, 0.5f, 1.5f};
        ModConfig modSize = {0, 0.1f, 0.0f};
        ModConfig modDensity = {0, 0.1f, 0.0f};
        ModConfig modForce = {0, 0.2f, 0.2f};
        ModConfig modHueSpeed = {0, 0.3f, 0.0f};
    };

} // namespace fastFluid
