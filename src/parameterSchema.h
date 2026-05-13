#pragma once

#include "FastLED.h"
#include <ArduinoJson.h>
#include <string>

#include "componentEnums.h"

bool displayOn = true;

extern uint8_t EMITTER;
extern uint8_t FLOW;
extern uint8_t OBSTACLE;

// ═══════════════════════════════════════════════════════════════════
// GLOBAL PARAMETERS
// ═══════════════════════════════════════════════════════════════════

const char* const GLOBAL_PARAMS[] PROGMEM = {
   "debugView", "globalSpeed", "paletteBlendRate", "paletteFloor",
   "colorContrast", "blackPoint", "flowSat", "flowBright",
   "glowStrength", "highlightSat" 
};

const uint8_t GLOBAL_PARAM_COUNT = 10;

// ═══════════════════════════════════════════════════════════════════
//  EMITTERS
// ═══════════════════════════════════════════════════════════════════

const char singlejet_str[] PROGMEM = "singlejet";
const char multijet_str[] PROGMEM = "multijet";

const char* const EMITTERS[] PROGMEM = {
      singlejet_str,
      multijet_str
   };

const uint8_t EMITTER_COUNTS[] = {2};

const char* const SINGLEJET_PARAMS[] PROGMEM = {
   "jetDensity", "jetForce", "jetRadius", "jetSpread", "jetHueSpeed",
   "jetSwingRange",
   "modJetForceRate", "modJetForceLevel",
   "modJetAngleRate", "modJetAngleLevel",
   "modJetSwingRate", "modJetSwingLevel"
};

const char* const MULTIJET_PARAMS[] PROGMEM = {
   "numJets", "directionMode", "colorMode",
   "radius", // "radialAngleBase", 
   "size",  "direction", "density", "force", 
   "hueSpeed", "hueSpread",
   "varRadius", //"varRadialAngle",
   /*"varSize",*/ "varDirection", //"varDensity",
   "varForce", //"varHueSpeed",
   "modRadiusRate", "modRadiusLevel",
   //"modRadialAngleRate", "modRadialAngleLevel",
   "modDirectionRate", "modDirectionLevel",
   //"modSizeRate", "modSizeLevel",
   //"modDensityRate", "modDensityLevel",
   "modForceRate", "modForceLevel",
   //"modHueSpeedRate", "modHueSpeedLevel"
};

struct EmitterParamEntry {
   const char* EmitterName;
   const char* const* params;
   uint8_t count;
};

const EmitterParamEntry EMITTER_PARAM_LOOKUP[] PROGMEM = {
   {"singlejet", SINGLEJET_PARAMS, 12},
   {"multijet", MULTIJET_PARAMS, 19},
};

static const EmitterParamEntry* getEmitterParams(uint8_t emitterIdx) {
      if (emitterIdx >= EMITTER_COUNT) return nullptr;
      return &EMITTER_PARAM_LOOKUP[emitterIdx];
}

// ═══════════════════════════════════════════════════════════════════
//  FLOWS
// ═══════════════════════════════════════════════════════════════════

const char smoke_str[] PROGMEM = "smoke";

const uint8_t FLOW_COUNTS[] = {1};

const char* const FLOWS[] PROGMEM = {
      smoke_str
   };

const char* const SMOKE_PARAMS[] PROGMEM = {
   "viscosity", "diffusion", "velocityDissipation", "dyeDissipation",
   "vorticity", "gravityForce", "gravityAngle",
   "diffuseIterations", "projectIterations",
   "modVelDissipRate", "modVelDissipLevel",
   "modDyeDissipRate", "modDyeDissipLevel",
};

struct FlowParamEntry {
   const char* FlowName;
   const char* const* params;
   uint8_t count;
};

const FlowParamEntry FLOW_PARAM_LOOKUP[] PROGMEM = {
   {"smoke", SMOKE_PARAMS, 13}
};

static const FlowParamEntry* getFlowParams(uint8_t flowIdx) {
      if (flowIdx >= FLOW_COUNT) return nullptr;
      return &FLOW_PARAM_LOOKUP[flowIdx];
}

// ═══════════════════════════════════════════════════════════════════
//  OBSTACLES
// ═══════════════════════════════════════════════════════════════════

const char paddles_str[] PROGMEM = "paddles";

const uint8_t OBSTACLE_COUNTS[] = {1};

const char* const OBSTACLES[] PROGMEM = {
      paddles_str
   };

const char* const PADDLES_PARAMS[] PROGMEM = {
   "paddleWidth", "paddleSlideRate", "paddleSlideLevel", "paddleSoftEdge",
   "paddleR", "paddleG", "paddleB"
};

struct ObstacleParamEntry {
   const char* ObstacleName;
   const char* const* params;
   uint8_t count;
};

const ObstacleParamEntry OBSTACLE_PARAM_LOOKUP[] PROGMEM = {
   {"paddles", PADDLES_PARAMS, 7}
};

static const ObstacleParamEntry* getObstacleParams(uint8_t obstacleIdx) {
      if (obstacleIdx >= OBSTACLE_COUNT) return nullptr;
      return &OBSTACLE_PARAM_LOOKUP[obstacleIdx];
}


// ═══════════════════════════════════════════════════════════════════
//  MISCELLANEOUS CONTROLS
// ═══════════════════════════════════════════════════════════════════

uint8_t cBright = 50;
uint8_t cMapping = 0;
uint8_t cOverrideMapping = 0;

uint8_t cEaseSat = 0;
uint8_t cEaseLum = 0;

// ═══════════════════════════════════════════════════════════════════
//  PARAMETER DECLARATIONS
// ═══════════════════════════════════════════════════════════════════

// GLOBAL -------------------------

uint8_t cDebugView = 0;
float cGlobalSpeed = 0.5f;

// Color settings
bool cPaletteMode = true;
bool cRotatePalette = true;
uint8_t cPaletteBlendRate = 32;
float cPaletteFloor = 0.05f;
float cColorContrast = 1.0f;
float cBlackPoint = 0.105f;
float cFlowSat = 2.0f;
float cFlowBright = 0.75f;
float cGlowStrength = 0.0f;
float cHighlightSat = 2.0f;

// EMITTER: singleJet --------------
//float cJetDensity = 30.0f;
//float cJetForce = 0.25f;
//float cJetRadius = 2.0f;
//float cJetSpread = 1.0f;
//float cJetAngle = 0.0f;
//float cJetHueSpeed = 0.69f;
//float cJetSwingRange = 4.0f;
//float cModJetForceRate = 0.5f;
//float cModJetForceLevel = 0.0f;
//float cModJetAngleRate = 0.5f;
//float cModJetAngleLevel = 0.0f;
//float cModJetSwingRate = 0.3f;
//float cModJetSwingLevel = 0.0f;

// EMITTER: multiJet --------------

// JETS: ---------------------
uint8_t cNumJets = 3;
uint8_t cDirectionMode = 0;
uint8_t cColorMode = 0;
float cRadius = (float)MIN_DIMENSION * 0.25f;
float cRadialAngleBase = 0.0f;
float cSize = (float)MIN_DIMENSION * 0.1f;
float cDensity = 30.0f;
float cForce = 0.25f;
float cDirection = 0.0f;
float cSpread = 0.0f;
float cHueSpeed = 0.25f;
float cHueSpread = 1.0f;
float cSlideRange = (float)MIN_DIMENSION * 0.125f; 
float cVarRadialAngle = 0.75f;
float cVarRadius = 0.25f;
float cVarSize = 0.25f;
float cVarDirection = 3.1415927f;
float cVarDensity = 0.25f;
float cVarForce = 0.15f;
float cVarHueSpeed = 0.1f;
float cModRadialAngleRate = 0.2f;
float cModRadialAngleLevel = 0.4f;
float cModRadiusRate = 0.4f;
float cModRadiusLevel = 1.5f;
float cModDirectionRate = 0.5f;
float cModDirectionLevel = 1.5f;
float cModSizeRate = 0.1f;
float cModSizeLevel = 0.0f;
float cModDensityRate = 0.1f;
float cModDensityLevel = 0.0f;
float cModForceRate = 0.2f;
float cModForceLevel = 0.2f;
float cModHueSpeedRate = 0.3f;
float cModHueSpeedLevel = 0.0f;
float cModSlideRate = 0.3f;
float cModSlideLevel = 0.0f;

// FLOW: smoke --------------------
uint8_t cDiffuseIterations = 6;
uint8_t cProjectIterations = 10;
float cVorticity = 0.0f;
float cViscosity = 0.0f;
float cDiffusion = 0.0f;
float cVelocityDissipation = 0.5f;
float cDyeDissipation = 0.4f;
float cModVelDissipRate = 0.5f;
float cModVelDissipLevel = 0.0f;
float cModDyeDissipRate = 0.5f;
float cModDyeDissipLevel = 0.0f;
float cGravityForce = 0.0f;
float cGravityAngle = 180.0f;

// OBSTACLE: Paddles
float cPaddleWidth      = 10.0f;
float cPaddleSlideRate  = 0.3f;     // modulator rate — how fast the noise evolves
float cPaddleSlideLevel = 0.85f;    // amplitude of slide as fraction of travel range
float cPaddleSoftEdge   = 0.22f;
float cPaddleR          = 220.0f;
float cPaddleG          = 220.0f;
float cPaddleB          = 220.0f;

// Paddles checkboxes (handled in processCheckbox via cx40/cx41)
bool  cPaddleEnable  = false;
bool  cPaddleOverlay = false;

// ═══════════════════════════════════════════════════════════════════
//  X-MACRO PARAMETER TABLE
// ═══════════════════════════════════════════════════════════════════

/*
X(float, JetDensity, 50.0f) \
   X(float, JetForce, 0.25f) \
   X(float, JetRadius, 2.0f) \
   X(float, JetSpread, 1.0f) \
   X(float, JetAngle, 0.0f) \
   X(float, JetHueSpeed, 0.7f) \
   X(float, JetSwingRange, 4.0f) \
   X(float, ModJetForceRate, 0.3f) \
   X(float, ModJetForceLevel, 0.1f) \
   X(float, ModJetAngleRate, 0.3f) \
   X(float, ModJetAngleLevel, 2.0f) \
   X(float, ModJetSwingRate, 0.3f) \
   X(float, ModJetSwingLevel, 0.0f) \
   
*/


#define PARAMETER_TABLE \
   X(uint8_t, OverrideMapping, 0) \
   X(float, GlobalSpeed, 0.5f) \
   X(float, PaletteBlendRate, 32.0f) \
   X(float, PaletteFloor, 0.05f) \
   X(uint8_t, DebugView, 0) \
   X(uint8_t, NumJets, 3) \
   X(uint8_t, DirectionMode, 0) \
   X(uint8_t, ColorMode, 0) \
   X(float, Radius, (float)MIN_DIMENSION * 0.25f) \
   X(float, RadialAngleBase, 0.0f) \
   X(float, Size, 5.0f) \
   X(float, Density, 30.0f) \
   X(float, Force, 0.25f) \
   X(float, Direction, 0.0f) \
   X(float, Spread, 0.0f) \
   X(float, SlideRange, 0.0f) \
   X(float, HueSpeed, 0.25f) \
   X(float, HueSpread, 1.0f) \
   X(float, VarRadialAngle, 0.74f) \
   X(float, VarRadius, 0.25f) \
   X(float, VarDirection, 3.1415927f) \
   X(float, VarSize, 0.25f) \
   X(float, VarDensity, 0.25f) \
   X(float, VarForce, 0.15f) \
   X(float, VarHueSpeed, 0.1f) \
   X(float, ModRadiusRate, 0.4f) \
   X(float, ModRadiusLevel, 1.5f) \
   X(float, ModRadialAngleRate, 0.2f) \
   X(float, ModRadialAngleLevel, 0.4f) \
   X(float, ModSizeRate, 0.1f) \
   X(float, ModSizeLevel, 0.0f) \
   X(float, ModDirectionRate, 0.5f) \
   X(float, ModDirectionLevel, 1.5f) \
   X(float, ModDensityRate, 0.1f) \
   X(float, ModDensityLevel, 0.0f) \
   X(float, ModForceRate, 0.2f) \
   X(float, ModForceLevel, 0.2f) \
   X(float, ModHueSpeedRate, 0.3f) \
   X(float, ModHueSpeedLevel, 0.0f) \
   X(float, ModSlideRate, 0.3f) \
   X(float, ModSlideLevel, 0.0f) \
   X(float, Viscosity, 0.0f) \
   X(float, Diffusion, 0.0f) \
   X(float, VelocityDissipation, 0.5f) \
   X(float, DyeDissipation, 0.4f) \
   X(float, Vorticity, 0.0f) \
   X(float, GravityForce, 0.0f) \
   X(float, GravityAngle, 180.0f) \
   X(uint8_t, DiffuseIterations, 6) \
   X(uint8_t, ProjectIterations, 10) \
   X(float, ModVelDissipRate, 0.5f) \
   X(float, ModVelDissipLevel, 0.0f) \
   X(float, ModDyeDissipRate, 0.5f) \
   X(float, ModDyeDissipLevel, 0.0f) \
   X(float, ColorContrast, 1.0f) \
   X(float, BlackPoint, 0.105f) \
   X(float, FlowSat, 2.0f) \
   X(float, FlowBright, 0.75f) \
   X(float, GlowStrength, 0.0f) \
   X(float, HighlightSat, 2.0f) \
   X(float, PaddleWidth, 10.0f) \
   X(float, PaddleSlideRate, 0.3f) \
   X(float, PaddleSlideLevel, 0.85f) \
   X(float, PaddleSoftEdge, 0.22f) \
   X(float, PaddleR, 220.0f) \
   X(float, PaddleG, 220.0f) \
   X(float, PaddleB, 220.0f)
