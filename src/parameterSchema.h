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
   "globalSpeed", "paletteBlendRate", "paletteFloor", "debugView" 
};

const uint8_t GLOBAL_PARAM_COUNT = 4;

// ═══════════════════════════════════════════════════════════════════
//  EMITTERS
// ═══════════════════════════════════════════════════════════════════

const char singlejet_str[] PROGMEM = "singlejet";
const char threejet_str[] PROGMEM = "threejet";

const char* const EMITTERS[] PROGMEM = {
      singlejet_str,
      threejet_str
   };

const uint8_t EMITTER_COUNTS[] = {2};

const char* const SINGLEJET_PARAMS[] PROGMEM = {
   "jetDensity", "jetForce", "jetRadius", "jetSpread", "jetHueSpeed",
   "jetSwingRange",
   "modJetForceRate", "modJetForceLevel",
   "modAngleRate", "modAngleLevel",
   "modJetSwingRate", "modJetSwingLevel"
};

const char* const THREEJET_PARAMS[] PROGMEM = {
   "threeJetDensity", "threeJetForce", "threeJetRadius",
   "threeJetHueSpeed", "threeJetRingRadius", "threeJetColorMode",
   "modJetAngleRate", "modJetAngleLevel",
   "modRingRadiusRate", "modRingRadiusLevel",
   "modJetForceRate", "modJetForceLevel"
};

struct EmitterParamEntry {
   const char* EmitterName;
   const char* const* params;
   uint8_t count;
};

const EmitterParamEntry EMITTER_PARAM_LOOKUP[] PROGMEM = {
   {"fluidjet", SINGLEJET_PARAMS, 12},
   {"threejet", THREEJET_PARAMS, 12},
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
   "colorContrast", "blackPoint", "flowSat", "flowBright",
   "glowStrength", "highlightSat",
   "paddleWidth", "paddleSlideRate", "paddleSlideLevel", "paddleSoftEdge",
   "paddleR", "paddleG", "paddleB"
};

struct FlowParamEntry {
   const char* FlowName;
   const char* const* params;
   uint8_t count;
};

const FlowParamEntry FLOW_PARAM_LOOKUP[] PROGMEM = {
   {"smoke", SMOKE_PARAMS, 26}
};

static const FlowParamEntry* getFlowParams(uint8_t flowIdx) {
      if (flowIdx >= FLOW_COUNT) return nullptr;
      return &FLOW_PARAM_LOOKUP[flowIdx];
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
float cGlobalSpeed = 1.0f;
//bool cUseRainbow = false;
bool cPaletteMode = false;
bool cRotatePalette = false;
uint8_t cPaletteBlendRate = 16;
float cPaletteFloor = 0.0f;
uint8_t cDebugView = 0;

// EMITTER: singleJet --------------
float cJetDensity = 60.0f;
float cJetForce = 0.35f;
float cJetRadius = 2.0f;
float cJetSpread = 1.0f;
float cJetAngle = 0.0f;
float cJetHueSpeed = 0.69f;
float cJetSwingRange = 4.0f;
float cModForceRate = 0.5f;
float cModForceLevel = 0.0f;
float cModAngleRate = 0.5f;
float cModAngleLevel = 0.0f;
float cModSwingRate = 0.3f;
float cModSwingLevel = 0.0f;

// EMITTER: threeJet --------------
float cThreeJetDensity     = 50.0f;
float cThreeJetForce       = 0.7f;
float cThreeJetRadius      = 4.0f;
float cThreeJetHueSpeed    = 0.25f;
float cThreeJetRingRadius  = 12.0f;
uint8_t cThreeJetColorMode   = 0;     // 0=triple, 1=double, 2=orange
float cModJetAngleRate    = 0.3f;
float cModJetAngleLevel   = 1.0f;
float cModRingRadiusRate    = 0.3f;
float cModRingRadiusLevel   = 1.0f;
float cModJetForceRate    = 0.3f;
float cModJetForceLevel   = 1.0f;

// FLOW: smoke --------------------
float cViscosity = 0.0f;
float cDiffusion = 0.0f;
float cVelocityDissipation = 0.75f;
float cDyeDissipation = 0.25f;
float cVorticity = 10.0f;
float cGravityForce = 1.0f;
float cGravityAngle = 180.0f;
uint8_t cDiffuseIterations = 6;
uint8_t cProjectIterations = 20;
float cModVelDissipRate = 0.5f;
float cModVelDissipLevel = 0.0f;
float cModDyeDissipRate = 0.5f;
float cModDyeDissipLevel = 0.0f;
float cColorContrast = 0.8f;
float cBlackPoint = 0.0897f;
float cFlowSat = 0.5583f;
float cFlowBright = 0.18f;
float cGlowStrength = 0.0f;
float cHighlightSat = 0.22f;

// OBSTACLE: Paddles
float cPaddleWidth      = 10.0f;
float cPaddleSlideRate  = 0.3f;     // modulator rate — how fast the noise evolves
float cPaddleSlideLevel = 0.85f;    // amplitude of slide as fraction of travel range
float cPaddleSoftEdge   = 0.22f;
float cPaddleR          = 220.0f;
float cPaddleG          = 220.0f;
float cPaddleB          = 240.0f;

// Paddles checkboxes (handled in processCheckbox via cx40/cx41)
bool  cPaddleEnable  = false;
bool  cPaddleOverlay = false;

// ═══════════════════════════════════════════════════════════════════
//  X-MACRO PARAMETER TABLE
// ═══════════════════════════════════════════════════════════════════

#define PARAMETER_TABLE \
   X(uint8_t, OverrideMapping, 0) \
   X(float, GlobalSpeed, 0.5f) \
   X(float, PaletteBlendRate, 16.0f) \
   X(float, PaletteFloor, 0.0f) \
   X(uint8_t, DebugView, 0) \
   X(float, JetDensity, 50.0f) \
   X(float, JetForce, 0.25f) \
   X(float, JetRadius, 2.0f) \
   X(float, JetSpread, 1.0f) \
   X(float, JetAngle, 0.0f) \
   X(float, JetHueSpeed, 0.7f) \
   X(float, JetSwingRange, 4.0f) \
   X(float, ModForceRate, 0.3f) \
   X(float, ModForceLevel, 0.1f) \
   X(float, ModAngleRate, 0.3f) \
   X(float, ModAngleLevel, 2.0f) \
   X(float, ModSwingRate, 0.3f) \
   X(float, ModSwingLevel, 0.0f) \
   X(float, ThreeJetDensity, 50.0f) \
   X(float, ThreeJetForce, 0.7f) \
   X(float, ThreeJetRadius, 4.0f) \
   X(float, ThreeJetHueSpeed, 0.25f) \
   X(float, ThreeJetRingRadius, 12.0f) \
   X(uint8_t, ThreeJetColorMode, 0) \
   X(float, ModJetAngleRate, 0.3f) \
   X(float, ModJetAngleLevel, 1.0f) \
   X(float, ModRingRadiusRate, 0.3f) \
   X(float, ModRingRadiusLevel, 1.0f) \
   X(float, ModJetForceRate, 0.3f) \
   X(float, ModJetForceLevel, 1.0f) \
   X(float, Viscosity, 0.0f) \
   X(float, Diffusion, 0.0f) \
   X(float, VelocityDissipation, 0.75f) \
   X(float, DyeDissipation, 0.25f) \
   X(float, Vorticity, 10.0f) \
   X(float, GravityForce, 1.0f) \
   X(float, GravityAngle, 180.0f) \
   X(uint8_t, DiffuseIterations, 6) \
   X(uint8_t, ProjectIterations, 10) \
   X(float, ModVelDissipRate, 0.5f) \
   X(float, ModVelDissipLevel, 0.0f) \
   X(float, ModDyeDissipRate, 0.5f) \
   X(float, ModDyeDissipLevel, 0.0f) \
   X(float, ColorContrast, 0.8f) \
   X(float, BlackPoint, 0.0897f) \
   X(float, FlowSat, 0.5583f) \
   X(float, FlowBright, 0.18f) \
   X(float, GlowStrength, 0.0f) \
   X(float, HighlightSat, 0.22f) \
   X(float, PaddleWidth, 10.0f) \
   X(float, PaddleSlideRate, 0.3f) \
   X(float, PaddleSlideLevel, 0.85f) \
   X(float, PaddleSoftEdge, 0.22f) \
   X(float, PaddleR, 220.0f) \
   X(float, PaddleG, 220.0f) \
   X(float, PaddleB, 220.0f)
