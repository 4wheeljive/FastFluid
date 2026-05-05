#pragma once

#include "FastLED.h"
#include <ArduinoJson.h>
#include <string>

#include "componentEnums.h"

bool displayOn = true;

extern uint8_t EMITTER;
extern uint8_t FLOW;

// ═══════════════════════════════════════════════════════════════════
// GLOBAL PARAMETERS
// ═══════════════════════════════════════════════════════════════════

const char* const GLOBAL_PARAMS[] PROGMEM = {
   "globalSpeed", "persistence", "persistFine", "colorShift"
};

const uint8_t GLOBAL_PARAM_COUNT = 4;

// ═══════════════════════════════════════════════════════════════════
//  EMITTERS
// ═══════════════════════════════════════════════════════════════════

const char fluidjet_str[] PROGMEM = "fluidjet";

const char* const EMITTERS[] PROGMEM = {
      fluidjet_str
   };

const uint8_t EMITTER_COUNTS[] = {1};

const char* const FLUIDJET_PARAMS[] PROGMEM = {
   "jetDensity", "jetForce", "jetRadius", "jetSpread", "jetHueSpeed",
   "modJetForceRate", "modJetForceLevel",
   "modAngleRate", "modAngleLevel"
};

struct EmitterParamEntry {
   const char* EmitterName;
   const char* const* params;
   uint8_t count;
};

const EmitterParamEntry EMITTER_PARAM_LOOKUP[] PROGMEM = {
   {"fluidjet", FLUIDJET_PARAMS, 9},
};

static const EmitterParamEntry* getEmitterParams(uint8_t emitterIdx) {
      if (emitterIdx >= EMITTER_COUNT) return nullptr;
      return &EMITTER_PARAM_LOOKUP[emitterIdx];
}

// ═══════════════════════════════════════════════════════════════════
//  FLOWS
// ═══════════════════════════════════════════════════════════════════

const char fluid_str[] PROGMEM = "fluid";

const uint8_t FLOW_COUNTS[] = {1};

const char* const FLOWS[] PROGMEM = {
      fluid_str
   };

const char* const FLUID_PARAMS[] PROGMEM = {
   "viscosity", "diffusion", "velocityDissipation", "dyeDissipation",
   "vorticity", "gravityForce", "gravityAngle", "solverIterations",
   "modVelDissipRate", "modVelDissipLevel",
   "modDyeDissipRate", "modDyeDissipLevel",
   "colorContrast", "blackPoint", "flowSat", "flowBright",
   "glowStrength", "highlightSat"
};

struct FlowParamEntry {
   const char* FlowName;
   const char* const* params;
   uint8_t count;
};

const FlowParamEntry FLOW_PARAM_LOOKUP[] PROGMEM = {
   {"fluid", FLUID_PARAMS, 18}
};

static const FlowParamEntry* getFlowParams(uint8_t flowIdx) {
      if (flowIdx >= FLOW_COUNT) return nullptr;
      return &FLOW_PARAM_LOOKUP[flowIdx];
}

// ═══════════════════════════════════════════════════════════════════
//  MISCELLANEOUS CONTROLS
// ═══════════════════════════════════════════════════════════════════

uint8_t cBright = 35;
uint8_t cMapping = 0;
uint8_t cOverrideMapping = 0;

uint8_t cEaseSat = 0;
uint8_t cEaseLum = 0;

// ═══════════════════════════════════════════════════════════════════
//  PARAMETER DECLARATIONS
// ═══════════════════════════════════════════════════════════════════

// GLOBAL -------------------------
float cGlobalSpeed = 1.0f;
float cPersistence = 0.0f;
float cPersistFine = 0.05f;
float cColorShift = 0.10f;
bool cUseRainbow = false;

// EMITTER: fluidJet --------------
float cJetDensity = 60.0f;
float cJetForce = 0.35f;
float cJetRadius = 2.0f;
float cJetSpread = 1.0f;
float cJetAngle = 0.0f;
float cJetHueSpeed = 0.69f;
float cModJetForceRate = 0.5f;
float cModJetForceLevel = 0.0f;
float cModAngleRate = 0.5f;
float cModAngleLevel = 0.0f;

// FLOW: fluid --------------------
float cViscosity = 0.0f;
float cDiffusion = 0.0f;
float cVelocityDissipation = 0.5f;
float cDyeDissipation = 0.5f;
float cVorticity = 7.0f;
float cGravityForce = 1.0f;
float cGravityAngle = 90.0f;
float cSolverIterations = 5.0f;
float cModVelDissipRate = 0.5f;
float cModVelDissipLevel = 0.0f;
float cModDyeDissipRate = 0.5f;
float cModDyeDissipLevel = 0.0f;
float cColorContrast = 0.8f;
float cBlackPoint = 0.0897f;
float cFlowSat = 0.5583f;
float cFlowBright = 0.18f;
float cGlowStrength = 0.24f;
float cHighlightSat = 0.22f;

// ═══════════════════════════════════════════════════════════════════
//  X-MACRO PARAMETER TABLE
// ═══════════════════════════════════════════════════════════════════

#define PARAMETER_TABLE \
   X(uint8_t, OverrideMapping, 0) \
   X(float, GlobalSpeed, 1.0f) \
   X(float, Persistence, 0.0f) \
   X(float, PersistFine, 0.05f) \
   X(float, ColorShift, 0.10f) \
   X(float, JetDensity, 50.0f) \
   X(float, JetForce, 0.25f) \
   X(float, JetRadius, 2.0f) \
   X(float, JetSpread, 1.0f) \
   X(float, JetAngle, 0.0f) \
   X(float, JetHueSpeed, 0.7f) \
   X(float, ModJetForceRate, 0.3f) \
   X(float, ModJetForceLevel, 0.1f) \
   X(float, ModAngleRate, 0.3f) \
   X(float, ModAngleLevel, 2.0f) \
   X(float, Viscosity, 0.0005f) \
   X(float, Diffusion, 0.0005f) \
   X(float, VelocityDissipation, 0.75f) \
   X(float, DyeDissipation, 0.25f) \
   X(float, Vorticity, 7.0f) \
   X(float, GravityForce, 1.0f) \
   X(float, GravityAngle, 90.0f) \
   X(float, SolverIterations, 5.0f) \
   X(float, ModVelDissipRate, 0.5f) \
   X(float, ModVelDissipLevel, 0.0f) \
   X(float, ModDyeDissipRate, 0.5f) \
   X(float, ModDyeDissipLevel, 0.0f) \
   X(float, ColorContrast, 0.8f) \
   X(float, BlackPoint, 0.0897f) \
   X(float, FlowSat, 0.5583f) \
   X(float, FlowBright, 0.18f) \
   X(float, GlowStrength, 0.24f) \
   X(float, HighlightSat, 0.22f)
