# FluidSim — Project Context for Claude

## What this project is

**FluidSim** is an ESP32-based real-time fluid simulation visualizer for WS2812 LED matrices. It runs a Stable-Fluids (Jos Stam, 1999) Navier-Stokes solver that advects RGB dye through a velocity field, producing flame/smoke/plume visuals. Bluetooth LE provides live parameter control via a web UI (`index.html`).

This project was **forked from FlowFields** in November 2026 to support obstacle-based simulation (smoke around paddles, then bitmap obstacles, eventually obstacle-as-emitter). The architectural reasoning is documented in [docs/PLAN.md](docs/PLAN.md). Read that file first if you're new to this project — it explains why FluidSim is separate from FlowFields and what the planned roadmap looks like.

## What's currently in place (v1)

Fully working, minimal Navier-Stokes visualizer ported from `colorTrailsOrig/navier_stokes_1.py`:

- **Single emitter** — `emitter_fluidJet.h`. 3-layered Gaussian splat at bottom-center. Per-cell hue dithering to break uint8 banding.
- **Single flow** — `flow_fluid.h`. Stam Stable Fluids: diffuse + project + advect + project + (optional) vorticity confinement; dye diffuse + advect; per-frame velocity + dye dissipation.
- **Output dithering** — Bayer 4×4 ordered dither at the f2u8 LED quantization step (`f2u8d` in `fluidSimTypes.h`) breaks color banding.
- **Modulator system** — Perlin-noise driven, multiple time-based outputs per timer. Same architecture as FlowFields.
- **BLE/UI parameter system** — X-macro `PARAMETER_TABLE` in `parameterSchema.h` auto-generates JSON serialization, deserialization, and UI sync. The web UI in `index.html` uses BLE characteristics for buttons / checkboxes / numbers / strings.

## Architecture intent (per the plan)

The current code is the foundation. **Do not optimize prematurely**. The plan is to add:

1. **`obstacle.h`** — porting `navier_stokes_3.py` "smokeObstacle". Stores `obstacleMask[H][W]` + soft-mask. `applyObstacleVelocity()` zeroes/clamps velocity at solid cells; `applyObstacleField()` is a generic per-channel hard-clear (designed to also accept a future `temperature` field). `obstacle_paddles.h` is a placeholder geometry generator; bitmap support comes later.
2. **Multiple concurrent jets** — refactor `emitter_fluidJet.h` to operate on a `JetSource` struct, with `JetSource jets[MAX_JETS]` (start with `MAX_JETS=1`). The emit step iterates. Per-jet UI sliders use indexed cVar names (`cJet0Force`, `cJet1Force`, …). Obstacle-as-emitter plugs in here.
3. **Heat field for fire/smoke simulator** — add `temperature[H][W]` + `_prev`. Solver primitives are already generic; structure `fluidAdvect()` so the heat block is additive (diffuse, advect, dissipate). Buoyancy coupling adds vertical force proportional to temperature. Jets get a `jetHeat` parameter.

These are not yet implemented in code. Read [docs/PLAN.md](docs/PLAN.md) for the full reasoning.

## Source visualizers (Python)

These live in `colorTrailsOrig/`:

- `navier_stokes_1.py` — already ported as the current emitter+flow pair
- `navier_stokes_2.py` — variation, not yet ported
- `navier_stokes_3.py` — **next up**: introduces the obstacle concept

## Build & test

- **S3 build (default):** `C:/Users/Jeff/.platformio/penv/Scripts/pio.exe run`
- **P4 build:** `C:/Users/Jeff/.platformio/penv/Scripts/pio.exe run -c platformio_p4.ini`
- **Upload (P4):** add `-t upload`
- The user runs the device and observes visuals; correctness is judged primarily on the LED output, not unit tests.

## Conventions inherited from FlowFields

- **5-file cVar pattern**: `parameterSchema.h` (cVars + PARAMETER_TABLE + PROGMEM arrays), `bleControl.h` (BLE plumbing), `fluidSimEngine.hpp` (cVar↔struct bridge), `index.html` (slider registry + UI mapping). Adding a new param means edits in all four locations.
- **Float color pipeline**: `gR/gG/gB[H][W]` are float grids. The single quantization point is `f2u8d` at LED copy. Never store color as `uint8_t` or `CRGB` mid-pipeline. Ref: `docs/FASTLED_FLOAT_COLOR_REPORT.md`.
- **Modulator naming** (recently refactored): `directional_*` = bipolar [-1,+1], `normalized_*` = unipolar [0,1], `radial_*` = angle [0,2π]. `_phase` suffix = deterministic sawtooth, `_sine` = deterministic sinusoid, `_noise` = Perlin stochastic.
- **Don't hardcode ports** in `platformio.ini` — auto-detect.
- **The user runs builds**, but it's fine for Claude to run `pio run` to verify a change compiles. Don't `pio run -t upload` unless asked.

## Namespace

All FluidSim code lives in `namespace fluidSim`. Main entry: `fluidSim::initFluidSim(myXY)` then `fluidSim::runFluidSim()` per frame, called from `main.cpp`.
