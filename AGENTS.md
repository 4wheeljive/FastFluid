# FastFluid Project Briefing

This file is the handoff note for coding agents working in this repo. It is meant to be current, practical, and shorter than the accumulated chat history. The older `CLAUDE.md` has useful ancestry notes, but it is stale in several names and architecture details.

## Project Shape

FastFluid is a realtime fluid-simulation LED engine for ESP32-class microcontrollers. It runs a Stable Fluids style velocity/dye simulation on a small 2D grid, then renders the result through FastLED to WS2812-style matrices.

The code is currently Arduino/PlatformIO C++ with most simulation code in headers under `src/`. Runtime control comes from BLE plus the web UI in `index.html`.

Primary targets:

- ESP32-S3, configured by `platformio.ini`
- ESP32-P4, configured by `platformio_p4.ini`

Build commands:

```sh
C:/Users/Jeff/.platformio/penv/Scripts/pio.exe run
C:/Users/Jeff/.platformio/penv/Scripts/pio.exe run -c platformio_p4.ini
```

Do not upload unless explicitly asked:

```sh
C:/Users/Jeff/.platformio/penv/Scripts/pio.exe run -c platformio_p4.ini -t upload
```

The user often judges correctness from LED output and FPS/device behavior, not from unit tests.

## Main Entry Points

- `src/main.cpp`: Arduino setup/loop, FastLED setup, BLE setup, calls `fastFluid::initfastFluid(myXY)` once and `fastFluid::runfastFluid()` every frame.
- `src/boardConfig.h`: active board/matrix selection, pins, dimensions, LED driver, matrix lookup tables, `myXY()`.
- `src/fastFluidEngine.hpp`: central frame pipeline, dispatch tables, cVar-to-struct sync, render pipeline.
- `src/fastFluidTypes.h`: shared namespace state, grid buffers, math helpers, color helpers, drawing primitives, component dispatch types.

All simulation code lives in `namespace fastFluid`.

## Frame Pipeline

`runfastFluid()` is the best mental model for runtime behavior:

1. Calculate `dt` from elapsed time and `globalSpeed`.
2. Detect emitter/flow/obstacle selection changes and push that component's defaults into cVars.
3. Sync global, emitter, flow, and obstacle structs from cVars.
4. Assign active modulator timer slots.
5. Component `prepareModulators()` functions write timer ratios.
6. One central `calculate_modulators()` fills `move.*` signals.
7. Update obstacle mask and enforce obstacle velocity.
8. Prepare flow values.
9. Run active emitter.
10. Advect flow.
11. Render either color output or a debug view.
12. Apply obstacle overlay and optional emitter overlay.
13. `FastLED.show()` happens back in `main.cpp`.

The intended pattern is: components write modulator ratios first, then all components read `move.*` after the central modulator pass. Avoid direct `noiseX.noise()` calls inside emitters unless there is a deliberate performance/behavior reason.

## Simulation State

Important grids:

- `gR/gG/gB[HEIGHT][WIDTH]`: live float dye/color buffers.
- `tR/tG/tB[HEIGHT][WIDTH]`: dye/render scratch buffers.
- `u/v[HEIGHT][WIDTH]`: persistent velocity field in `flow_smoke.h`.
- `uPrev/vPrev`, `pressure`, `divergence`: solver scratch.

Keep the color pipeline float until the final LED copy. Quantization should happen at `f2u8d()` with Bayer dithering. Avoid introducing mid-pipeline `uint8_t` or `CRGB` color storage unless it is only for final LED output.

## Components

### Emitters

Current enum values are in `src/componentEnums.h`:

- `EMITTER_SINGLEJET`
- `EMITTER_MULTIJET`

Shared emitter primitive:

- `src/emitters.h`: `jetSplat()`, a Gaussian splat that injects dye into `gR/gG/gB` and momentum into `u/v`.

Emitter files:

- `src/emitters/emitter_singleJet.h`: bottom/near-bottom single plume.
- `src/emitters/emitter_multiJet.h`: generalized multi-jet ring emitter.
- `src/jets.h`: shared multi-jet data model.

MultiJet design intent:

- `JetPackParams` is the shared family controller: count, direction/color modes, ring radius, splat size, force, density, hue, variances, and `ModConfig`s.
- `JetParams jet[MAX_NUM_JETS]` is static per-slot personality/state.
- `MAX_NUM_JETS` is currently 5.
- `jetPack.numJets`/`multiJetCount()` should control how many slots are visited. Do not permanently mutate `jet[i].enabled` merely because the UI count was lowered.
- Per-frame resolved values should be temporary locals. Avoid mutating persistent `JetParams` every frame to represent modulation results.
- Keep expensive modulation cheap: prefer one shared modulator signal per property plus per-jet arithmetic/scales/cross-coupling, not one Perlin noise call per property per jet.
- Ring placement currently resolves from `radialAngleBase`, evenly spaced jet index, and radius modulation. There is no active `layoutMode` model unless you add it consistently.
- `direction` means absolute angle in absolute mode, and shared rotation offset in radial/tangent modes.

### Flow

Current flow:

- `src/flows/flow_smoke.h`

It is a Stable Fluids inspired smoke solver:

- velocity diffuse -> project -> self-advect -> project
- optional vorticity confinement -> project
- RGB dye diffuse/advect
- velocity/dye dissipation
- obstacle hooks before/after key velocity and dye phases

Angle convention used by jets/gravity:

- `0` = up
- `90 deg` = right
- `180 deg` = down
- `270 deg` = left

### Obstacles

Current obstacle enum:

- `OBSTACLE_PADDLES`

Files:

- `src/obstacles.h`: generic obstacle mask, soft mask, segment storage, solver hooks, overlay.
- `src/obstacles/obstacle_paddles.h`: current paddle-pair generator with a centered gap and noise-driven slide.

`obstacleCommon.enable` controls solver enforcement. `obstacleCommon.overlay` controls whether the obstacle is drawn over the rendered LEDs. Keep those concepts independent.

### Debug Views

`src/views.h` owns debug rendering and overlay helpers. Debug views include color, velocity, vorticity, pressure, divergence, dye density, and emitter overlay.

The emitter overlay is intentionally LED-only: it draws resolved multiJet anchor markers/arrows after rendering and must not contaminate `gR/gG/gB` or `u/v`.

## Parameter And UI System

The control path is split across several files. When adding or renaming a parameter, keep all layers consistent:

- `src/parameterSchema.h`
  - cVar declaration, such as `cForce`
  - component parameter list, such as `MULTIJET_PARAMS`
  - `PARAMETER_TABLE` X-macro entry
- `src/fastFluidEngine.hpp`
  - push defaults from component structs to cVars
  - sync cVars back into component structs each frame
- `index.html`
  - slider/dropdown metadata
  - component parameter panel lists
- `src/bleControl.h`
  - usually uses schema/X-macro helpers, but check special button/checkbox handling

The UI uses lower-camel parameter ids like `force`, while C++ cVars are usually `cForce` and X-macro names are `Force`. Do not mix old singleJet-specific names and generic multiJet names accidentally. [USER NOTE: I'm going to ask for your help related to this.]

Defaults should generally be pushed on component selection/startup, not every frame. Per-frame pushes from defaults to cVars can overwrite user changes before `sync*FromCVars()` sees them.

## Modulator System

Files:

- `src/modulators.h`
- `src/noise.h`
- `ModConfig` in `src/fastFluidTypes.h`

Each active component exposes a `static constexpr ModConfig ...::* MODS[]` array. The engine assigns timer slots with `assignModSlots()`. Component `prepareModulators()` functions write `timings.ratio[modTimer]`; then `calculate_modulators()` fills:

- `move.linear`
- `move.radial_phase`
- `move.normalized_phase`
- `move.directional_sine`
- `move.normalized_sine`
- `move.directional_noise`
- `move.normalized_noise`
- `move.radial_noise`

Naming convention:

- `directional_*`: bipolar `[-1, +1]`
- `normalized_*`: unipolar `[0, 1]`
- `radial_*`: angle-ish signal

## Hardware Notes

`src/boardConfig.h` selects exactly one matrix/board configuration by macro. Current checked-in selection is `S3_22x22`.

P4 parallel output is through PARLIO with ESP-Hosted BLE via the C6. `src/hosted_ble_bridge.cpp/.h` prepares the hosted BLE controller when available; otherwise `hostedBlePrepare()` is a no-op for normal NimBLE builds.

When measuring P4 LED output, distinguish:

- WS2812 wire time
- CPU packing/color conversion
- DMA/PARLIO wait/synchronization

For 12 pins x 256 LEDs, expected wire time per lane is about `7.7-8 ms`.

## Current Cautions

- The worktree may be dirty and may include user edits. Do not revert unrelated files.
- Several comments still mention old names such as `FluidSim`, `flow_fluid.h`, `emitter_fluidJet.h`, or `threeJet`; treat those as ancestry unless current code confirms them.
- If touching multiJet/singleJet headers, check for stale type names and duplicate concepts. The project has recently been through a multiJet refactor, so name drift is the main risk.
- Before large behavioral changes, run `rg` across `src/` and `index.html` for the parameter/function/type names you are changing.
- Prefer scoped fixes over sweeping cleanup; performance and visual feel matter on device.

