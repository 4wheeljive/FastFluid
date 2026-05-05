# Plan: Port navier_stokes_3.py — recommend fresh "FluidSim" project

## Context

The user wants to port `colorTrailsOrig/navier_stokes_3.py` (the "smokeObstacle" sketch). It builds on `navier_stokes_1.py` (which we've already ported as `emitter_fluidJet` + `flow_fluid` in FlowFields) but introduces a fundamentally new concept: **obstacles**. The user has signaled openness to either integrating into FlowFields or starting fresh, and explicitly flagged that this is just the start — future obstacles will include bitmap images and will eventually act as emitters themselves.

The architectural question is whether the obstacle concept fits FlowFields' existing emitter+flow model or warrants a fresh project. After analyzing the Python source, **my recommendation is to start fresh as a new "FluidSim" project.**

## What's actually new in `navier_stokes_3.py`

The Explore agent's analysis confirms:

- **Solver core** (diffuse, project, advect, vorticity): unchanged from `navier_stokes_1.py`. Already ported.
- **Emitter** (3-layered Gaussian jet at bottom-center): unchanged. Already ported.
- **What's new**: an obstacle layer that
  - Stores two grids: `obstacle_mask` (bool) + `obstacle_soft_mask` (float, gap edge softening)
  - Wraps the solver — applies obstacle boundary conditions **3 times** during the velocity step (after diffuse, after advect, after vorticity) and **once** for dye (hard erase in solid cells)
  - Animates dynamically (paddle position swings via sinusoid)
  - Renders as an overlay on the dye field
- **6 new parameters**: obstacle width, speed, thickness, soft-edge factor, color, outline.

So the algorithmic delta is small. The architectural delta is significant.

## Why fresh project, not integration

1. **Obstacles break the emitter+flow abstraction.** FlowFields' core invariant is "any emitter pairs with any flow." An obstacle is a **third entity** — it isn't an emitter (it doesn't inject dye) and isn't a flow (it doesn't transport dye); it's a constraint on the simulation. Adding it as global namespace state pollutes the architecture for the 6 emitters and 5 flows that don't use it.

2. **The obstacle only makes sense paired with `flow_fluid`.** Pairing the existing `noiseFlow`/`radial`/`spiral`/etc. with an obstacle does nothing — those flows don't have a velocity field to constrain. Either we'd need a "fluid-only" UI gate (which doesn't exist), or we'd ship dead UI weight for the other 4 flows.

3. **Obstacle-as-emitter (planned) merges roles that the dispatch table separates.** FlowFields uses `EMITTER_RUN[activeEmitter]()` and `FLOW_PREPARE[activeFlow]()` as parallel function-pointer dispatches. An obstacle that *also emits* doesn't fit either slot — it's neither a pure emitter (it has geometry) nor a pure flow (it injects color). Retrofitting this into the dispatch table requires either a third dispatch table (`OBSTACLE_RUN[]`), conditional logic in the existing tables, or breaking the parallelism. All three options compromise the architecture.

4. **Bitmap obstacles will need new infrastructure** that FlowFields doesn't currently have: image loading/decoding, bitmap rasterization, BLE transfer protocols for image data, possibly file-system support. None of this is needed for FlowFields' current emitters/flows. Adding it as optional code that only the fluid path uses is awkward.

5. **A fresh project is cheap.** The expensive parts — Stable Fluids solver, jet emitter, BLE/UI plumbing pattern, modulator framework, color pipeline — are already written and can be **copied directly** from FlowFields. We're not starting from zero; we're starting from "FlowFields minus the things FluidSim doesn't need."

The case **against** a fresh project is real but smaller: code duplication (the solver and jet emitter would now exist in two places), and cognitive overhead of two projects. Worth it for the architectural clarity.

## Recommended approach

### Project structure

Fresh PlatformIO project at `Projects/FluidSim/` with:

```
src/
  main.cpp                   ← LED setup, main loop
  fluidSimTypes.h            ← grid state, math helpers, color helpers (copy from flowFieldsTypes.h)
  parameterSchema.h          ← cVars + PARAMETER_TABLE (subset of FlowFields)
  bleControl.h               ← BLE stack (copy from FlowFields)
  fluidSimEngine.hpp         ← main loop dispatch + cVar bridge (slim)
  modulators.h               ← copy from FlowFields
  emitter_fluidJet.h         ← copy + adapt
  flow_fluid.h               ← copy + adapt + obstacle hooks
  obstacle.h                 ← NEW: obstacle representation + boundary enforcement
  obstacle_paddles.h         ← NEW: paddles geometry generator (placeholder for future bitmap)
index.html                   ← copy + slim down to fluid-only sliders + add obstacle sliders
```

### What to copy verbatim from FlowFields

- `flowFieldsTypes.h` → `fluidSimTypes.h` (rename namespace, keep math/color helpers, `f2u8d` dither, `bayerOutputDither`, `rainbow`, `hsvSpectrum`, `hsvRainbow`, `sin_fast`, `fastpow`, etc.)
- `modulators.h` (unchanged)
- `bleControl.h` (X-macro pattern, BLE characteristic plumbing — unchanged structure)
- `flow_fluid.h` Stable Fluids solver functions (`linSolve`, `diffuse`, `advectField`, `project`, `applyVorticityConfinement`)
- `emitter_fluidJet.h` (the 3-layered Gaussian splat with hue dither)

### What's new for FluidSim

- **`obstacle.h`** — obstacle state + boundary enforcement
  - `obstacleMask[HEIGHT][WIDTH]` (bool or uint8)
  - `obstacleSoftMask[HEIGHT][WIDTH]` (float, optional)
  - `applyObstacleVelocity(u, v)` — zeroes velocity in solid cells, applies directional clamping at edges (mirror of `apply_obstacle_boundary`)
  - `applyObstacleField(grid)` — generic scalar-field helper that hard-clears any transported scalar (dye R/G/B, future heat) in solid cells. Replaces the per-channel approach so heat slots in trivially.
  - `setObstacleFromGenerator(gen)` — entry point for paddle/bitmap geometry sources
- **`obstacle_paddles.h`** — paddle geometry generator
  - `paddlesParams` struct: width, speed, thickness, gap fraction
  - `generatePaddleObstacle(t)` — produces mask given time (allows dynamic animation)
- **`emitter_fluidJet.h`** — refactor the existing single-jet code to operate on a `JetSource` struct rather than a single `fluidJet` global. v1 ships with `JetSource jets[1]` and the existing UI; v2 expands `MAX_JETS`.
- **`flow_fluid.h` modifications** — invoke `applyObstacleVelocity()` after each project step and `applyObstacleField()` for each dye channel after advection. Structure dye/heat/etc. as parallel "transported field" blocks so adding heat in v2 is additive. Other solver steps unchanged.
- **`fluidSimEngine.hpp`** — slim version of `flowFieldsEngine.hpp`. No emitter/flow dispatch tables (only one of each). Single `runFluidSim()` function that calls fluidPrepare → (loop over jets calling emit) → fluidAdvect → applyObstacle → LED copy.

### Architecture allowances for the future

The architecture should be designed with these planned extensions in mind, even if not implemented in v1:

1. **Bitmap obstacle source**: `setObstacleFromGenerator(gen)` takes a generator function. `obstacle_paddles.h` is one such generator; later `obstacle_bitmap.h` can be another, sharing the same mask interface.
2. **Obstacle-as-emitter**: When the obstacle generator includes color/velocity injection points, the engine's main loop can call `obstacle.emit()` after `emitter.run()`. Keeping obstacles as a discrete module lets this be added without breaking the simulation loop.

3. **Multiple concurrent injection sources** (planned). FluidSim's emitter is **not** an interchangeable dispatch slot like FlowFields' — it is a **collection of jet sources** active simultaneously. Design implications:
   - `JetSource` struct holds per-jet params (position, angle, force, density, hue offset, modulators).
   - `JetSource jets[MAX_JETS]` (or `fl::vector<JetSource>` for dynamic count) — start with a fixed `MAX_JETS` of, say, 4–8 to keep memory predictable.
   - The emit step iterates: `for (auto& jet : jets) if (jet.enabled) jet.emit();`
   - Per-jet UI: each jet exposes its own slider group; UI groups them into "Jet 1", "Jet 2", etc. cVar names should include the jet index (`cJet0Force`, `cJet1Force`, …) for the X-macro pattern to work.
   - Obstacle-as-emitter (per #2 above) plugs in as another `JetSource` whose position is driven by the obstacle geometry rather than fixed coordinates. Same emission interface.

   **v1 keeps `MAX_JETS = 1`** with the current bottom-center jet. The struct + iteration are in place; we just don't expose more sources yet. This makes adding additional jets in v2 a parameter-list extension, not a refactor.

4. **Heat field for fire/smoke simulator** (planned). The cleanest integration point is to model heat as **another transported scalar field**, structurally identical to a dye channel:

   - Add `temperature[HEIGHT][WIDTH]` + `temperature_prev[HEIGHT][WIDTH]` (or reuse `tR/tG/tB`-style scratch convention). Cost: ~6–12 KB extra static state.
   - The solver primitives (`diffuse`, `advectField`, `linSolve`) are already generic in our existing `flow_fluid.h` — they take `float (*)[WIDTH]` pointers, so they work on **any** scalar field.
   - In `fluidAdvect()`, the heat step slots in alongside the dye step:
     ```
     diffuse(0, temperature, temperature_prev, heatDiffusion, dt)
     advectField(0, temperature, temperature_prev, u, v, dt)
     applyHeatDissipation()
     ```
   - **Buoyancy coupling**: heat couples to velocity via a vertical force proportional to temperature. Add one loop after the dye step (or before the velocity step, design choice):
     ```
     for each cell: v[y][x] -= buoyancyCoeff * temperature[y][x] * dt
     ```
   - **Obstacle interaction**: heat should be hard-cleared in obstacle cells, mirroring dye treatment. The obstacle module's `applyObstacleField(field)` generic helper handles this naturally.
   - **Emitter coupling**: jets can inject heat alongside dye and velocity. Add `jetHeat` parameter to `JetSource` and a heat write inside the splat loop.
   - **Rendering coupling** (optional v2+): hot cells can be tinted toward yellow/white regardless of dye color, producing flame appearance independently of the dye field.

   To enable this cleanly later, **v1 should**:
   - Keep solver primitives generic (they already are)
   - Structure `fluidAdvect()` so each field's diffuse/advect/dissipate trio is its own block (already true for dye), so a heat block is additive
   - Design `obstacle.h` with a generic `applyObstacleField(grid)` helper, not just per-channel `applyObstacleDye()`

### What NOT to bring over from FlowFields

- All non-fluid emitters (orbitalDots, swarmingDots, lissajous, borderRect, noiseKaleido, cube)
- All non-fluid flows (noise, radial, directional, rings, spiral)
- Audio code (unless we want fluid-driven-by-audio later — defer)
- The dispatch table machinery (only one emitter, one flow — direct calls)
- `componentEnums.h` (becomes trivial or unneeded)

### Trade-offs

- **Code duplication**: The Stable Fluids solver, jet emitter, modulator framework, color helpers, BLE stack, and X-macro pattern will exist in both projects. If you fix a bug in `linSolve()`, you fix it in two places. For the next 6 months while both projects are actively developed, this is real overhead. Mitigation: keep the shared files as close to verbatim copies as possible so they can be diffed.
- **Two projects to maintain**: Slightly more cognitive overhead and switching cost. But the FluidSim project's footprint is smaller than FlowFields and won't have audio/multiple-emitter complexity.

## Files to read before implementation begins

- [colorTrailsOrig/navier_stokes_3.py](colorTrailsOrig/navier_stokes_3.py) — full source for the port
- [src/flows/flow_fluid.h](src/flows/flow_fluid.h) — the existing solver to copy
- [src/emitters/emitter_fluidJet.h](src/emitters/emitter_fluidJet.h) — the existing jet to copy
- [src/parameterSchema.h](src/parameterSchema.h) — pattern reference for the new project's slimmed-down version
- [src/bleControl.h](src/bleControl.h) — BLE infrastructure to copy

## Verification (when ready to implement)

1. Build the new project standalone — `pio run` in `Projects/FluidSim`
2. With obstacle disabled (e.g., empty mask), behavior should match the existing fluid pair in FlowFields
3. With paddles enabled, the plume should deflect around solid cells; dye should not penetrate them
4. Toggle obstacle width, speed → live response via BLE
5. Run for 5+ minutes — confirm no NaN explosions or velocity buildup at obstacle edges (a known sensitivity point in obstacle-aware Navier-Stokes)
