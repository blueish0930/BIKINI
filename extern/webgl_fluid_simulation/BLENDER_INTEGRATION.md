# WebGL Fluid Simulation → Blender Image Process

## Upstream (source of truth)

- **Repository**: https://github.com/PavelDoGreat/WebGL-Fluid-Simulation  
- **License**: MIT (see `LICENSE`)  
- **Vendored copy**: this directory (`extern/webgl_fluid_simulation/`)  
- **Canonical logic**: `script.js` function `step(dt)`  
- **Original GLSL**: `extracted_shaders/*` (verbatim extract from `script.js`)

## Integration rule

**Do not invent a custom fluid solver.** The Blender GPU path must run **his** algorithm only:

1. Same pass order as `step(dt)`
2. Same equations as his fragment shaders (compute ports under `source/blender/compositor/shaders/compositor_fluid_pavel_*.glsl`)
3. Thin C++ driver + Blender node I/O only

**Removed / not used on the live path:**

- Geometric Multigrid pressure (residual / restrict / prolong)
- MAC staggered custom solver (`NOD_image_fluid_solver.hh` CPU MAC, `execute_gpu` MAC)
- Domain bounce BC invents mid-step

## Pass mapping (`script.js` → Blender)

| `script.js` | Blender compute shader |
|-------------|------------------------|
| `curlProgram` | `compositor_fluid_pavel_curl` |
| `vorticityProgram` | `compositor_fluid_pavel_vorticity` |
| `divergenceProgram` | `compositor_fluid_pavel_divergence` |
| `clearProgram` (pressure *= PRESSURE) | `compositor_fluid_pavel_clear` |
| `pressureProgram` × PRESSURE_ITERATIONS | `compositor_fluid_pavel_pressure` |
| `gradienSubtractProgram` | `compositor_fluid_pavel_gradient` |
| `advectionProgram` (velocity) | `compositor_fluid_pavel_advect_velocity` |
| `advectionProgram` (dye) | `compositor_fluid_pavel_advect` |
| `getResolution(n)` | `fluid_sim_size()` in `node_image_fluid_sim.cc` |

Shared UV / texelSize / bilerp helpers:  
`source/blender/compositor/shaders/library/webgl_fluid_pavel_common.glsl`

## Blender entry point

- Node: Image Process **Fluid Simulation** zone  
  `source/blender/nodes/image/nodes/node_image_fluid_sim.cc`
- GPU execute: `FluidSimOutputOperation::execute_gpu_pavel()` only  
  (`execute()` calls this; no MAC/CPU fallback)
- Defaults mirror `config` in `script.js` (SIM 128, DYE 1024, PRESSURE_ITERATIONS 20, CURL 30, …)

## Optional Blender-only extensions (no-ops when unconnected)

- Collision Mask / Collider Velocity  
- Air Mask  
- Temperature (extra scalar advect)  
- Display-domain resample of outputs  

## Regenerating extracts

If upstream updates: re-clone into this tree, re-extract fragment sources from `script.js` into `extracted_shaders/`, then re-sync the `compositor_fluid_pavel_*.glsl` compute ports to those equations.
