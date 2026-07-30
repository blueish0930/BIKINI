# BIKINI Docs

| File | Description |
|------|-------------|
| [changelog.html](changelog.html) | Release notes in Blender Developer Docs style: **left nav = each period → modules** (Geometry Nodes, Math & Solver, Image Process/COP, Shaders, EEVEE, UI, Zones). **EN / 中文**. |

## How to read

1. Open `changelog.html` in a browser.
2. **Left sidebar**
   - **BIKINI 5.3** — cumulative feature notes by module
   - **Package history** — dated deltas (`2026-07-30`, `2026-07-29`, earlier baseline)
3. Language is auto-detected (`zh*` → 中文), can be switched in the header, and is remembered in `localStorage`.

Documentation only — does not affect `blender.exe`.

## Structure (mirrors upstream)

Same idea as [developer.blender.org release notes](https://developer.blender.org/docs/release_notes/5.3/):

```
Release (period)
  └── Module (Geometry Nodes / Shaders / EEVEE / UI / …)
        └── Feature + short technical intro
```

Notable deep dives in the page:

- **Mesh Laplacian** — sparse Laplacian as COO (`weight` / `row` / `col` bundle)
- **Linear Solver** — Eigen + Spectra (solve / factor / eigen / mat-vec)
- **Image Process (COP)** — dedicated `ImageNodeTree` for image graphs
