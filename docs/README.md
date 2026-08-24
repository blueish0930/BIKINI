# BIKINI Docs (GitHub Pages source)

This folder is the **GitHub Pages** root. Layout mirrors [docs.blender.org](https://docs.blender.org/) — hub page with **User Manual** and **Developer Docs** (no Python API).

| File | Role | URL (after Pages is on) |
|------|------|-------------------------|
| [index.html](index.html) | Documentation hub | https://blueish0930.github.io/BIKINI/ |
| [manual.html](manual.html) | User Manual (node usage — expand later) | …/manual.html |
| [changelog.html](changelog.html) | Developer Docs (release notes by module) | …/changelog.html |

## Structure

```
docs/
├── index.html        # Hub — like docs.blender.org (2 cards)
├── manual.html       # User Manual shell (Sphinx / Manual style)
├── changelog.html    # Developer Docs / Release Notes (MkDocs style)
├── README.md
└── .nojekyll
```

### index.html — Documentation home
- Dark commercial navbar, hero title **Documentation**
- Two cards only: **User Manual** · **Developer Docs**
- Bilingual EN / 中文

### manual.html — User Manual
- Placeholder ready for per-node parameter pages
- Left nav: **v1** (previous nodes) and **v2** (CGAL, Box2D/3D, Texture, Object Editor, …)
- Fill in later like the [official Manual](https://docs.blender.org/manual/)

### changelog.html — Developer Docs
- Styled after [developer.blender.org release notes](https://developer.blender.org/docs/release_notes/5.3/geometry_nodes/)
- Left: **v1** / **v2** version folds → modules; main: feature lists per module

Language preference is shared across all three pages (`localStorage`).

Documentation only — does not affect `blender.exe` at runtime.

## Local preview

Open `index.html` in a browser (no server required).
