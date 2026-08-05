# BIKINI

非官方 **Blender 5.3** 便携构建（Windows x64）。在主线之上做自己真正想用的节点、图像工具与编辑器体验。

**文档：** [主页](https://blueish0930.github.io/BIKINI/) · [更新日志](https://blueish0930.github.io/BIKINI/changelog.html) · 本地 `docs/`

---

## 中文

### 目标

- 跟进 **Blender main / 5.3**，不当死分叉。
- 以 **Geometry Nodes** 为中枢，补齐几何处理、稀疏求解、交互选择与重网格等能力。
- 新增 **GPU Texture Editor（Image Process）**：面向贴图 / 栅格 / 流体的节点图（COP 向）。
- 改进 **节点编辑器 UX** 与调试（Portal、对齐、组接口、属性预览等）。
- 便携分发：解压即用，文档随包。

### 已优化 / 主要方向

| 方向 | 内容概要 |
|------|----------|
| 几何节点 | Portal、Clip、Loop 细分、近邻、Debug、Time Shift、Select/Edit、Heat Geodesic、染色、切向场、曲线求交、Expression、Write at Index、Delaunay… |
| 稀疏数学 | Mesh Laplacian → Sparse Matrix Math → Linear Solver（稀疏求解 / 分解 / 特征） |
| 重网格与破碎 | Instant Meshes、QuadWild、Triangle Remesh、Voronoi Fracture |
| 图像节点 | Point Stamp、光栅化、法线/高度、Paint、Histogram、流体区、SDF/Fractal、Bake、Sample/Write at Pixel… |
| 着色器 | SDF Shape、Fractal、HLSL、Image Socket |
| 界面 | 抖动拆线、U 对齐、组 Separator/Message、Ctrl/Alt 改 socket、Menu 多选、Spreadsheet 排序… |
| 编辑器 | Data-Block Graph；合成器内嵌 GTE 组节点 |

### 使用

1. 下载并**整包解压**（勿只拷 `blender.exe`）。
2. 运行 `blender.exe`（同级需保留 `5.3/`、`blender.crt/`、`blender.shared/`、`license/` 等）。
3. 详见 `BIKINI_BUILD_INFO.txt`。

> 非官方构建，与 Blender Foundation 无隶属关系，未做官方签名。

---

## English

### Goals

- Track **Blender main / 5.3**; keep changes as maintainable increments, not a dead fork.
- Center on **Geometry Nodes**: geometry ops, sparse solvers, interactive select/edit, remesh.
- Add **GPU Texture Editor (Image Process)** for image/raster/fluid node graphs (COP-like).
- Improve **node-editor UX** and debugging.
- Ship a **portable** Windows build with docs.

### Focus areas

Geometry Nodes (portals, clip, remesh, fracture, sparse linear algebra, expression…), Image Process (stamp, fluid, SDF/fractal, bake, paint…), shaders (SDF/HLSL), UI gestures and group interface tools, extra editors (Data-Block Graph; compositor bridge to GTE).

### Usage

Download ZIP → extract whole tree → run `blender.exe`. Keep runtime folders next to the executable. See `BIKINI_BUILD_INFO.txt`.

> Unofficial custom build; not affiliated with or signed by the Blender Foundation.

---

## 引用资源与许可证 / Resources & licenses

以下为构建中**关键上游与自带库**的协议摘要。完整第三方列表见包内 [`license/`](license/)（Blender 官方许可证树）。

| 资源 / Resource | 用途 / Role | 协议 / License | 可否随本项目开源分发？ |
|-----------------|-------------|----------------|------------------------|
| **Blender** | 主体程序与节点框架 | **GPL-2.0-or-later** | **可以**，但必须以 **GPL 兼容** 方式发布：附带/提供对应源码，不得改成闭源专有产品冒充官方。 |
| **Eigen** | 稀疏线性代数（Linear Solver 等） | **MPL-2.0** | 可以；保留版权与 MPL 声明，修改文件需按 MPL 要求。 |
| **Spectra** | 部分特征分解 | **MPL-2.0** | 同上。 |
| **Voro++** | Voronoi 破碎单元 | **BSD-3-Clause** | 可以；保留版权与免责声明。 |
| **Instant Meshes**（上游算法/实现） | Instant Meshes 重网格 | 遵循**上游仓库许可证**（常见为 **GPL-3** 系） | 可以，但须遵守其 GPL 条款（源码可得、许可兼容）。 |
| **QRemeshify / QuadWild** | QuadWild 节点桥接 | **GPL-3**（见 `quadwild/README.txt`） | 可以，须按 GPL-3 提供源码与许可证。 |
| **Inigo Quilez SDF / fractal 公式** | SDF Shape、Fractal 参考 | 通常允许学习与使用（以作者页面声明为准） | 可引用实现；建议保留出处说明。 |
| 其它 Blender 捆绑库（Python、OpenEXR、OpenVDB、Bullet…） | 运行时 | **MIT / BSD / Apache-2.0 / LGPL / Zlib…** 见 `license/` | 可以，但须**完整保留** Blender 自带的 `license/` 与归属，不得拆掉只发 exe。 |

### 你能不能「这样开源代码发出去」？

**可以开源，但性质是「基于 Blender 的 GPL 衍生作品」，不是 MIT 单许可证小工具。**

1. **Blender 本体是 GPL**：改节点、链进 `blender.exe` 的 C/C++ 代码一般需按 **GPL（或兼容许可证）** 提供源码；不能只发二进制、拒绝对应源码。  
2. **BIKINI 自定义部分**建议明确写清：以 **GPL-2.0-or-later**（或与 Blender 相同）发布，避免和主程序冲突。  
3. **第三方库**（Eigen、Spectra、Voro++、QuadWild…）按各自协议保留声明；**不要**把别人的代码改头换面声称「纯 MIT 自研」。  
4. **允许**：GitHub 公开源码、发便携包、写文档与更新日志（本仓库做法）。  
5. **不允许**：去掉 GPL/`license/`、声称官方 Blender、用闭源商业条款覆盖整棵 Blender 树。  
6. **GPL-2-or-later + GPL-3 组件**（如部分 remesh 工具）：组合分发时通常按 **GPL-3** 兼容方式处理；保留各组件原许可证文本。

更细的 SPDX 文本在 `license/spdx/`。若只 fork 文档站点、不重新分发 `blender.exe`，仍建议标注 Blender / 上游版权；**一旦分发修改后的 Blender 二进制，就必须满足 GPL 源码义务。**

---

**BIKINI** 仅标识此非官方构建。站点：https://blueish0930.github.io/BIKINI/ · 仓库：https://github.com/blueish0930/BIKINI
