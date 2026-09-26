# BIKINI

非官方 Blender **5.3** 便携包（Windows x64）。跟踪官方每日版，在节点系统上补计算几何、求解器和纹理/着色能力。与 Blender Foundation 无关，未做官方签名。**不是**官方 Blender。

- 文档：[主页](https://blueish0930.github.io/BIKINI/) · [手册](https://blueish0930.github.io/BIKINI/manual.html) · [更新日志](https://blueish0930.github.io/BIKINI/changelog.html) · 本地 `docs/index.html`
- 源码：[projects.blender.org/blueish/BIKINI](https://projects.blender.org/blueish/BIKINI)
- 站点 / 发布：[github.com/blueish0930/BIKINI](https://github.com/blueish0930/BIKINI)

节点参数、各版改动写在 `docs/`（手册与更新日志）。本文件只说明项目目标和引用的资源。

## 目标

1. 跟官方 Blender 5.3 每日版走，维护一份 Windows x64 便携构建，整包解压即可用。
2. 在几何节点、着色器、合成器和 GPU 纹理编辑器里，接入官方构建没有带上的计算几何、物理求解、重网格和图像处理能力。
3. 这些能力尽量接已有库，或按公开论文里的方法做精简实现；出处和许可证留在本文件与 `license/`。

## 在做什么

- 源码跟 Blender 5.3 主干。便携包、文档站和议题在 GitHub。
- 改动集中在节点图：几何节点、着色器、合成器、GPU 纹理编辑器，以及和它们配套的视口、渲染与编辑器行为。
- 几何与物理侧使用下面列出的计算几何、刚体、流体和重网格库。降噪可选附带 NVIDIA DLSS / NGX 运行库。
- 某个节点的输入、输出和版本差异以手册、更新日志为准，不在这里逐条列举。

## 使用

1. **整包解压**，不要只拷 `blender.exe`。
2. 运行同目录的 `blender.exe`。
3. 下列内容必须留在 exe 旁边：`5.3/`、`blender.crt/`、`blender.shared/`、`license/`、`bf_intern_*.dll`、`python3.dll` / `python313.dll`；若启用 DLSS，还需保留 `nvngx_dlssd.dll`（根目录与 `5.3/scripts/addons_core/cycles/` 各有一份）。

构建日期与提交见 `BIKINI_BUILD_INFO.txt`。

## 引用的资源

完整许可证文本在 `license/`（`license/license.md`、`license/spdx/`、`license/others/`）。下表只列 **BIKINI 额外引入或需要单独致谢** 的部分；官方 Blender 自带的依赖仍以 `license/license.md` 为准。

| 库 / 作品 | 版本 / 说明 | 许可证 | 用在 | 出处 |
|-----------|-------------|--------|------|------|
| [CGAL](https://www.cgal.org) | 6.2 | GPL-3.0-or-later / LGPL | 几何节点 CGAL | [cgal.org](https://www.cgal.org) · [GitHub](https://github.com/CGAL/cgal) |
| [Box2D](https://box2d.org) | 3 | MIT · Erin Catto | Box Engine 2D | [GitHub](https://github.com/erincatto/box2d) |
| [Box3D](https://github.com/erincatto/box3d) | — | MIT · Erin Catto | Box Engine 3D | [GitHub](https://github.com/erincatto/box3d) |
| [Jolt Physics](https://github.com/jrouwe/JoltPhysics) | 5.6.0 | MIT · Jorrit Rouwe | Jolt Solver | [GitHub](https://github.com/jrouwe/JoltPhysics) |
| [Instant Meshes](https://github.com/wjakob/instant-meshes) | — | **BSD-3-Clause** · Jakob / Panozzo / Tarini / Sorkine-Hornung | Instant Meshes 重网格 | [GitHub](https://github.com/wjakob/instant-meshes) |
| [QRemeshify / QuadWild](https://github.com/ksami/QRemeshify) | 插件整包 | **GPL-3.0** | QuadWild 节点 | [GitHub](https://github.com/ksami/QRemeshify) · 见 `quadwild/README.txt` |
| [QuadriFlow](https://github.com/hjwdzh/QuadriFlow) | 27a6867 | MIT | 四边重网格 | [GitHub](https://github.com/hjwdzh/QuadriFlow) |
| [pmp-library](https://github.com/pmp-library/pmp-library) | 核心 remesh | MIT | Triangle Remesh | [GitHub](https://github.com/pmp-library/pmp-library) |
| [Voro++](https://github.com/chr1shr/voro) | — | BSD-3-Clause · LBNL | Voronoi 破碎 | [GitHub](https://github.com/chr1shr/voro) |
| [Eigen](https://gitlab.com/libeigen/eigen) | 5.x | MPL-2.0 | 稀疏线性代数 | [GitLab](https://gitlab.com/libeigen/eigen) |
| [Spectra](https://github.com/yixuan/spectra) | — | MPL-2.0 | 特征分解 | [GitHub](https://github.com/yixuan/spectra) |
| [JET](https://github.com/doyubkim/fluid-engine-dev) | fluid-engine-dev | MIT · Doyub Kim | Flip Solver | [GitHub](https://github.com/doyubkim/fluid-engine-dev) |
| [WebGL Fluid Simulation](https://github.com/PavelDoGreat/WebGL-Fluid-Simulation) | — | MIT · Pavel Dobryakov | GTE 流体参考 | [GitHub](https://github.com/PavelDoGreat/WebGL-Fluid-Simulation) |
| **NVIDIA DLSS / NGX** | 4.5（`nvngx_dlssd.dll`） | **专有（非开源）** | Cycles / 合成器 DLSS | [NVIDIA DLSS](https://github.com/NVIDIA/DLSS) · `license/others/NVIDIA-RTX-SDK.txt` |

论文 / 方法引用（实现参考，不是整库拷贝）：

- Macklin / Müller / Bender, XPBD（PBD Solver 的方法来源；本树是独立精简实现，不是 PositionBasedDynamics 整仓）
- Jiang / Hu et al., MLS-MPM（MPM Solver 的方法来源）

源码目录（开源部分）：`extern/cgal`、`extern/box2d`、`extern/box3d`、`extern/jolt`、`extern/instant-meshes`、`extern/pmp-library`、`extern/quadriflow`、`extern/voro++`、`extern/jet`、`intern/cycles`（Apache-2.0）。DLSS 仅以二进制 DLL 形式附带，**无对应开源树**。

## 许可证

这是 **GPL 衍生作品**，不是 MIT 小工具。

- Blender 应用程序按 **GPL-3.0-or-later** 分发（官方说明；个别源文件 SPDX 仍为 GPL-2.0-or-later）。
- **Cycles** 本身是 **Apache-2.0**，留在 `intern/cycles`，没有改成 GPL。
- Apache-2.0 **可以**放进 GPL-3 发行包，**不能**放进纯 GPL-2 包。本构建因 CGAL、QRemeshify 等已按 GPL-3 分发，兼容。
- CGAL、QRemeshify/QuadWild 是 **GPL-3**，链进 exe 后整包按 GPL-3 分发。
- Instant Meshes 是 **BSD-3-Clause**（不是 GPL）。Box2D / Box3D / Jolt / PMP / QuadriFlow / JET 是 MIT，保留版权声明即可。
- 分发时请保留整个 `license/` 目录，**不要**声称这是官方 Blender。

### 特别说明：NVIDIA DLSS 4.5（专有组件）

本构建可选包含 **NVIDIA DLSS / NGX** 运行库（`nvngx_dlssd.dll`），用于降噪等功能。请注意：

1. **不是开源软件**。DLSS / NGX 受 NVIDIA RTX SDKs License 约束，版权归 NVIDIA Corporation；与 GPL 组件**并存**，但 **DLSS 二进制本身不因同包分发而变成 GPL，也不得被“传染”为开源义务**（NVIDIA 许可证明确禁止以会导致 SDK 适用开源许可证的方式使用）。
2. **只能作为本应用的一部分、以目标代码形式分发**，不得单独拆出 `nvngx_dlssd.dll` 当作独立产品再分发；不得对 DLL 做逆向工程、去版权声明等。
3. **面向 NVIDIA GPU**。DLSS / NGX 功能依赖兼容的 NVIDIA 硬件与驱动；在非 NVIDIA 设备上相关选项可能不可用或自动回退。
4. **商标与背书**。除非另有 NVIDIA 书面许可，不得暗示本构建由 NVIDIA 赞助或背书。可按 NVIDIA 许可证要求在 About / 启动相关说明中标注使用了 DLSS / NGX。
5. **完整条款**见 `license/others/NVIDIA-RTX-SDK.txt`（NVIDIA RTX SDKs License，含 DLSS / NGX；v. March 14, 2024）。官方原文：<https://github.com/NVIDIA/DLSS/blob/main/LICENSE.txt>。商业发布前请按该许可证履行 NVIDIA 通知等义务（见补充条款中的 Notification）。

若你分发本便携包：请保留上述 DLL 与 `license/others/NVIDIA-RTX-SDK.txt`；若你**删除** DLSS DLL，则本专有条款对该副本不再适用，但其余 GPL / Apache / 第三方声明仍须保留。
