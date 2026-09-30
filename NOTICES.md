# 第三方组件与许可

本文件列出 BIKINI 额外引入或需要单独致谢的组件。官方 Blender 自带依赖仍以便携包中的 `license/license.md` 为准。完整许可证文本见源码树的 [COPYING](COPYING)、各 `extern/` 目录，以及便携包中的 `license/`（`license/license.md`、`license/spdx/`、`license/others/`）。

## 组件

| 库 / 作品 | 版本 / 说明 | 许可证 | 用在 | 出处 |
|-----------|-------------|--------|------|------|
| [CGAL](https://www.cgal.org) | 6.2 | GPL-3.0-or-later / LGPL | 几何节点 CGAL | [cgal.org](https://www.cgal.org) · [GitHub](https://github.com/CGAL/cgal) |
| [Box2D](https://box2d.org) | 3 | MIT · Erin Catto | Box Engine 2D | [GitHub](https://github.com/erincatto/box2d) |
| [Box3D](https://github.com/erincatto/box3d) | — | MIT · Erin Catto | Box Engine 3D | [GitHub](https://github.com/erincatto/box3d) |
| [Jolt Physics](https://github.com/jrouwe/JoltPhysics) | 5.6.0 | MIT · Jorrit Rouwe | Jolt Solver | [GitHub](https://github.com/jrouwe/JoltPhysics) |
| [Instant Meshes](https://github.com/wjakob/instant-meshes) | — | BSD-3-Clause · Jakob / Panozzo / Tarini / Sorkine-Hornung | Instant Meshes 重网格 | [GitHub](https://github.com/wjakob/instant-meshes) |
| [QRemeshify / QuadWild](https://github.com/ksami/QRemeshify) | 插件整包 | GPL-3.0 | QuadWild 节点 | [GitHub](https://github.com/ksami/QRemeshify) · `quadwild/README.txt` |
| [QuadriFlow](https://github.com/hjwdzh/QuadriFlow) | 27a6867 | MIT | 四边重网格 | [GitHub](https://github.com/hjwdzh/QuadriFlow) |
| [pmp-library](https://github.com/pmp-library/pmp-library) | 核心 remesh | MIT | Triangle Remesh | [GitHub](https://github.com/pmp-library/pmp-library) |
| [Voro++](https://github.com/chr1shr/voro) | — | BSD-3-Clause · LBNL | Voronoi 破碎 | [GitHub](https://github.com/chr1shr/voro) |
| [Eigen](https://gitlab.com/libeigen/eigen) | 5.x | MPL-2.0 | 稀疏线性代数 | [GitLab](https://gitlab.com/libeigen/eigen) |
| [Spectra](https://github.com/yixuan/spectra) | — | MPL-2.0 | 特征分解 | [GitHub](https://github.com/yixuan/spectra) |
| [JET](https://github.com/doyubkim/fluid-engine-dev) | fluid-engine-dev | MIT · Doyub Kim | Flip Solver | [GitHub](https://github.com/doyubkim/fluid-engine-dev) |
| [WebGL Fluid Simulation](https://github.com/PavelDoGreat/WebGL-Fluid-Simulation) | — | MIT · Pavel Dobryakov | GTE 流体参考 | [GitHub](https://github.com/PavelDoGreat/WebGL-Fluid-Simulation) |
| NVIDIA DLSS / NGX | 4.5（`nvngx_dlssd.dll`） | 专有 | Cycles / 合成器 DLSS | [NVIDIA DLSS](https://github.com/NVIDIA/DLSS) · `license/others/NVIDIA-RTX-SDK.txt` |

方法参考（实现依据，不是整库拷贝）：

- Macklin / Müller / Bender, XPBD。PBD Solver 的方法来源。本树为独立实现，不是 PositionBasedDynamics 仓库。
- Jiang / Hu et al., MLS-MPM。MPM Solver 的方法来源。

源码目录：`extern/cgal`、`extern/box2d`、`extern/box3d`、`extern/jolt`、`extern/instant-meshes`、`extern/pmp-library`、`extern/quadriflow`、`extern/voro++`、`extern/jet`、`intern/cycles`（Apache-2.0）。DLSS 仅以二进制 DLL 随便携包分发，仓库中没有对应源码树。

## 分发

- Blender 应用程序按 GPL-3.0-or-later 分发。个别源文件的 SPDX 标识仍为 GPL-2.0-or-later。
- Cycles 为 Apache-2.0，位于 `intern/cycles`。
- Apache-2.0 可以进入 GPL-3 发行包。本构建因 CGAL、QRemeshify 等组件按 GPL-3 分发。
- CGAL 与 QRemeshify / QuadWild 为 GPL-3。链入可执行文件后，整包按 GPL-3 分发。
- Instant Meshes 为 BSD-3-Clause。Box2D、Box3D、Jolt、PMP、QuadriFlow、JET 为 MIT，分发时保留版权声明。
- 分发便携包时请保留整个 `license/` 目录，并说明这不是官方 Blender。

## NVIDIA DLSS 4.5

便携包可选包含 NVIDIA DLSS / NGX 运行库（`nvngx_dlssd.dll`），用于降噪。

1. DLSS / NGX 不是开源软件，受 NVIDIA RTX SDKs License 约束，版权归 NVIDIA Corporation。它可以与 GPL 组件放在同一发行包中，但 DLL 本身不因此变为 GPL。NVIDIA 许可证禁止以会导致 SDK 适用开源许可证的方式使用该 SDK。
2. 只能作为本应用的一部分、以目标代码形式分发。不得单独将 `nvngx_dlssd.dll` 作为独立产品再分发，不得对 DLL 做逆向工程或去除版权声明。
3. 功能面向 NVIDIA GPU，依赖兼容的硬件与驱动。在其他设备上，相关选项可能不可用或自动回退。
4. 未经 NVIDIA 书面许可，不得暗示本构建由 NVIDIA 赞助或背书。可按许可证要求在 About 或启动说明中标注使用了 DLSS / NGX。
5. 完整条款见便携包 `license/others/NVIDIA-RTX-SDK.txt`（NVIDIA RTX SDKs License，含 DLSS / NGX，v. March 14, 2024）。原文：<https://github.com/NVIDIA/DLSS/blob/main/LICENSE.txt>。商业发布前请履行其中的通知义务。

分发便携包时请保留该 DLL 与 `license/others/NVIDIA-RTX-SDK.txt`。若某个副本删除了 DLSS DLL，则上述专有条款对该副本不再适用；GPL、Apache 与其他第三方声明仍然有效。
