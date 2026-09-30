# BIKINI

Blender 5.3 的生产向分支。在几何节点、着色器、合成器与 GPU 纹理编辑器中，提供计算几何、物理求解、重网格和图像处理。

Windows x64，与官方每日版同步。独立项目，与 Blender Foundation 无关，未经官方签名。

## 概览

BIKINI 跟随 Blender 5.3 主干，把官方构建未带上的几何与图像能力接到现有节点图中。节点参数、版本差异与操作说明以文档站为准。

## 能力

- 几何节点中的计算几何、刚体与流体求解、重网格
- 着色器、合成器与 GPU 纹理编辑器中的配套节点
- 视口、渲染与编辑器中与上述节点配套的行为
- 可选的 NVIDIA DLSS / NGX 降噪（便携包中的专有运行库）

## 文档

- [文档站](https://blueish0930.github.io/BIKINI/)
- [手册](https://blueish0930.github.io/BIKINI/manual.html)
- [更新日志](https://blueish0930.github.io/BIKINI/changelog.html)
- 本仓库 `docs/`

## 获取与构建

源码即本仓库。预编译依赖通过 `.gitmodules` 中的 `lib/windows_x64` 等子模块获取，不包含在仓库文件内。Windows 构建步骤见 [Blender 官方手册](https://developer.blender.org/docs/handbook/building_blender/)。

本地便携包不进入本仓库。运行时请将 `5.3/`、`blender.crt/`、`blender.shared/`、`license/` 以及相关 DLL 留在 `blender.exe` 同目录。启用 DLSS 时同时保留 `nvngx_dlssd.dll`。

## 许可

BIKINI 按 GPL 衍生作品分发。第三方组件、许可证与 NVIDIA DLSS 条款见 [NOTICES.md](NOTICES.md)。Blender 本体许可见 [COPYING](COPYING)。
