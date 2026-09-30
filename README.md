# BIKINI

面向几何与图像制作的 Blender 5.3 分支。

跟随官方每日版，在几何节点、着色器、合成器与 GPU 纹理编辑器中提供计算几何、物理求解、重网格和图像处理。发行平台为 Windows x64。

本分支独立维护，不属于 Blender Foundation 的官方发行，构建未经官方签名。

## 简介

BIKINI 与 Blender 5.3 主干同步更新。扩展接入现有节点图。节点参数、版本差异与操作步骤以文档站为准。

## 功能

- 几何节点：计算几何、刚体与流体求解、重网格
- 着色器、合成器与 GPU 纹理编辑器：图像与着色处理
- 便携包可选附带 NVIDIA DLSS / NGX 降噪（专有运行库）

## 文档

- [产品文档](https://blueish0930.github.io/BIKINI/)
- [用户手册](https://blueish0930.github.io/BIKINI/manual.html)
- [更新日志](https://blueish0930.github.io/BIKINI/changelog.html)

源文件位于本仓库 `docs/`。

## 源码与构建

源码即本仓库。Windows 预编译库通过 `.gitmodules` 中的 `lib/windows_x64` 等子模块获取，不包含在仓库文件中。构建步骤见 [Blender 官方手册](https://developer.blender.org/docs/handbook/building_blender/)。

便携包不随本仓库发布。运行时将 `5.3/`、`blender.crt/`、`blender.shared/`、`license/` 及相关 DLL 与 `blender.exe` 置于同一目录。使用 DLSS 时保留 `nvngx_dlssd.dll`。

## 许可

BIKINI 以 GPL 衍生作品的形式分发。第三方组件、许可证与 NVIDIA DLSS 条款见 [NOTICES.md](NOTICES.md)。Blender 本体许可见 [COPYING](COPYING)。
