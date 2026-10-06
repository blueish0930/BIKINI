# 引用库协议说明

BIKINI 的程序整体沿用 Blender 的 GNU General Public License v3.0 or later。版权归 Blender Foundation 及其贡献者；BIKINI 在此基础上的修改同样按该许可证提供。

- 许可证全文：[release/license/spdx/GPL-3.0-or-later.txt](release/license/spdx/GPL-3.0-or-later.txt)
- 根目录的简短说明：[COPYING](COPYING)
- 项目文档：<https://blueish0930.github.io/BIKINI/>

随程序一起编译或链接的第三方库保留它们自己的许可证。分发二进制时，需要同时满足 GPL，以及下面这些库各自的条件。

上游 Blender 已经生成的库清单和许可证正文在 [release/license/license.md](release/license/license.md)。那份文件按许可证分组，每组先列出库名、版本和版权行，再附上许可证原文。SPDX 文本在 [release/license/spdx/](release/license/spdx/)。那份清单对应 Blender 原有依赖，还没有列入 BIKINI 另外引用的 CGAL、LuxCore、Spectra、OptiX 和 DLSS。这几项写在下面。

## 已经在本仓库里的第三方源码

`extern/` 里除 `luxcore` 子模块以外的目录，都随 Git 历史分发。每个目录带有自己的许可证文件。和几何节点直接相关、且不在 `license.md` 清单里的是：

| 库 | 位置 | 许可证 |
| --- | --- | --- |
| CGAL 6.2 | `extern/cgal` | 各文件为 GPL-3.0-or-later 或 LGPL-3.0-or-later，少量文件为 Boost Software License。说明见 [extern/cgal/LICENSE](extern/cgal/LICENSE) 和 [extern/cgal/README.blender](extern/cgal/README.blender) |

其余入库库（Audaspace、Bullet、Eigen、Jolt、OpenSubdiv、gtest 等）以各目录中的 `COPYING`、`LICENSE` 或 `README` 为准，并已汇总进 `release/license/license.md`。

## 需要自己拉取的库

这些库不在本仓库的普通文件里。拉取方式和编译步骤见 [README.md](README.md)。

| 库 | 放到哪里 | 许可证 | 许可证文本在哪 |
| --- | --- | --- | --- |
| Blender 平台预编译库 | `lib/windows_x64` 等 | 每个库各自的开源许可证，与 `release/license/license.md` 中的条目对应 | 预编译库仓库里的 `deps.md`，以及 `license.md` |
| LuxCore / SLG / LuxRays | `extern/luxcore` | Apache-2.0 | 检出后的 `extern/luxcore/COPYING.txt` |
| Spectra | `lib/<platform>/spectra` | Mozilla Public License 2.0 | 检出后的 `LICENSE`；SPDX 文本见 [release/license/spdx/MPL-2.0.txt](release/license/spdx/MPL-2.0.txt) |
| NVIDIA OptiX 头文件 | `lib/<platform>/optix` | NVIDIA Software Developer Kits, Samples and Tools License | 用户下载的 SDK 内 `LICENSE.txt` |
| NVIDIA DLSS / NGX SDK | `lib/<platform>/dlss` | NVIDIA RTX SDKs License | 用户下载的 SDK 内 `LICENSE.txt` |

`patches/luxcore-bikini.patch` 是打在 LuxCore 上的本地修改，不改变 LuxCore 的 Apache-2.0。

OptiX 和 DLSS 是 NVIDIA 的专有 SDK，本仓库不收录这两份代码，也不转载其许可证全文。用户从 NVIDIA 或对应公开仓库取得 SDK 时，以随包的 `LICENSE.txt` 为准。
