# BIKINI

BIKINI 是与 Blender 每日版同步的源码分支，重点在几何节点和操作体验。功能说明、节点手册和 VEX Wrangle 在文档站；这个仓库只放可以编译的源码。

- 文档站：<https://blueish0930.github.io/BIKINI/>
- 源码仓库：<https://github.com/blueish0930/BIKINI>
- 引用库协议说明：[COPYING.md](COPYING.md)

上游 Blender 的构建手册：<https://developer.blender.org/docs/handbook/building_blender/>。下面写的是这个仓库和上游的差别，以及在 Windows 上复现本机已验证构建的步骤。

## 源码里有什么

克隆仓库之后，这些目录已经在 Git 历史里：

| 目录 | 内容 |
| --- | --- |
| `source/` | Blender 程序本体：窗口、编辑器、几何节点、合成器、序列编辑器、Python API |
| `intern/` | 跟 Blender 一起编译的引擎和桥接，包括 Cycles、OpenVDB、Mantaflow，以及 BIKINI 的 LuxCore 接入（`intern/luxcore`）和 CGAL 桥（`intern/cgal_bridge`、`intern/cgal_volume_bridge`） |
| `extern/` | 已经随源码入库的第三方源码，例如 Audaspace、Bullet、CGAL、Eigen、Jolt、OpenSubdiv、gtest。各库自己的许可证在对应目录里 |
| `scripts/` | 启动脚本、内置插件、预设、模板 |
| `release/` | 运行时数据。第三方许可证全文在 `release/license/` |
| `locale/` | 界面翻译 |
| `assets/` | 画笔和节点资源 |
| `build_files/` | CMake 配置和 Windows 构建脚本（`make.bat`） |
| `tests/`、`tools/` | 测试和构建辅助工具 |
| `patches/luxcore-bikini.patch` | 打在 LuxCore `for_v2.11` 上的本地修改（Rough Glass 的 Cauchy B） |

`extern/luxcore` 和 `lib/` 下的平台库是 Git 子模块。普通 `git clone` 只会记下它们的提交号，目录是空的。

## 需要自己拉取的第三方库

子模块在 `.gitmodules` 里把平台库设成了 `update = none`。`git clone --recurse-submodules` 也不会检出它们，需要按下面的命令单独初始化。

### 编译必需

| 放到哪里 | 是什么 | 从哪里拉 |
| --- | --- | --- |
| `lib/windows_x64` | Blender 官方 Windows x64 预编译库（Python、LLVM、OpenEXR、USD、Embree、FFmpeg 等）。体积很大，文件在 Git LFS 里 | <https://projects.blender.org/blender/lib-windows_x64.git>，检出父仓库记录的提交 |
| `lib/windows_x64/spectra` | Spectra，header-only 特征值库。几何节点 `linear_solver_math.cc` 要包含 `include/Spectra/`。MPL-2.0。官方预编译库仓库里没有这一份 | <https://github.com/yixuan/spectra> |

Linux、macOS、Windows ARM64 分别使用 `lib/linux_x64`、`lib/macos_arm64`、`lib/windows_arm64`，地址写在 `.gitmodules`。只编译当前平台的那一个。

### LuxCore 引擎

`WITH_LUXCORE` 默认打开。源码和依赖齐了之后，`intern/luxcore` 会把 LuxCore / SLG / LuxRays 编进 `blender.exe`。缺源码或缺依赖时，配置阶段会跳过内核，引擎只剩一个空壳。

| 放到哪里 | 是什么 | 从哪里拉 |
| --- | --- | --- |
| `extern/luxcore` | LuxCore 源码，子模块分支 `for_v2.11`，当前钉在 `125879e3c9a13b603e9d8ab4cdf4a6c61c0ecdc2`。Apache-2.0，见检出后的 `extern/luxcore/COPYING.txt` | <https://github.com/LuxCoreRender/LuxCore.git> |
| `extern/luxcore` 工作区 | 应用 `patches/luxcore-bikini.patch`。补丁在本仓库里，上游那一提交里没有 | 本仓库 `patches/luxcore-bikini.patch` |
| `extern/luxcore/out/dependencies/full_deploy/host` | LuxCore 自己的依赖：Boost 1.88.0、spdlog 1.17.0、nlohmann_json 3.12.0、robin-hood-hashing 3.11.5。由 Conan 生成，不进 Git | 在 `extern/luxcore` 里执行 `make.bat deps` |

内核查找的标记文件是：

`extern/luxcore/out/dependencies/full_deploy/host/boost/1.88.0/Release/x86_64/include/boost/version.hpp`

`extern/boost` 是 Blender 自带的另一份 Boost，不能代替上面这一份。

### 打开对应功能时才要

| 放到哪里 | 什么时候要 | 从哪里拉 |
| --- | --- | --- |
| `lib/windows_x64/optix`，其中要有 `include/optix.h` | Cycles OptiX。非 Apple 平台默认 `WITH_CYCLES_DEVICE_OPTIX=ON`。找不到头文件时，配置会关掉 OptiX 并给出警告 | <https://github.com/NVIDIA/optix-dev>，或 NVIDIA 提供的完整 OptiX SDK。协议以 SDK 里的 `LICENSE.txt` 为准 |
| `lib/windows_x64/dlss`，其中要有 `include/nvsdk_ngx.h` | DLSS Ray Reconstruction。`make.bat release` 会强制 `WITH_DLSS=ON`；`make.bat full` 保持默认关闭。找不到 SDK 时，该选项会被关掉 | <https://github.com/NVIDIA/DLSS>（包含子模块）。运行库期望在 `lib/Windows_x86_64/rel/` 下，文件名是 `nvngx_dlss.dll`、`nvngx_dlssd.dll`、`nvngx_dlssg.dll`。协议以 SDK 里的 `LICENSE.txt` 为准 |

这三项目录（`spectra`、`optix`、`dlss`）都不在 Blender 官方 `lib-windows_x64` 提交里。

## 自己编译（Windows）

本机已验证的构建是 Visual Studio 2022（vc17）、x64、`full` 配置、Release。产物目录在源码上一级：`build_windows_Full_x64_vc17_Release`，可执行文件是 `bin\Release\blender.exe`。

先安装并放进 `PATH`：

- Visual Studio 2022，含“使用 C++ 的桌面开发”
- CMake
- Git，以及 Git LFS（`git lfs install`）
- Python 3，再执行 `pip install conan`（只给 LuxCore 的 `make.bat deps` 用；Blender 本体用的是 `lib/windows_x64` 里的 Python）

在已经打开的 **x64 Native Tools Command Prompt for VS 2022** 里：

```bat
git clone https://github.com/blueish0930/BIKINI.git
cd BIKINI

git submodule update --init extern/luxcore
git -C extern/luxcore apply ../../patches/luxcore-bikini.patch

git config --local submodule.lib/windows_x64.update checkout
set GIT_LFS_SKIP_SMUDGE=1
git submodule update --progress --init lib/windows_x64
set GIT_LFS_SKIP_SMUDGE=
git -C lib/windows_x64 lfs pull

git clone --depth 1 https://github.com/yixuan/spectra.git lib\windows_x64\spectra
git clone --depth 1 https://github.com/NVIDIA/optix-dev.git lib\windows_x64\optix
git clone --recurse-submodules https://github.com/NVIDIA/DLSS.git lib\windows_x64\dlss

cd extern\luxcore
make.bat deps
cd ..\..

make.bat full 2022
```

`make.bat` 发现 `lib/windows_x64` 还是空目录时，也会询问是否下载。LFS 对象很大，第一次 `lfs pull` 会久一些。

其他入口：

| 命令 | 结果 |
| --- | --- |
| `make.bat 2022` | 默认配置。LuxCore 仍默认打开；DLSS 关闭；OptiX 默认打开 |
| `make.bat full 2022` | 与官方 full 构建同一组功能开关，输出目录带 `Full` |
| `make.bat release 2022` | 与官方 release 构建对应，并强制打开 DLSS 和 OptiX |
| `make.bat nobuild 2022` | 只生成 Visual Studio 工程 |
| `make.bat help` | 全部开关，包括 `lite`、`ninja`、`debug`、`2019`、`2026` |

Linux 或 macOS：初始化对应的 `lib/<platform>` 子模块和 `extern/luxcore`（补丁、`make deps` 同样要做），再在源码根目录执行 `make full`。这个仓库的历史是独立的根提交，和 upstream Blender 没有共同祖先，不要用 `make update` 去快进官方 `main`。
