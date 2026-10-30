# BIKINI

BIKINI is a source branch that tracks Blender's daily builds, aimed at procedural work: it pushes
geometry nodes further, adds editors that were missing, and makes large node graphs comfortable to
build and read. Feature notes, node documentation and the VEX Wrangle reference live on the
documentation site; this repository holds the source that can be compiled.

- Documentation: <https://blueish0930.github.io/BIKINI/>
- Source repository: <https://github.com/blueish0930/BIKINI>
- Source tree layout, third-party libraries to fetch and build steps: [BUILDING.txt](BUILDING.txt)
- Third-party license notes: [COPYING.md](COPYING.md)
- Upstream Blender build handbook: <https://developer.blender.org/docs/handbook/building_blender/>

## Highlights

- **A GPU texture editor.** Textures are painted and edited on the GPU, so large images stay
  responsive at full resolution while you work on them.
- **A dedicated object editor.** Object-level data gets an editor of its own instead of being
  scattered across property tabs.
- **Geometry nodes taken further.** The node system reaches the stages that normally force a
  custom build — advanced simulation, native solver kernels, heavy per-element evaluation — so
  elaborate effects are assembled from nodes rather than from compiled code.
- **A paper-sized math toolkit.** Vectors, matrices, quaternions, fields, iteration and solver
  primitives are exposed both as nodes and in the VEX Wrangle language. Reproducing an algorithm
  from a paper becomes graph work instead of another wheel being rebuilt.
- **Editing that stays out of the way.** Node thumbnails, recursive node groups, in-place group
  editing and a searchable library keep large graphs legible; the interaction design favours
  direct manipulation over stacked dialogs and property hunting.

## 项目优势

- **GPU 纹理编辑器。** 纹理直接在 GPU 上绘制与编辑，大尺寸图像也能在全分辨率下保持流畅。
- **独立的物体编辑器。** 物体层级的数据有了自己的编辑器，不再散落在各个属性面板里。
- **几何节点更进一步。** 节点系统覆盖了以往必须写代码才能完成的环节——高级模拟、原生求解内核、
  大规模的逐元素求值——复杂效果可以直接用节点搭出来，而不必再编译自定义代码。
- **论文级的数学工具。** 向量、矩阵、四元数、场、迭代与求解器原语，既是节点，也是 VEX Wrangle
  语言的一部分。复现论文里的算法变成了搭图的工作，而不是又一次造轮子。
- **顺手而不打扰的编辑体验。** 节点缩略图、节点组递归、就地编辑节点组、可搜索的节点库，让大图
  依然可读；交互设计偏向直接操作，而不是层层弹窗和翻属性。

## Download

Ready-to-run builds are published on the Releases page. Open the latest **Daily Build** release
(<https://github.com/blueish0930/BIKINI/releases>) and download `Release.rar` — that archive is
the compiled BIKINI itself, with the third-party libraries and submodules it needs already packed
inside. Extract it to a local folder, keep the directory layout intact, and start `blender.exe`.

The other assets in the same release:

| Asset | Contents |
| --- | --- |
| `Release.rar` | The BIKINI build: `blender.exe`, the `4.0/` runtime data, and the prebuilt third-party libraries (including `lib/windows_x64` and `extern/luxcore`). Nothing else has to be downloaded or compiled. |
| `BIKINI.Demo.Files.rar` | The demo files — the scenes used by the documentation and by the node / VEX Wrangle examples. Extract it anywhere and open the `.blend` files from inside BIKINI. |
| `BIKINI-build-dependencies-v4.0.1.zip` | Prebuilt dependency libraries for compiling the source tree yourself. Only needed for a local build, not for running `Release.rar`. |

Building BIKINI from this repository is described in [BUILDING.txt](BUILDING.txt).

## 下载

可直接运行的版本发布在 Releases 页面。打开最新的 **Daily Build** 发布页
（<https://github.com/blueish0930/BIKINI/releases>），下载 `Release.rar`——这个压缩包就是编译好的
BIKINI 本体，它所需的第三方库和子模块已经一并打包在里面。解压到本地目录，保持目录结构完整，运行
`blender.exe` 即可。

同一个发布页里的其他文件：

| 文件 | 内容 |
| --- | --- |
| `Release.rar` | BIKINI 本体：`blender.exe`、`4.0/` 运行数据，以及预编译的第三方库（含 `lib/windows_x64` 与 `extern/luxcore`）。不需要再下载或编译任何东西。 |
| `BIKINI.Demo.Files.rar` | 演示文件，也就是文档、节点与 VEX Wrangle 示例所用的场景。解压到任意位置，用 BIKINI 打开其中的 `.blend` 文件。 |
| `BIKINI-build-dependencies-v4.0.1.zip` | 自行从源码编译时用的预编译依赖库。只有本地构建需要，运行 `Release.rar` 时用不到。 |

从本仓库自行编译的步骤见 [BUILDING.txt](BUILDING.txt)。
