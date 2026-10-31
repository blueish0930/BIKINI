# BIKINI

BIKINI is a Blender branch for people who build things with nodes. It tracks Blender's daily builds
and adds two new editors, one for textures and one for objects, plus a much wider set of geometry
nodes. Feature notes, node documentation and the VEX Wrangle reference live on the documentation
site; this repository is the source you can compile.

- Documentation: <https://blueish0930.github.io/BIKINI/>
- Source repository: <https://github.com/blueish0930/BIKINI>
- Source tree layout, third-party libraries to fetch and build steps: [BUILDING.txt](BUILDING.txt)
- Third-party license notes: [COPYING.md](COPYING.md)
- Upstream Blender build handbook: <https://developer.blender.org/docs/handbook/building_blender/>

## What BIKINI adds

**A texture editor that runs on the GPU.** The GPU Texture Editor is a node editor for images.
You build a texture as a graph — paint, SDF shapes, fractals, gradients, fluid, geometry
rasterisation, baking, FFT — and the graph is evaluated on the GPU, so a large map such as an 8K
set stays interactive while you change it in the editor.

**An object editor.** The Object Editor is a node tree whose subject is the scene rather than the
mesh. It creates and deletes objects and sets their transform, visibility, material slots,
modifiers, parent and viewport display. Work that used to be a walk through property tabs becomes
a graph you can read, copy and reuse.

**Geometry nodes that reach further.** Much of the node set is aimed at effects that normally mean
writing code: simulation, remeshing, parametrisation, geodesics, point-cloud reconstruction,
optimal transport, sparse linear solvers, SDF and fractal primitives. That is what makes a paper's
algorithm practical to rebuild as a graph. Bijective UV mapping, heat geodesics, RBF interpolation
or quad remeshing are graph work here, not a plugin project.

**The maths is already there.** Vectors, matrices, quaternions, sparse matrices, linear solvers,
expressions and VEX Wrangle are exposed as nodes and as a language, so the primitives a technique
needs are in place before you start. Less time rebuilding the same helper, more time on the part
that is actually new.

**Editing that keeps up with large graphs.** A thumbnail navigator shows the whole tree and takes
you wherever you click, node groups can call themselves, groups can be copied between editors,
links can be drawn with angles and the canvas shows coordinates. Small things, but they are what
keeps a graph of hundreds of nodes readable instead of intimidating.

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

---

# 中文

BIKINI 是给用节点干活的人准备的 Blender 分支。它跟着 Blender 的每日构建走，在上面加了两个新编辑器
——一个管纹理，一个管物体——以及范围大得多的几何节点。功能笔记、节点文档和 VEX Wrangle 参考都在
文档站上；这个仓库是可以编译的源码。

- 文档：<https://blueish0930.github.io/BIKINI/>
- 源码仓库：<https://github.com/blueish0930/BIKINI>
- 源码目录结构、需要自行获取的第三方库与构建步骤：[BUILDING.txt](BUILDING.txt)
- 第三方许可说明：[COPYING.md](COPYING.md)
- 上游 Blender 构建手册：<https://developer.blender.org/docs/handbook/building_blender/>

## BIKINI 加了什么

**跑在 GPU 上的纹理编辑器。** GPU 纹理编辑器是一个针对图像的节点编辑器。纹理用图来搭——绘制、
SDF 形状、分形、渐变、流体、几何栅格化、烘焙、FFT——整张图在 GPU 上求值，所以 8K 这样的贴图集
在编辑器里改参数时依然保持交互。

**物体编辑器。** 物体编辑器的节点树管的是场景，不是网格。它负责创建和删除物体，设置变换、可见性、
材质槽、修改器、父级和视图显示。以前要在属性面板里翻一圈才能做完的事，现在是一张能看懂、能复制、
能复用的图。

**更进一步的几何节点。** 节点集里有很大一部分是冲着那些平时只能写代码的效果去的：模拟、重网格、
参数化、测地距离、点云重建、最优传输、稀疏线性求解、SDF 与分形图元。正因为这些，把论文里的算法
复现成一张图才变得可行——双射 UV、热测地、RBF 插值、四边重网格，在这里都是搭图的工作，而不是一个
插件项目。

**数学工具已经摆在手边。** 向量、矩阵、四元数、稀疏矩阵、线性求解器、表达式和 VEX Wrangle 都开放
成了节点和一门语言，一项技术需要的底层原语在动手之前就已经在了。少花时间重复造轮子，多花时间在
真正新的那部分。

**跟得上大图的编辑体验。** 缩略图导航能看到整棵树，点哪里就跳到哪里；节点组可以调用它自己；节点组
能在不同编辑器之间复制；连线可以走斜角；画布上带坐标。都是小事，但正是这些小事让几百个节点的图
看得懂，而不是望而生畏。

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
