# BIKINI

非官方 Blender **5.3** 便携包（Windows x64），每日版同步更。

- 文档：[主页](https://blueish0930.github.io/BIKINI/) · [手册](https://blueish0930.github.io/BIKINI/manual.html) · [更新日志](https://blueish0930.github.io/BIKINI/changelog.html) · 本地 `docs/index.html`
- 源码：[projects.blender.org/blueish/BIKINI](https://projects.blender.org/blueish/BIKINI)
- 站点：[github.com/blueish0930/BIKINI](https://github.com/blueish0930/BIKINI)

与 Blender Foundation 无关，未做官方签名。

## 使用

1. **整包解压**，不要只拷 `blender.exe`。
2. 运行同目录的 `blender.exe`。
3. `5.3/`、`blender.crt/`、`blender.shared/`、`license/`、`bf_intern_*.dll` 必须留在 exe 旁边。

详见 `BIKINI_BUILD_INFO.txt`。节点参数见手册。

---

## v1

相对官方 5.3 第一批改动。

**几何节点**

- String 属性、Portal、几何 Clip、设置默认闭包
- Loop 细分、Nearest Neighbours、Debug、Time Shift
- 模拟区 Cache Limit；Repeat Zone Break
- Sparse Matrix Math、Linear Solver、Mesh Laplace
- Object Info Seed
- Select / Edit Element
- Heat Geodesic、切向场、Gradient / Divergence
- Instant Meshes、QuadWild、Triangle Remesh
- Voronoi 破碎、Delaunay 3D、图染色
- 曲线求交、Write at Index、RBF Interpolate
- Expression（自动补全）
- 组接口 enable 优化

**界面**

- 视口叠加层属性预览
- 节点组 Separator / Message；参数换行；Ctrl+LMB 改名
- 抖动拆线、一次插入多个节点
- 按住 U 拖动对齐
- 电子表格属性排序
- 组 Menu 多选，输出 List
- Ctrl+LMB 改接口数据类型，Alt+LMB 改接口形状（v2 改为 Shift+LMB 改类型）

**着色器**

- Image Socket、SDF Shape、Fractal、Expression、HLSL

**编辑器**

- Data-Block Graph
- 合成器里的 GPU Texture Editor 组节点

**GPU Texture Editor**

- Import Points、Point Stamp
- Normal ↔ Height
- Simulation / Repeat、流体输入输出
- Rasterize Geometry
- Sample / Write at Pixel、Paint、Histogram
- SDF Shape、Fractal
- Bake Image、Image Output

---

## v2

**几何节点**

- CGAL 计算几何（Add → Bikini → Lib → CGAL）
- Box Engine（Box2D 二维、Box3D 三维）、Jolt 三维刚体
- Set Group Input Default、Guide Geometry、Attribute Transfer
- Get / Set Vector Component、Get / Set Matrix Component
- Make It Stand、Optimal Transport

**GPU Texture Editor**

- Camera View、Island UV、Island Padding
- Portal、FFT、Import Geo、Geo SDF
- Render Material、ShaderToy

**着色器**

- Portal、字符串节点

**物体编辑器**

- 用节点创建 / 引用物体，改变换、可见性、材质槽、修改器、父级、删除

**界面**

- G 只移动，不插入连线
- 改名高亮
- Shift+LMB 改接口数据类型（替代 v1 的 Ctrl+LMB）
- 不同编辑器之间复制粘贴节点组
- Dirty 评估（不再整树重煮）；Ctrl 点击预览
- 电子表格 Group / Attribute Filter
- Drag Search 建组不再整树重评估
- 3D 视口纹理绘制模式可画 8K（不是 GTE）

---

## 第三方库

完整文本在 `license/`。

| 库 | 版本 | 许可证 | 用在 | 出处 |
|----|------|--------|------|------|
| [CGAL](https://www.cgal.org) | 6.2 | GPL-3.0-or-later / LGPL | 几何节点 CGAL | [cgal.org](https://www.cgal.org) · [GitHub](https://github.com/CGAL/cgal) |
| [Box2D](https://box2d.org) | 3 | MIT · Erin Catto | Box Engine 2D | [box2d.org](https://box2d.org) · [GitHub](https://github.com/erincatto/box2d) |
| [Box3D](https://github.com/erincatto/box3d) | — | MIT · Erin Catto | Box Engine 3D | [GitHub](https://github.com/erincatto/box3d) |
| [Jolt Physics](https://github.com/jrouwe/JoltPhysics) | 5.6.0 | MIT · Jorrit Rouwe | Jolt Solver | [GitHub](https://github.com/jrouwe/JoltPhysics) |
| Eigen | — | MPL-2.0 | 稀疏线性代数 | |
| Spectra | — | MPL-2.0 | 特征分解 | |
| Voro++ | — | BSD-3-Clause | Voronoi 破碎 | |
| Instant Meshes / QuadWild | — | GPL-3 | 重网格 | 见 `quadwild/README.txt` |

引用：

- CGAL — The CGAL Project. *CGAL User and Reference Manual*, 6.2. <https://www.cgal.org>
- Box2D / Box3D — Erin Catto. <https://box2d.org> · <https://github.com/erincatto/box3d>
- Jolt Physics — Jorrit Rouwe, 5.6.0. <https://github.com/jrouwe/JoltPhysics>

源码目录：`extern/cgal`、`extern/box2d`、`extern/box3d`、`extern/jolt`。

## 许可证

这是 GPL 衍生作品，不是 MIT 小工具。

Blender 本体是 GPL-2.0-or-later，改过的二进制必须能提供源码。CGAL、QuadWild 是 GPL-3，链进 exe 后整包按 GPL-3 分发。Box2D / Box3D / Jolt 是 MIT，声明留着即可。分发时保留 `license/`，不要声称官方 Blender。
