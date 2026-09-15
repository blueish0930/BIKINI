# BIKINI

非官方 Blender **5.3** 便携包（Windows x64），跟踪官方每日版并叠加自用功能。

- 文档：[主页](https://blueish0930.github.io/BIKINI/) · [手册](https://blueish0930.github.io/BIKINI/manual.html) · [更新日志](https://blueish0930.github.io/BIKINI/changelog.html) · 本地 `docs/index.html`
- 源码：[projects.blender.org/blueish/BIKINI](https://projects.blender.org/blueish/BIKINI)
- 站点 / 发布：[github.com/blueish0930/BIKINI](https://github.com/blueish0930/BIKINI)

与 Blender Foundation 无关，未做官方签名。**不是**官方 Blender。

## 使用

1. **整包解压**，不要只拷 `blender.exe`。
2. 运行同目录的 `blender.exe`。
3. 下列内容必须留在 exe 旁边：`5.3/`、`blender.crt/`、`blender.shared/`、`license/`、`bf_intern_*.dll`、`python3.dll` / `python313.dll`；若启用 DLSS，还需保留 `nvngx_dlssd.dll`（根目录与 `5.3/scripts/addons_core/cycles/` 各有一份）。

详见 `BIKINI_BUILD_INFO.txt`。节点参数见手册。

## 近期（相对 v2 之后）

- **DLSS 4.5**：Cycles / 合成器降噪路径接入 NVIDIA DLSS（需兼容 NVIDIA GPU；二进制闭源，见下方协议说明）
- **节点缩略图**：节点编辑器内预览缩略图
- **Layered texture painting**、GTE refresh
- Shader **ddx / ddy**（Derivative）等

更早的 v1 / v2 功能列表见下文。

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
- Terrain / Erosion（盖亚风格：山体图元、水力/热力侵蚀、雪/河/海、阶地、坡度遮罩、卫星着色）

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

完整许可证文本在 `license/`（含 `license/license.md` 与 `license/others/`）。

| 库 | 版本 / 说明 | 许可证 | 用在 | 出处 |
|----|-------------|--------|------|------|
| [CGAL](https://www.cgal.org) | 6.2 | GPL-3.0-or-later / LGPL | 几何节点 CGAL | [cgal.org](https://www.cgal.org) · [GitHub](https://github.com/CGAL/cgal) |
| [Box2D](https://box2d.org) | 3 | MIT · Erin Catto | Box Engine 2D | [box2d.org](https://box2d.org) · [GitHub](https://github.com/erincatto/box2d) |
| [Box3D](https://github.com/erincatto/box3d) | — | MIT · Erin Catto | Box Engine 3D | [GitHub](https://github.com/erincatto/box3d) |
| [Jolt Physics](https://github.com/jrouwe/JoltPhysics) | 5.6.0 | MIT · Jorrit Rouwe | Jolt Solver | [GitHub](https://github.com/jrouwe/JoltPhysics) |
| Eigen | — | MPL-2.0 | 稀疏线性代数 | |
| Spectra | — | MPL-2.0 | 特征分解 | |
| Voro++ | — | BSD-3-Clause | Voronoi 破碎 | |
| Instant Meshes / QuadWild | — | GPL-3 | 重网格 | 见 `quadwild/README.txt` |
| **NVIDIA DLSS / NGX** | **4.5（`nvngx_dlssd.dll`）** | **专有（非开源）** | Cycles / 合成器 DLSS 降噪 | [NVIDIA DLSS](https://github.com/NVIDIA/DLSS) · 全文见 `license/others/NVIDIA-RTX-SDK.txt` |

引用：

- CGAL — The CGAL Project. *CGAL User and Reference Manual*, 6.2. <https://www.cgal.org>
- Box2D / Box3D — Erin Catto. <https://box2d.org> · <https://github.com/erincatto/box3d>
- Jolt Physics — Jorrit Rouwe, 5.6.0. <https://github.com/jrouwe/JoltPhysics>
- NVIDIA DLSS / NGX — © NVIDIA Corporation. Redistributable object code under the NVIDIA RTX SDKs License. <https://github.com/NVIDIA/DLSS>

源码目录（开源部分）：`extern/cgal`、`extern/box2d`、`extern/box3d`、`extern/jolt`。DLSS 仅以二进制 DLL 形式附带，**无对应开源树**。

## 许可证

这是 **GPL 衍生作品**，不是 MIT 小工具。

Blender 本体是 GPL-2.0-or-later，改过的二进制必须能提供对应源码。CGAL、QuadWild 是 GPL-3，链进 exe 后整包按 GPL-3 分发。Box2D / Box3D / Jolt 是 MIT，保留声明即可。分发时请保留整个 `license/` 目录，**不要**声称这是官方 Blender。

### 特别说明：NVIDIA DLSS 4.5（专有组件）

本构建可选包含 **NVIDIA DLSS / NGX** 运行库（`nvngx_dlssd.dll`），用于降噪等功能。请注意：

1. **不是开源软件**。DLSS / NGX 受 NVIDIA RTX SDKs License 约束，版权归 NVIDIA Corporation；与 GPL 组件**并存**，但 **DLSS 二进制本身不因同包分发而变成 GPL，也不得被“传染”为开源义务**（NVIDIA 许可证明确禁止以会导致 SDK 适用开源许可证的方式使用）。
2. **只能作为本应用的一部分、以目标代码形式分发**，不得单独拆出 `nvngx_dlssd.dll` 当作独立产品再分发；不得对 DLL 做逆向工程、去版权声明等。
3. **面向 NVIDIA GPU**。DLSS / NGX 功能依赖兼容的 NVIDIA 硬件与驱动；在非 NVIDIA 设备上相关选项可能不可用或自动回退。
4. **商标与背书**。除非另有 NVIDIA 书面许可，不得暗示本构建由 NVIDIA 赞助或背书。可按 NVIDIA 许可证要求在 About / 启动相关说明中标注使用了 DLSS / NGX。
5. **完整条款**见 `license/others/NVIDIA-RTX-SDK.txt`（NVIDIA RTX SDKs License，含 DLSS / NGX；v. March 14, 2024）。官方原文：<https://github.com/NVIDIA/DLSS/blob/main/LICENSE.txt>。商业发布前请按该许可证履行 NVIDIA 通知等义务（见补充条款中的 Notification）。

若你分发本便携包：请保留上述 DLL 与 `license/others/NVIDIA-RTX-SDK.txt`；若你**删除** DLSS DLL，则本专有条款对该副本不再适用，但其余 GPL / 第三方声明仍须保留。