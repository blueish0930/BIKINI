# BIKINI

**Unofficial custom Blender build** — Windows x64 portable, tracking **Blender 5.3 Alpha / main**.

专注于 **Geometry Nodes**、**稀疏数学求解**、**Image Process（COP 向图像节点）** 与 **节点编辑器 UX** 的实验分支。随 daily 合入上游，在主线之上做自己真正想用的节点与工具。

> Token 在燃烧，谁来帮我分担一下~

---

## 文档与站点

| 链接 | 说明 |
|------|------|
| **[GitHub Pages · 主页](https://blueish0930.github.io/BIKINI/)** | 项目介绍与入口 |
| **[Release Notes 更新日志](https://blueish0930.github.io/BIKINI/changelog.html)** | 按「每期 → 模块」组织（Geometry Nodes / Math & Solver / COP / Shaders / UI…），**中英切换** |
| [仓库内 docs/](docs/) | 同源文档（离线也可打开 `docs/changelog.html`） |
| [上游 Blender 5.3 Notes](https://developer.blender.org/docs/release_notes/5.3/) | 官方 release notes 结构参考 |

---

## 开发目标

BIKINI 不是功能堆砌包，而是围绕下面几条线长期演进的 **个人 / 实验向 Blender 构建**：

### 1. 跟主线，不当死分支

- 基线：**Blender 5.3 Alpha**，持续同步 `main` / daily。
- 自定义功能以「可维护的增量」加在节点、编辑器与数学库上，而不是整树硬分叉。

### 2. Geometry Nodes 作为中枢

把「在几何上能算的」尽量放进节点图，形成可组合管线，例如：

- **Named Portal**、**Geometry Clip**、**Loop Subdivision**、**K-Nearest**、**Time Shift**、**Debug**…
- **SDF Grid** 系列（Boolean / Fillet / Laplacian / Mean / Offset…）
- 与主线 Zones 增强：**Repeat Break**、**Simulation Cache Limit**

### 3. 稀疏线性代数（可解、可特征）

面向平滑、扩散、钉约束、谱处理等几何问题：

```text
Mesh → Mesh Laplacian (COO) → Sparse Matrix Math → Linear Solver → x / eigenpairs
```

- **Mesh Laplacian**：离散拉普拉斯以 **COO** 稀疏 Bundle 存储（`weight` / `row` / `col`），Uniform / Cotangent，可选 `I+tL` 与质量矩阵。
- **Linear Solver**：引入 **Eigen**（稀疏直接 / 迭代）与 **Spectra**（部分特征、shift-invert）；模式参考 Houdini 式 Solve / Decompose / SWD / Multiply。
- **原则**：端到端稀疏，避免大网格 densify。

### 4. Image Process = COP 向图像节点编辑器

独立节点树 **`ImageNodeTree`（UI 名 Image Process）**，专门处理图片与栅格数据，而不是挤在经典场景合成器里：

- Point Stamp、Rasterize / Import Points  
- Normal ↔ Height、Bake Image（多格式）  
- SDF Shape / Fractal Primitive（Image 路径）  
- Fluid Simulation Zone、Bundle 工具  

语义上接近 Houdini **COPs**：为贴图、点云盖章、高度图与烘焙服务。

### 5. 着色器与视口

- **SDF Shape / Fractal Primitive**（Inigo Quilez 公式族），Shader 与 Image 共享数学核；EEVEE / GPU 材质路径。
- 视口属性 / Viewer 叠加、Spreadsheet 列排序等调试体验。

### 6. 节点编辑器 UX

少摩擦地搭图：

- 节点组 **Separator / Message / 参数并排 / Ctrl+LMB 改名 / Enable I/O**  
- **抖动拆线**、**按住 U 拖拽对齐**  
- **String 属性** 全链路  

### 7. 便携分发

- 解压即用的 **Windows x64** 目录结构（`blender.exe` + `5.3/` + 运行时）。
- 文档与构建信息随仓；**GitHub Pages** 提供可读的更新说明。

---

## 下载与使用

1. 打开仓库 → **Code → Download ZIP**（或用上方 Pages 里的下载入口）。  
2. 解压整个目录（不要只拷 `blender.exe`）。  
3. 运行 `blender.exe`。  

**必须与下列内容放在同一目录树：**

- `5.3/`（脚本、Python、datafiles）  
- `blender.crt/`、`blender.shared/`、`license/`  

详见 [`BIKINI_BUILD_INFO.txt`](BIKINI_BUILD_INFO.txt)。

> 这是**非官方**自定义构建，**未经 Blender Foundation 数字签名**。请勿当作官方发行版宣传或替换系统安装包。

---

## 仓库结构（摘要）

| 路径 | 说明 |
|------|------|
| `blender.exe` | 主程序（便携） |
| `5.3/` | Blender 运行时数据与脚本 |
| `docs/` | **GitHub Pages 源**：主页 + Release Notes |
| `BIKINI_BUILD_INFO.txt` | 包日期与简短变更 |
| `README.md` | 本说明 |

Pages 从分支的 **`/docs`** 目录发布（见下方）。

---

## GitHub Pages

站点地址：

- https://blueish0930.github.io/BIKINI/  
- https://blueish0930.github.io/BIKINI/changelog.html  

源文件：

- [`docs/index.html`](docs/index.html) — 落地页  
- [`docs/changelog.html`](docs/changelog.html) — 更新日志（仿 [developer.blender.org release notes](https://developer.blender.org/docs/release_notes/5.3/)：每期 → 模块 → 条目）  

本地预览：用浏览器直接打开 `docs/index.html` / `docs/changelog.html` 即可（纯静态，不依赖 Blender）。

---

## 状态与贡献

- **状态**：个人实验构建，API / 节点 / 打包方式可能随时变。  
- **平台**：当前公开包以 **Windows x64** 为主。  
- **反馈**：Issues / Discussion 欢迎；PR 需能在你自己的构建树上验证。  
- **上游**：功能最终若成熟，会优先考虑回馈或对齐 Blender 主线设计，而不是永久私有分叉。

---

## 许可证与归属

- Blender 本体遵循其 [许可证与第三方声明](license/)（GNU GPL 等）。  
- 本仓库中的自定义节点与文档说明以 GPL 兼容方式随构建分发；第三方数学库（如 Eigen、Spectra）遵循各自许可证。  
- **BIKINI** 名称与本仓库仅标识此非官方构建，与 Blender Foundation 无隶属关系。

---

## 快速链接

- [Pages 主页](https://blueish0930.github.io/BIKINI/)  
- [更新日志](https://blueish0930.github.io/BIKINI/changelog.html)  
- [Blender 官网](https://www.blender.org/)  
- [Blender 5.3 Geometry Nodes Notes](https://developer.blender.org/docs/release_notes/5.3/geometry_nodes/)  
