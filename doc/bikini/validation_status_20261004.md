# BIKINI daily 合并后验证状态（2026-10-04 复验）

运行程序：`E:\build_windows_Full_x64_vc17_Release\bin\Release\blender.exe`
SHA-256 `4A0802341C257B680EA0B504613EC721358FE9938C3DAA919C4B6565B95609EB`（135,745,536 字节，链接于 2026-10-04 21:37）。
内嵌 buildinfo 提交 `5265b82a252c`。

## 这一轮改了什么

上一轮验收（见同目录旧版 `VALIDATION_STATUS.md`）留下的结论是 14 个 Demo 里 5 个不通过、
9 个只做到「能打开 + 跑一次当前帧求值」。这次查清了根因并修掉了：

1. **40 个 CGAL 节点的注册被作者停用。** 源码里写成
   `/* Deleted: xxx */` 后面跟 `return;`，函数体其实都还在。
   停用的原因写在注释里：一批是改名/合并（Hole Fill → Fair Hole Fill、
   Point Region Growing 等并入 Point Shape Fitting），一批标的是 `deleted`。
2. **这 40 个节点在 `makesrna/intern/rna_nodetree.cc` 里的 `define(...)` 同样是注释掉的。**
   只恢复第 1 步没用：运行时会把它们判成「无 RNA 结构」并跳过注册，
   所以旧文件里依然显示 `NodeUndefined`。两步必须一起做。
3. **4 个周期节点是被合并掉的，不是被删的。** RNA 注释写得很清楚：
   `GeometryNodeCgalPeriodicDelaunay2 merged into GeometryNodeCgalDelaunay2`、
   `PeriodicVoronoi2 merged into GeometryNodeCgalVoronoi2`，3D 同理。
   也就是说这些功能现在挂在 Delaunay/Voronoi 节点的 `Periodic` 开关上，
   但旧 Demo 存的是独立的节点类型名，所以旧文件里它们是 `NodeUndefined`。
   本轮按手册的接口把它们重新做成独立节点。

## 本轮结果

14 个 `.blend` 全部：加载成功、当前帧依赖图求值成功、脚本正常退出、退出码均为 0。
共盘点 **4,444 个节点**。
**除 Physics 外，全部文件 `NodeUndefined` 为 0。**

| 文件 | 节点数 | NodeUndefined | CGAL 类型实例化 | 外部缺失 |
|------|-------:|--------------:|----------------|---------:|
| V1\Showcase V1.blend | 1167 | 0 | 183/183 | 11 |
| V2\CGAL.blend | 1603 | 0 | 183/183 | 10 |
| V2\Dirty Evaluation.blend | 111 | 0 | 183/183 | 4 |
| V2\Object Editor.blend | 14 | 0 | 183/183 | 0 |
| V2\Physics.blend | 137 | **2** | 183/183 | 0 |
| V2\shadertoy.blend | 112 | 0 | 183/183 | 14 |
| V2\Showcase V2.blend | 186 | 0 | 183/183 | 4 |
| V3\Curve Ramp Sockets.blend | 42 | 0 | 183/183 | 11 |
| V3\FLIP_Dam_Break_Demo.blend | 33 | 0 | 183/183 | 0 |
| V3\GN wrangle.blend | 145 | 0 | 183/183 | 13 |
| V3\modal_tool.blend | 806 | 0 | 183/183 | 11 |
| V3\shader wrangle.blend | 22 | 0 | 183/183 | 11 |
| V3\Showcase V3.blend | 43 | 0 | 183/183 | 12 |
| V4\node_group_fibonacci_recursion.blend | 23 | 0 | 183/183 | 0 |

「CGAL 类型实例化」= 在一个临时 GeometryNodeTree 里对二进制暴露的每个
`GeometryNodeCgal*` 类型调用一次 `nodes.new()`，183 个全部创建成功、0 失败。
改动前是 139 个（且 40 个恢复的节点会因缺 RNA 被运行时跳过）。

`node_group_fibonacci_recursion.blend` 求值得到 **451 顶点 / 439 面**，
与上一轮记录一致，递归求值路径没有回归。

## 仍然未通过 / 未解决

1. **`V2\Physics.blend` — 未通过。** `LiquidFun Solver`、`MPM Solver` 两个节点仍然
   `NodeUndefined`。这两个节点**从来没有实现过**：整棵树里只有编号常量
   （`GEO_NODE_LIQUIDFUN_SOLVER 2205`、`GEO_NODE_MPM_SOLVER 2206`），
   没有对应的 `node_geo_*.cc`、没有 RNA 定义、没有 geometry 求解器实现。
   要让这个文件通过，等于从零写两个求解器；本轮没有做。
   `Box2D_Stack` 物体求值为 0 顶点，也与该文件依赖这些缺失节点一致。

2. **`V3\FLIP_Dam_Break_Demo.blend` — 未通过（未能取得有效证据）。**
   `Water` 物体的 `FLIP` 修改器（节点组 `SOP FLIP Dam Break`）在帧
   1/2/5/30/69/70/71/72 上求值均为 0 顶点 / 0 面。节点组结构本身是完整的：
   `GeometryNodeSimulationInput`（Previous Frame）→ `GeometryNodeFlipSolver`
   → `GeometryNodeSimulationOutput`（Store Frame），种子来自 `Mesh to Points`，
   另外还有 4 个 `GeometryNodeBake` 节点（`Bake`、`Bake.001`、`Bake.002`、`Bake.003`）。
   文件 429,646,309 字节，其中绝大部分就是这 4 个 Bake 节点的烘焙数据。

   **重要说明（本轮踩到的坑）**：我第一次测这个文件时，磁盘上的 FLIP 文件已经被截断成
   136,757 字节（只剩场景，4 个 Bake 节点连数据一起没了），所以当时「0 顶点」的结论
   是在残缺文件上得出的，不能算数。已从 GitHub release 的 `BIKINI.Demo.Files.rar`
   还原为 429,646,309 字节（`LastWriteTime` 保留 2026-09-28 16:14:52）。
   **在还原后的真文件上复测，`Water` 依然是 0 顶点 / 0 面**，所以这是真实存在、
   尚未解决的问题：烘焙缓存不在仓库里（`E:\BIKINI Demo Files` 下没有任何
   cache/bake 目录），需要在 GUI 里重新烘焙一次才能判定求解器是否正确。
   `bpy.ops.object.bake_simulation_geometry` 在这个版本里不存在，后台触发不了烘焙。

   还原后我用 SHA-256 逐项验证：`accept_probe`、`dump_graph`、以及只加载不操作，
   三种方式都不会改动该文件（前后哈希一致，均为 `FE3CA1F7A2B75621…`）。

3. **退出内存泄漏没有修。** 上一轮记录：`Showcase V1` 退出报 2 个未释放内存块、
   `shadertoy` 报 46 个、`shader wrangle` 报 1 个。数量很小
   （合计 <0.04 MB），形态是 `NODE_DECL_ALLOC_STATIC`（节点声明期一次性静态分配），
   属于低危退出期泄漏。本轮重点放在节点可用性上，没有动这部分，
   需要在最终验收里逐文件复核当前是否仍然存在。

4. **外部引用缺失是场景自己记录的旧路径，不是构建问题。**
   所有 `.blend` 原件都没有被我修改。缺的资产分两类：
   - **能修的**：链入的资产库 `geometry_nodes_essentials.blend`、
     `geometry_nodes_dynamics_assets.blend`、`procedural_hair_node_assets.blend`
     指向旧机器的 `D:\UGit`、`E:\Github`、微信目录，但真实文件就在
     `E:\Blender_Source\assets\nodes\`，运行目录 `5.3\datafiles\assets\nodes\` 里也有。
   - **修不了的**：图片资源（`bob_diffuse.png`、`spot_texture.png`、
     一批 `testgeometry_*.jpg`、`download.png`、`下载 (*).png`）在本机 C/D/E 盘
     都不存在，是用户自己的外部素材，只能由用户补。
     `geometry_nodes_category_layout.blend` 连源码树里也没有。

   已按「不覆盖原件」的要求把可修的链路修好，另存到：
   `E:\build_windows_Full_x64_vc17_Release\bin\Release\_repaired_demos\`
   修复结果：Showcase V1 6/6 全修复；CGAL 4/6（余 2 条 category_layout）；
   GN wrangle 2/2 全修复。每个文件旁边有 `.remap.json` 记录改了什么。

## 源码改动与回退

本轮改了 `E:\Blender_Source`：

- 40 个 `source/blender/nodes/geometry/nodes/node_geo_cgal_*.cc`：去掉停用守卫
- `source/blender/makesrna/intern/rna_nodetree.cc`：44 行 `define(...)` 恢复生效
- 新增 4 个周期节点 `.cc` + `CMakeLists.txt` 登记
- `BKE_node_legacy_types.hh`：无净改动（用的本来就是文件里已有的周期编号）

回退步骤和「只停用某一个节点」的做法写在
`REVERT_NODES_20261004.md`，原始文件备份在
`E:\build_windows_Full_x64_vc17_Release\bin\Release\_node_register_backup\`。

## 证据位置

- 逐文件原始结果：`E:\build_windows_Full_x64_vc17_Release\bin\Release\_accept_final\*.json`
- 节点注册表（二进制暴露的 183 个 CGAL 类型）：`_bintypes_periodic.json`
- FLIP 节点图与模拟探测：`_flip_graph.json`、`_flip_probe2.json`

## 下一轮建议顺序

1. 复核三个文件退出时的内存泄漏计数是否仍在（低危，但要求里写了「退出也不许报错」）。
2. 决定 Physics 里的 LiquidFun / MPM：要么实现，要么把这个 Demo 的验收范围
   明确标为「依赖未实现节点」并从「必须通过」里排除。
3. FLIP 必须在 GUI 里走一次真实烘焙/播放再判定。
4. 图片素材由用户提供后路径才能补齐。

## 构建环境注意

本机环境变量同时存在大小写两份代理变量
（`NO_PROXY`/`no_proxy`、`HTTP_PROXY`/`http_proxy`、`HTTPS_PROXY`/`https_proxy`），
MSBuild 读环境变量时会抛
`已添加项。字典中的关键字:"NO_PROXY"所添加的关键字:"no_proxy"` 而失败。
所以不要直接跑 `cmake --build`，要用 `_build_clean.py`（先清掉小写那三个再调 cmake）。
失败的构建还会留下 MSBuild 僵尸进程锁住 `.obj`，下次构建前先
`Get-Process MSBuild | Stop-Process -Force`。
