# 动态过滤第一套候选

日期：2026-10-03。状态：四包运行及初步几何检查完成，场景验收进行中。baseline 为 `9a11f2e`；开发分支为 `feature/dynamic-object-filtering`。

## 实现与范围

`scripts/filter_dynamic_map.py` 是基于实测射线邻域的自定义离线可见性过滤，使用 SciPy cKDTree 查询和 Open3D PCD 解析。它借鉴多时段占据变化的研究方向，但不是 Removert、ERASOR 或 Dynablox 的代码移植或复现。

使用原前端与优化后轨迹计算每帧世界坐标修正，并根据 `extrinsic_T` 得到雷达原点。检查全部捕获输出帧的同时间戳关联，按至少 0.3 s 间隔选取证据帧；地图始终来自全部帧重建的 3 cm baseline。每个 2 s 时间段中，同一点的命中和自由空间证据各至多计一次。这种分段用于减少高频重复投票，不代表统计上完全独立。

地图点的实际射线邻域需满足以下条件，才累计自由空间证据：

- 至少三个测得回波在 0.7° 角度锥内。
- 邻近回波都在候选点后方，超过 `max(0.20 m, 0.02 × 距离)` 的余量。
- 回波深度跨度小于 `0.25 m + 0.01 × 距离`，减少深度边界插值。
- 该邻域没有 0.10 m 内的候选点端点支持。

没有测量、近处遮挡和证据不足都不计为空闲。处理半径为雷达原点周围 20 m，半径外点保留。最终移除要求至少四个自由证据时间段、自由票占命中加自由票的比例至少 0.8、证据时间跨度至少 6 s，且命中支持最多三个时间段。四个及以上时间段重复支持的点均保留。

输出是原 baseline 的精确点子集，没有改变保留点的坐标、强度或轨迹，也没有额外体素化、空间裁剪或删除最后一段数据。可见性由角度邻域近似，仍可能受薄结构、玻璃、位姿残差和部分遮挡影响。长期停留的人或证据不足的动态物体可能保留；只有短暂可见的静态物体可能误删。

## 四包实测

| 包时间 | baseline 点数 | 候选保留点 | 移除点 | 移除比例 | 证据帧 / 全部输出帧 | 初次证据运行耗时 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 10-01 21:52:47 | 589414 | 564190 | 25224 | 4.280% | 633 / 2253 | 356.8 s |
| 10-02 16:21:02 | 517521 | 501486 | 16035 | 3.098% | 355 / 1261 | 196.7 s |
| 10-02 16:23:42 | 645399 | 630478 | 14921 | 2.312% | 670 / 2387 | 402.7 s |
| 10-02 16:27:44 | 503873 | 492191 | 11682 | 2.318% | 247 / 879 | 127.8 s |

这些是删除统计，不是动态物体检出率。初次证据运行峰值 RSS 约 488–523 MiB；复用证据重新分类约 0.96–1.30 s，不包含结构审计、绘图或三维加载。完整报告区分本次运行耗时和复用证据的原运行耗时。

四包的保留点与移除点已通过精确分区检查，baseline 文件哈希不变，所有被至少四个时间段支持的点均保留。局部 PCA 几何平面候选抽样的移除量为：65/30719、12/31856、51/24538、35/13328。这些候选未被人工确认全部静态，不能将其损失率称为真实静态误删率。

第三包 18 个既有固定表面检查区域中，16 个有点的区域完全保留；一个区域原本无点；一个 78 点区域全部移除。追查发现该区域仅在约第 223–225 s 有端点支持，其他 11 个时间窗口都没有 8 cm 内支持，且具有多个自由证据时间段，因此原“平面候选”标签不能直接当作静态真值。仍需人工确认该区域的实际物体。

第一版较稀疏/保守的第三包对照只移除 428 点；较密集观测的版本移除 15007 点，静态支持保护又恢复了其中 86 点。没有将删除量增加本身当作质量提高的证明。

## 复现现有四包结果

在已有 baseline 实验根目录上运行，源码仓库本身不包含 bag、缓存或地图：

```bash
cd yhbuildmap_ws
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/run_dynamic_filtering.py \
  --data-root /path/to/existing/buildmap_test_ws \
  --output-root "$PWD/results/dynamic_delivery"
```

脚本先核对 `config/dynamic_filtering_recordings.json` 中四张地图与轨迹哈希，再逐包使用相同参数过滤和审计。`--id 162342` 可只处理第三包。已有同参数证据时，可使用 `--reuse-root`，目录布局为 `<reuse-root>/<id>/visibility_v2/`；复用时检查来源、地图与轨迹哈希、帧数和采样设置。输出目录需尚不存在。

直接处理单个前端和后端目录：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/filter_dynamic_map.py \
  /path/to/frontend /path/to/coverage_geometry_v5 \
  --frame-step 0.3 --angle-deg 0.7 --max-hit-bins 3 \
  --output-dir "$PWD/results/single_dynamic"
/usr/bin/python3 scripts/audit_dynamic_filter.py \
  /path/to/coverage_geometry_v5/optimized_3cm.pcd results/single_dynamic
/usr/bin/python3 scripts/view_pcd.py results/single_dynamic/filtered_3cm.pcd
```

`filter_dynamic_map.py` 单独调用的默认采样间隔/角度是更保守的 0.75 s / 0.4°；复现表中结果必须提供上面的 0.3 s / 0.7°，四包 runner 和集成入口已经明确提供。

从新 bag 开始的集成入口：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/run_faster_robust_mapping.py \
  --bag /path/to/test_1301_2026-10-02-16-23-42.bag \
  --calibration pitch25 --rate 0.75 --dynamic-filter
```

默认不开启动态过滤，baseline 入口行为保持原样。过滤要求成功完成的优化后轨迹；失败时保留已有 baseline 文件。`pipeline.json` 明确记录最终地图、未过滤地图和检查范围。原轨迹早晚一致性检查仍针对未过滤的后端时间窗口；动态点子集结构检查在 `dynamic/audit/`，二者不能互相替代。

## 产物与检查

- `filtered_3cm.pcd`：候选保留地图。
- `removed_3cm.pcd`：被移除点，可独立三维检查。
- `point_evidence.npz`：与 baseline 点顺序一致的移除掩码、命中/自由时间段计数和首次/最后观测时间。
- `frame_evidence.csv`：各证据帧的候选、命中、自由票统计。
- `report.json`：输入关联、参数、哈希、耗时和内存。
- `audit/`：同坐标全图与局部三投影图、精确子集检查、平面候选抽样和可选固定表面检查。

新增几何和 CLI 测试覆盖静态表面、移动遮挡、缺测方向、深度边界、薄表面、噪声余量、重复支持保护、正确雷达原点、未知点保留、短暂点过滤、缓存复现和外来轨迹/帧数拒绝。它们是合成测试，不替代四包实际质量验收。

本机另保留每包 baseline / filtered / removed 的独立网页入口。完整 3 cm 地图直接渲染，未使用额外预览降采样；四包已完成 16 项桌面/手机非空画布、像素变化、单图加载和页面异常检查，另检查桌面顶/正/侧视及剖切。网页源与本机数据目录属于现有实验工作区；公开源码提供 Open3D 查看工具。

当前候选不能作为已验证的实时动态检测模块或零误删产品保证。Goal 仍保持 active，后续需要逐包确认动态残留和有代表性的静态细节，再决定参数和后续实现。
