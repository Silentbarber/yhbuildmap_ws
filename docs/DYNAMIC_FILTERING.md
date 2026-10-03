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

## 角度包围检查

2026-10-04，新增对真实扫描的定向审计：每包均匀抽取 16 帧，检查全部第一版移除候选，以及固定随机种子的最多 10000 个保留点对照。统计单位为帧与点的组合，不是独立物体或最终被误删点。

| 包时间 | 移除候选的自由空间组合 | 射线方向未包围目标的组合 | 比例 |
| --- | ---: | ---: | ---: |
| 10-01 21:52:47 | 22946 | 16294 | 71.01% |
| 10-02 16:21:02 | 11251 | 8183 | 72.73% |
| 10-02 16:23:42 | 8386 | 5970 | 71.19% |
| 10-02 16:27:44 | 13914 | 10339 | 74.31% |

三条邻近射线可能都从目标的一侧经过，因此仅处于角度锥内不足以证明目标方向得到覆盖。以上统计证明这种角度外插在实际数据中很常见，但不证明这些组合全部错误，也不能据此推算真实静态误删率。

新增 `--require-angular-support` 可选检查：在目标方向的切平面投影中，三条射线形成的三角形必须包围目标方向；退化共线邻域只有与实际测量方向重合时才支持。端点命中规则、轨迹、时间段、删除阈值和地图坐标保持相同。此检查仍是局部插值近似，无法证明射线之间不存在薄物体。

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/run_dynamic_filtering.py \
  --data-root /path/to/existing/buildmap_test_ws \
  --require-angular-support --output-root "$PWD/results/dynamic_angular_support"
```

开启这一检查后必须重新采集证据，不能复用第一版未检查角度包围的计数。缓存检查会拒绝混用。第一版过滤默认与复现入口继续保留，角度保护候选单独生成并待场景确认。

四包已经重新计算全部采样帧的证据，参数和采样帧与第一版一致，只增加角度包围检查：

| 包时间 | 保留点 | 移除点 | 移除比例 | 相对第一版恢复点 | 证据计算耗时 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 10-01 21:52:47 | 575002 | 14412 | 2.445% | 10812 | 322.0 s |
| 10-02 16:21:02 | 511128 | 6393 | 1.235% | 9642 | 176.2 s |
| 10-02 16:23:42 | 637932 | 7467 | 1.157% | 7454 | 366.7 s |
| 10-02 16:27:44 | 497206 | 6667 | 1.323% | 5015 | 122.0 s |

耗时来自四个包并行运行的每进程墙钟时间，不能用于证明相对第一版串行性能提高。进程峰值 RSS 约 488–525 MiB。不以删除减少自动推断质量提高：角度保护可能恢复误判的静态结构，也可能留下更多真实动态点。

四包精确子集分区均通过，端点命中时间段计数与第一版完全一致，自由空间时间段计数逐点不增加，第二组被移除点严格包含在第一组被移除点内。新候选的局部 PCA 平面候选抽样移除量为 47/30719、3/31856、12/24538、22/13328，仍不能作为语义静态误删率。第三包 16 个原本完全保留的固定表面区域仍完整保留；前述 78 点短暂可见区域恢复 3 点、移除 75 点，其真实类别仍待确认。

真实扫描审计可复现：

```bash
/usr/bin/python3 scripts/audit_angular_visibility.py \
  --data-root /path/to/existing/buildmap_test_ws \
  --delivery-root /path/to/first_candidate_delivery \
  --frames 16 --output "$PWD/results/angular_visibility_audit.json"
```

该审计仅读取第一版候选与实际扫描，输出统计，不修改地图。合成测试检查包围、单侧、共线、直接射线、顺序反转、端点支持不变，以及混用缓存拒绝。

## 时间区域审计

为了区分“地图点被判为时间不一致”与“实际扫描中只在短时间出现”，新增 `scripts/audit_dynamic_regions.py`。它在四个包的固定优化坐标中自动选择三个移除候选区域、持续支持的几何候选，并对第三包的 `reference_11_plane_1_patch_14` 单独保留一个待确认区域。审计读取全部捕获输出帧，使用 Livox 外参得到每帧雷达原点，并将局部扫描点按 5 cm 最近邻关联到 baseline 点。

每个区域输出 `frames.csv`、`frames.json`、`comparison.png`、`time_profile.png` 和用于复核的源索引二进制文件。`frames.csv` 包含源 chunk、chunk 内帧号、优化时间、保留/移除/第一版恢复/未关联计数、局部平面 p95 残差；`frames.json` 还包含源扫描点数和雷达原点。未关联并不代表动态，局部最近邻也不是精确体素 lineage 或语义标签。

四包均已完成该审计，结果位于现有实验根目录的 `results/dynamic_filtering/temporal_review/<id>/`。已观察到的候选区域摘要如下：

| 包 | 自动候选区域 | 每个区域的移除点 | 结论边界 |
| --- | --- | ---: | --- |
| 10-01 21:52:47 | 3 个候选 | 2,935、1,767、7,692 | 计数在时间上间歇出现，仍需查看实际物体 |
| 10-02 16:21:02 | 3 个候选 | 742、2,295、3,254 | 计数在时间上间歇出现，仍需查看实际物体 |
| 10-02 16:23:42 | 3 个候选 + patch 14 | 728、1,119、3,738；patch 14 为 1,402 | patch 14 集中在约 222.5–226.8 s，仍未确认语义类别 |
| 10-02 16:27:44 | 3 个候选 | 1,047、2,061、2,975 | 计数在时间上间歇出现，仍需查看实际物体 |

这些区域是复核入口和时间证据，不是动态物体真值。固定几何控制区域的移除量很低或为零，但局部 PCA/平面候选不等于人工确认的墙面、门框或柱子。复现全部四包审计：

```bash
for id in 215247 162102 162342 162744; do
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/audit_dynamic_regions.py \
    --data-root /path/to/existing/buildmap_test_ws \
    --candidate-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/angular_guard \
    --first-candidate-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/delivery \
    --output-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/temporal_review \
    --id "$id"
done
```

审计不会修改四张交付 PCD，也不会为了改善截图删除局部区域。它的目的是把用户肉眼检查定位到具体时间段和源帧。

## 阈值敏感性

`scripts/sweep_dynamic_filter_params.py` 使用已经采集的角度保护证据，只重新计算删除掩码，不重新播放 bag、不写 PCD。它同时统计被至少四个时间段命中的点是否被删除，以及第三包已有固定表面区域的删除数量。

本轮比较了 `min_free_bins={4,6,8}`、`free_ratio={0.8,0.9}`、`min_span={6,10}s` 和 `max_hit_bins={1,3}` 的 24 组组合。相对于当前 `max_hit_bins=3, min_free_bins=4, free_ratio=0.8, min_span=6s`：

| 包 | 当前移除 | `max_hit_bins=1` 移除 | 恢复点 | `patch_14` 删除 |
| --- | ---: | ---: | ---: | ---: |
| 10-01 21:52:47 | 14,412 | 11,574 | 2,838 | 不适用 |
| 10-02 16:21:02 | 6,393 | 6,108 | 285 | 不适用 |
| 10-02 16:23:42 | 7,467 | 7,130 | 337 | 65（当前为 75） |
| 10-02 16:27:44 | 6,667 | 6,549 | 118 | 不适用 |

这里的“恢复点”是重新分类后不再删除的 baseline 点，不代表它们都是静态点。`max_hit_bins=1` 只是更保守的参数对照，仍可能保留动态物体；当前两组 PCD 和证据均保留，待用户在同坐标三维页面中比较。

复现敏感性分析：

```bash
/usr/bin/python3 scripts/sweep_dynamic_filter_params.py \
  --data-root /path/to/existing/buildmap_test_ws \
  --evidence-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/angular_guard \
  --min-free 4 6 8 --ratio .8 .9 --span 6 10 --max-hit 1 3 \
  --output results/dynamic_filtering/threshold_sweep.json
```

进一步补齐的 `threshold_sweep_v2.json` 核对了地图和轨迹哈希、证据帧数、角度保护设置及数组尺寸，并复用了结构审计的同一套局部 PCA 抽样。收紧 `max_hit_bins` 从 3 到 1 后，四包几何平面候选删除量分别为 47→47、3→3、12→11、22→22。这个差异没有提供明确的整体静态质量优势，当前没有因此替换默认候选。24 组参数与删除统计不是动态检出率对照。

## 三维时间轴

仓库 `viewer/temporal.html` 是局部时间审计的独立三维工具，使用本地 Three.js、OrbitControls 和 Lucide 文件，不依赖外部 CDN 加载。场景显示来自 `audit_dynamic_regions.py` 的实际前端输出扫描，应用相同优化轨迹修正，不额外配准。绿色/红色/黄色对应角度保护 `max_hit_bins=3` 候选的保留/移除/相对第一版恢复状态，不是语义运动真值。

它支持时间轴、播放、前后样本帧、固定相机的 Scan/Baseline/Filtered/Removed 切换、背景上下文以及顶/正/侧视。页面使用约 0.5 s 的显示采样，精确时刻以当前源帧时间为准；`frames.csv` 仍覆盖所有捕获输出帧。局部区域是检查视图，不改变交付地图的范围。

先生成全部四包时间审计数据，再从仓库根目录启动静态服务器：

```bash
/usr/bin/python3 -m http.server 8766 --bind 127.0.0.1 --directory .
```

如果审计生成在本仓库的 `results/dynamic_filtering/temporal_review/`，打开 `http://127.0.0.1:8766/viewer/temporal.html?bag=162342&region=uncertain_patch_14&t=225.4`。审计数据在其他被同一服务器提供的目录时，可用 `data` 参数指定其 URL 根目录，以 `/` 结尾。网页不读取 bag 或 NPZ，它只读取已生成的审计产物。

独立源帧核对工具 `scripts/verify_temporal_region_sources.py` 检查所有缓存档案与元数据哈希，并对每个区域抽取最多四个非空显示帧，从原始缓存索引重建坐标和强度，重新核对最近邻 baseline 索引、距离、分类及雷达原点。抽样核对不等于所有点的精确体素 lineage 或语义标注。

```bash
/usr/bin/python3 scripts/verify_temporal_region_sources.py \
  --data-root /path/to/existing/buildmap_test_ws \
  --review-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/temporal_review \
  --output results/dynamic_filtering/source_validation.json
```

本机源帧检查已验证 21 个区域共 84 个非空显示样本，重建坐标和强度逐值一致，关联索引、距离、分类及雷达原点均通过，同时核对全部缓存档案哈希。时间查看器完成 21 个桌面区域及 5 个手机区域的 26 项检查，包含真实画布像素、旋转、前后样本和播放推进，以及局部 baseline/filtered/removed 点数核对。渲染检查本身不是语义质量证明。

独立发布仓库中的查看器还完成了第三包待确认区域的桌面、手机加载检查，使用本机已生成的同一份审计数据。`bash scripts/test_release.sh` 通过 37 项 Python 测试及 100 轮原生 MID360 输入回归；这些测试不包含新增整包回放或人工动态标签验收。

## 新录包入口

从新 bag 回放到后端及角度保护过滤的 ROS1 入口：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/run_faster_robust_mapping.py \
  --bag /path/to/new_recording.bag --calibration pitch25 --rate .75 \
  --dynamic-filter --dynamic-angular-support \
  --output-dir "$PWD/results/new_dynamic_run"
```

`--calibration` 仍需匹配实际录制配置。`--dynamic-max-hit-bins 1` 可追加为保守对照；默认是 3。未启用 `--dynamic-filter` 时保持 baseline 行为。动态专用参数在没有过滤开关时被拒绝，且入口禁止要求删除被四个及以上时间段支持的点。CLI 集成测试验证参数传递和输出路径；本轮四包过滤仍复用已有完整前端缓存与固定优化轨迹，没有再次执行整包 ROS 回放。
