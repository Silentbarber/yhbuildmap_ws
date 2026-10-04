# 动态过滤研究与候选

日期：2026-10-03，交付更新：2026-10-04。状态：研究实现与四包候选交付完成。baseline 为 `9a11f2e`；开发分支为 `feature/dynamic-object-filtering`。当前选定阶段候选是 `angular_guard`；用户反馈观察到改善，局部实际物体类别仍未知。参数、四张 PCD 哈希与资源统计固定在 `config/dynamic_filtering_delivery.json`，场景复核入口见 `DYNAMIC_FILTERING_ACCEPTANCE.md`。

2026-10-04 更新：常用批处理入口及 ROS1 建图入口在开启动态过滤时，已默认使用角度保护候选。下面先保留第一版的历史统计和显式复现方法；当前角度保护结果见后续章节。未开启动态过滤时仍使用 baseline。

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

## 第一版对照

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

脚本先核对 `config/dynamic_filtering_recordings.json` 中四张地图与轨迹哈希，再逐包使用相同参数过滤和审计。`--id 162342` 可只处理第三包。已有同参数证据时，可使用 `--reuse-root`，支持当前 `<reuse-root>/<id>/` 和旧 `<reuse-root>/<id>/visibility_v2/` 布局；复用时检查来源、地图与轨迹哈希、帧数和采样设置。输出目录需尚不存在。省略旧版外插参数时，默认生成角度保护候选，不能混用第一版缓存。

第一版对照的单目录复现：

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

默认不开启动态过滤，baseline 入口行为保持原样。上面的 `--dynamic-filter` 现在自动开启角度保护；额外传 `--dynamic-allow-angular-extrapolation` 才复现第一版旧模型。过滤要求成功完成的优化后轨迹；失败时保留已有 baseline 文件。`pipeline.json` 明确记录最终地图、未过滤地图和检查范围。原轨迹早晚一致性检查仍针对未过滤的后端时间窗口；动态点子集结构检查在 `dynamic/audit/`，二者不能互相替代。

## 产物与检查

- `filtered_3cm.pcd`：候选保留地图。
- `removed_3cm.pcd`：被移除点，可独立三维检查。
- `point_evidence.npz`：与 baseline 点顺序一致的移除掩码、命中/自由时间段计数和首次/最后观测时间。
- `frame_evidence.csv`：各证据帧的候选、命中、自由票统计。
- `report.json`：输入关联、参数、哈希、耗时和内存。
- `audit/`：同坐标全图与局部三投影图、精确子集检查、平面候选抽样和可选固定表面检查。

新增几何和 CLI 测试覆盖静态表面、移动遮挡、缺测方向、深度边界、薄表面、噪声余量、重复支持保护、正确雷达原点、未知点保留、短暂点过滤、缓存复现和外来轨迹/帧数拒绝。它们是合成测试，不替代四包实际质量验收。

本机另保留每包 baseline / filtered / removed 的独立网页入口。完整 3 cm 地图直接渲染，未使用额外预览降采样；四包已完成 16 项桌面/手机非空画布、像素变化、单图加载和页面异常检查，另检查桌面顶/正/侧视及剖切。网页源与本机数据目录属于现有实验工作区；公开源码提供 Open3D 查看工具。

当前候选是离线地图清理实现，不能作为已验证的实时动态检测模块或零误删产品保证。研究与候选交付完成；逐对象人工标签不是交付前提。未来若发现具体结构缺口或动态残留，可据失败区域另行改进，不把未测量的语义质量写成已验证。

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

开启这一检查后必须重新采集证据，不能复用第一版未检查角度包围的计数。缓存检查会拒绝混用。常用入口已默认开启该检查；第一版仅通过显式旧版参数或低层脚本复现，历史结果仍保留。角度保护候选仍待场景确认。

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
  --dynamic-filter \
  --output-dir "$PWD/results/new_dynamic_run"
```

`--calibration` 仍需匹配实际录制配置。`--dynamic-max-hit-bins 1` 可追加为保守对照；默认是 3。角度保护已默认开启，原 `--dynamic-angular-support` 仍可显式传入。`--dynamic-allow-angular-extrapolation` 仅用于旧版对照，并与保护开关互斥。未启用 `--dynamic-filter` 时保持 baseline 行为。动态专用参数在没有过滤开关时被拒绝，且入口禁止要求删除被四个及以上时间段支持的点。CLI 集成测试验证参数传递和输出路径；本轮四包过滤仍复用已有完整前端缓存与固定优化轨迹，没有再次执行整包 ROS 回放。

## 完整帧运动复核

`scripts/audit_dynamic_motion.py` 比较选定窗口内相邻扫描的固定世界模型和 Open3D 刚体 ICP 模型。拟合只使用训练空间块，检查未参与拟合的点、双向重叠、反向配准闭合、点到平面信息和整体非平面形状。中位数残差也必须改善，以降低可见范围缩小或 MID360 扫描条纹变化带来的误判。平面内运动和形变可能无法确认；该工具只审计，不修改 PCD、轨迹或过滤掩码。

默认读取原时间审计的显示样本。`--full-source` 从原捕获缓存重建选定峰值前后窗口内的全部输出帧，约 10 Hz，不保存整套扫描的副本。校验原缓存、参数、地图和轨迹哈希；雷达原点包含固定外参。它另外使用实际位姿直接计算随设备运动的刚体模型，无需 ICP 拟合。与该模型相容只提示可能存在随设备移动的回波，不能直接确认是手、身体或衣物。

```bash
for recording_id in 215247 162102 162342 162744; do
  OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/audit_dynamic_motion.py \
    --review-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/temporal_review \
    --data-root /path/to/existing/buildmap_test_ws --full-source \
    --output-root results/dynamic_filtering/motion_audit_full \
    --export-review-root results/dynamic_filtering/motion_review --id "$recording_id" || exit 1
done
```

输出位置必须是新目录。`--export-review-root` 为现有 `viewer/temporal.html` 导出完整窗口数据；省略该参数只生成数值报告和固定坐标对照图。导出保留实际 chunk/源帧/点索引、最近 baseline 点索引、距离与分类。这里是选定局部窗口的全部帧，约 8 秒，不是四包全程运动检测；完整全程时间计数仍在原 `temporal_review` 的 CSV。

本机最新运动报告在 `results/dynamic_filtering/motion_audit_full_v3/`，对应完整帧三维数据在 `motion_review/`：

| 包 | 区域帧数 | 刚体运动候选帧对 | 随设备运动相容帧对 | 几何控制中的对应帧对 |
| --- | ---: | ---: | ---: | ---: |
| 10-01 21:52:47 | 399 | 3 | 0 | 0 / 0 |
| 10-02 16:21:02 | 398 | 2 | 0 | 0 / 0 |
| 10-02 16:23:42 | 480 | 1 | 2 | 0 / 0 |
| 10-02 16:27:44 | 398 | 4 | 14 | 0 / 0 |

区域可能重叠，第二包两处候选包含同一时间帧对；这些不是独立物体数量、动态精确率或召回率。控制区域是几何候选，表中零候选也不是全图静态零误删证明。多数帧对的几何不足或仍无法确认，报告明确保留这些状态，不能将它们强行归类为静态或动态。

第三包 `removal_00` 在约 227.60–227.70 s 有 8.38 cm 的拟合位移，雷达原点位移约 0.85 mm；未参与拟合点的中位数距离从约 4.05 cm 降到 1.76 cm。这加强了该区域存在实际运动的几何证据，但它不等于原 `uncertain_patch_14` 的逐对象类别确认，也不能排除尚未建模的误差。随设备运动相容点对主要出现在第三包约 226.8–227.0 s 和第四包约 59–61 s、71.1–71.2 s，可优先查看。

独立重建检查全部非空导出帧：

```bash
/usr/bin/python3 scripts/verify_temporal_region_sources.py \
  --data-root /path/to/existing/buildmap_test_ws \
  --review-root results/dynamic_filtering/motion_review \
  --window-review --all-display-frames \
  --output results/dynamic_filtering/motion_review/source_validation.json
```

本机 21 个窗口共 1,675 个区域帧，其中 1,563 个非空帧的 3,277,635 个 XYZI 点、源索引、关联索引、距离和分类核对通过；去重后覆盖 1,286 个捕获帧。完整帧查看器另完成 26 项桌面/手机像素、旋转、切换和播放检查。局部数据约占 99 MiB；最新每包运动审计约 11–16 s、峰值内存约 497–532 MiB，均不含首次过滤耗时。

查看完整窗口时，使用 `http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=162342&region=uncertain_patch_14&t=225.4`，端口和 `data` URL 按实际静态服务器目录调整。页面每次仍只显示一个包、一个区域。

本轮保持四包 baseline 与两套过滤候选 PCD 哈希一致；运动检查尚未反馈到删除策略。后续需结合实际场景确认残留和静态误删，再决定是否使用随设备运动证据补充过滤。

`bash scripts/test_release.sh` 已通过 50 项 Python 测试及 100 轮原生 MID360 输入回归。新增测试包含静止物体可见范围变化、带噪平面扫描、实际刚体位移、设备运动模型、源坐标/分类/原点、窗口选择、缓存篡改及导出索引核对。没有新增整包 ROS 回放。

## 设备随动补充试验

`scripts/filter_sensor_motion.py` 是未合入默认链路的研究工具。它在现有角度保护地图上尝试补充删除：原位置的固定世界匹配距离至少 10 cm，实际设备运动预测的位置有 4 cm 内的测量，位置变化至少 10 cm，且实测射线在原位置后方至少 10 cm、具有方向包围支持。遮挡和空方向不计为空位。这里只研究离原雷达位置 0.35–1.5 m、最多一个命中时间段的保留点，不裁剪交付地图。

检查全部捕获输出帧的位姿关联，按至少 0.3 s 选择扫描；使用约 0.6 / 1.2 / 1.8 s 间隔的源扫描作设备随动对照。要求至少三个不同 0.5 s 时间段投票、跨度至少 1 s，这些时间段不意味着统计上完全独立。长期支持点受保护；工具也会拒绝已删除 `hit_bins >= 4` 点的父地图。多帧计数不等于语义动态标签。

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/filter_sensor_motion.py \
  /path/to/frontend /path/to/coverage_geometry_v5 \
  --visibility-dir /path/to/angular_guard/162744 \
  --output-dir "$PWD/results/sensor_motion_trial"
```

四包完整缓存试验在本机 `results/dynamic_filtering/sensor_motion_v1/<id>/`：

| 包 | 全部源帧 / 选中帧 | 累计随动匹配 | 射线确认的累计匹配 | 有投票点数 | 新增删除 | 耗时 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 10-01 21:52:47 | 2253 / 633 | 2227 | 5 | 5 | 0 | 20.7 s |
| 10-02 16:21:02 | 1261 / 355 | 3990 | 14 | 14 | 0 | 16.3 s |
| 10-02 16:23:42 | 2387 / 670 | 2545 | 3 | 3 | 0 | 25.3 s |
| 10-02 16:27:44 | 879 / 247 | 2717 | 4 | 4 | 0 | 10.1 s |

累计匹配会重复统计点与源/目标对，不能称为物体数或检出率。所有得到射线确认的点都只有一个投票时间段，没有持续证据。因此四张输出的保留/移除 PCD 都与原角度保护候选逐字节相同；没有通过放宽门槛制造新增删除。峰值 RSS 约 453–505 MiB，试验产物约 44.4 MiB，没有重复录制或保存整包扫描。这个结果只说明该补充规则在当前设置下未改善四包，不代表已经没有动态残留。

## 默认入口检查

ROS1 入口现在只需 `--dynamic-filter` 即启用角度保护。四包批处理也默认启用；要回放历史第一版，使用 `--allow-angular-extrapolation`。原显式保护参数继续兼容，低层 `filter_dynamic_map.py` 的实验默认保持原样。

本机 `results/dynamic_filtering/default_entry_validation.json` 记录了真实四包批处理默认参数的缓存复现：四张保留/移除 PCD SHA256 均与当前角度保护候选一致，精确分区审计通过。临时输出已清理，原地图和缓存不变。这里验证了批处理实际运行和 ROS1 参数传递；没有新增四包整段 ROS 回放，也没有完成逐对象人工验收。

本轮 `bash scripts/test_release.sh` 通过 65 项 Python 测试及 100 轮原生 MID360 输入回归，全部命令行帮助入口通过。新增测试覆盖设备随动与实际空位联合证据、遮挡与缺失方向、重复命中保护、外来轨迹拒绝，以及批处理和 ROS1 默认参数传递。加强父地图掩码检查的测试样本后，设备随动模块的 11 项测试再次通过。这些检查证明实现和数据关联符合已定义规则，不能代替真实场景动态物体与静态结构验收。

## 支撑射线的物理间距

角度包围仍是一种插值：三个远处回波可以围住近处细杆的方向，却都没有打到细杆。新增 `scripts/audit_ray_footprint.py` 直接计算候选点到三条实测支撑射线的垂直距离，并取其中最大值作为支撑半径。测试中的 10 m 处 2 cm 半径静态细杆被三条约 5.8 cm 外的射线绕过，原角度规则仍给出自由空间证据；这是受控的风险样例，不是四包中已确认的误删实例。

四包审计读取全部捕获输出帧并核对同时间戳位姿，复现原 1,905 个证据帧。覆盖所有 34,939 个删除候选及每包 10,000 个固定随机种子的保留控制点。源缓存、元数据、地图、轨迹和父掩码哈希均校验；被审计点的原始 `hit_bins`、`free_bins`、首次/末次自由证据和删除分类逐值复现，再分别限制三条射线的支撑半径。原保留点不会因收紧自由证据而新增删除。

| 包 | 原删除候选 | 2 cm 限制后仍支持删除 | 3 cm 限制后仍支持删除 | 5 cm 限制后仍支持删除 | 5 cm 恢复候选 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 10-01 21:52:47 | 14412 | 5749 | 11271 | 14136 | 276 |
| 10-02 16:21:02 | 6393 | 4082 | 5521 | 6351 | 42 |
| 10-02 16:23:42 | 7467 | 4874 | 6054 | 7198 | 269 |
| 10-02 16:27:44 | 6667 | 2173 | 4216 | 6119 | 548 |

这里的恢复是诊断掩码中的变化，没有写出新的 PCD。限制不是实际激光束宽，也不包括位姿不确定度；没有语义静态或动态标签。实际支撑半径的最大值约为 7.4 / 7.6 / 10.1 / 9.5 cm，说明固定角度阈值对应的物理间距确实会变化。

`--structure-only` 复用完整审计数组，对照原结构审计使用的同一套局部 PCA 抽样和固定参考面片，并重新校验源哈希、整个原删除集合的覆盖及诊断分类。5 cm 限制后，四包平面样本的删除量是 47→47、3→3、12→12、22→20；第三包待确认的 78 点参考区域仍删除 75 点。2 cm 限制虽分别恢复 16 / 0 / 10 / 14 个几何平面样本，却会恢复大量尚未确认类别的点。不能据此宣称静态结构质量提高或动态过滤更好，当前不把间距限制合入默认链路。

本机产物位于 `results/dynamic_filtering/ray_footprint_v1/<id>/`。`point_audit.npz` 保存原 baseline 索引、各间距的时间段证据与诊断掩码；`report.json` 保存原证据复现及间距统计；`structural_report.json` 保存固定几何对照。四包证据审计约 28.8 / 15.0 / 24.5 / 10.5 s，串行同一进程累计峰值 RSS 不超过 479 MiB。产物约 1.3 MiB，未复制整包扫描或地图；四张 baseline、候选 PCD 和轨迹保持原哈希。耗时不包含另行执行的结构比较，也不包含首次前端回放。

复现完整审计，然后复用其结果比较结构：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/audit_ray_footprint.py \
  --data-root /path/to/existing/buildmap_test_ws \
  --evidence-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/angular_guard \
  --review-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/temporal_review \
  --output-root "$PWD/results/ray_footprint_trial"

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/audit_ray_footprint.py \
  --data-root /path/to/existing/buildmap_test_ws \
  --evidence-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/angular_guard \
  --review-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/temporal_review \
  --output-root "$PWD/results/ray_footprint_trial" --structure-only
```

输出目录必须是首次审计的新位置；结构比较要求已有完整间距审计，且不能覆盖已有结构报告。最新 `bash scripts/test_release.sh` 通过 77 项 Python 测试、100 轮原生 MID360 输入回归和全部帮助入口。新增 12 项测试覆盖细杆绕射线风险样例、物理范围缩放、最远射线约束、坐标变换不变性、缺测、完整缓存重放、源篡改拒绝和结构报告分类核对；仍未进行语义场景验收或新增整包 ROS 回放。

### 帧末射线原点近似

进一步检查前端代码 `LaserMapping::PublishFrameWorld()`：四包都设置 `dense_publish_en=true`，发布 `scan_undistort_` 经帧末位姿转换后的世界点，消息时间戳是 `lidar_end_time_`。当前缓存只有 XYZI 及帧时间，因此过滤用所有点共有的帧末雷达位置作射线原点；它不能恢复每个点实际发射时刻的原点。去畸变后的物体坐标与实测回波有关，但从共同原点连接的射线仍是近似。

`--origin-motion-only` 对已完成间距审计的四包轨迹做独立诊断。检查地图、轨迹和外参配置哈希，含雷达相对 IMU 的平移杆臂；仅统计相邻间隔 75–125 ms 的优化帧末位姿：

| 包 | 约 0.1 s 相邻位姿对 | 原点位移中位数 | 95 分位 | 最大值 |
| --- | ---: | ---: | ---: | ---: |
| 10-01 21:52:47 | 2246 | 2.25 cm | 6.14 cm | 10.28 cm |
| 10-02 16:21:02 | 1260 | 2.73 cm | 4.74 cm | 5.88 cm |
| 10-02 16:23:42 | 2381 | 2.03 cm | 4.25 cm | 6.35 cm |
| 10-02 16:27:44 | 878 | 6.71 cm | 11.45 cm | 14.43 cm |

这些是相邻帧末原点的位移，既不是实际逐点原点误差，也不是扫描内部运动的真值或误删率。第四包的运动尺度说明，只收紧 2–5 cm 间距并不能修正共有原点的近似。下一步应研究从 bag 的点时间与完整 IMU 运动恢复逐点射线原点，并与固定 baseline 保持坐标一致；不能根据这个表断定所有当前删除都错误。缓存不足以直接做这项修正，需先验证 raw 点时间、去畸变与外参的关联，再决定局部重建或增加专用捕获格式。

原始第一包进一步审计确认 `/livox/lidar` 的 `timestamp` 字段是 `FLOAT64` 绝对纳秒时间戳：2,261 帧、45,206,016 个点，时间戳全部有效；相对消息头的点时间范围为 0–105.208 ms，扫描持续时间中位数为 100.180 ms，消息头间隔约 100 ms。`scripts/audit_livox_point_timing.py` 只读取 bag，不复制或修改数据。该结果使逐点原点修正具备输入依据，但尚未证明估计器发布的世界点仍保留同一时间基准。

ROS1 捕获器现在会从输出 `PointCloud2` 保存 PCL 的 `curvature`（或兼容的 `time`）偏移字段到每个 `temporal_raw/*.npz` 的 `point_time_ms`，并在 `capture.json` 中记录 `point_time_preserved`。原始 Livox 的绝对 `timestamp` 不会被误当作毫秒偏移；它由独立 bag 审计工具处理。旧缓存没有这个字段，不能伪造补齐；动态过滤继续按原有帧末原点运行，直到完成新的逐点时间捕获与轨迹插值验证。

## 逐点雷达原点实验入口

过滤器支持显式的 `--point-time-groups-ms 0.5..20`。它要求每个捕获归档都有与 `lengths` 完全一致的 `point_time_ms`，把点偏移换算为绝对时间，在优化轨迹上做平移线性插值和旋转 SLERP，并将 Livox 外参杆臂转换到世界坐标。短时间组随后按时间顺序合并为最多 4 个连续段，每段用组内中位雷达原点执行现有 `ray_evidence`；组间命中优先于自由空间，少于三条回波的组保持未知。这个上限用于控制 60 万级地图的重复空间查询；它是可审计的近似，不能把 5 ms 分组或中位原点当成逐点真值。

ROS1 Faster-LIO 入口：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/run_faster_robust_mapping.py \
  --bag /path/to/recording.bag --calibration pitch25 --rate .75 \
  --dynamic-filter --dynamic-point-time-groups-ms 5 \
  --output-dir "$PWD/results/point_time_dynamic_run"
```

批处理入口对应 `scripts/run_dynamic_filtering.py --point-time-groups-ms 5`。默认值为 0，关闭该实验；没有完整 `point_time_ms` 的旧缓存会在建立输出目录前拒绝。第一包新捕获的实验结果位于 `results/point_time_capture_v2/215247/point_time_dynamic_v2/`，生成了独立的 `filtered_3cm.pcd`、`removed_3cm.pcd`、`point_evidence.npz` 和 `report.json`，不覆盖 `angular_guard` 候选。其余三包目前仍只有旧缓存，因此不能伪造逐点原点 PCD。合成 CLI、时间插值、组内原点和缺失字段测试已通过；四包最终采用哪一版仍需真实场景复核。

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/audit_ray_footprint.py \
  --data-root /path/to/existing/buildmap_test_ws \
  --evidence-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/angular_guard \
  --review-root /path/to/existing/buildmap_test_ws/results/dynamic_filtering/temporal_review \
  --output-root "$PWD/results/ray_footprint_trial" --origin-motion-only
```

`origin_motion_report.json` 保存上述独立诊断，不改变地图。加入旋转杆臂和长位姿间隔测试后，间距审计模块的 14 项测试再次通过；随后在加载 ROS1/Livox 工作空间的环境中，完整 `bash scripts/test_release.sh` 通过 94 项 Python 测试、100 轮原生 MID360 输入回归和全部帮助入口。

### 第一包真实点时间实验

2026-10-04，修复 `LaserMapping::PointBodyToWorld()` 没有复制 `curvature` 的问题后重新编译并完整回放第一包。新捕获包含 2,253 帧、29,873,222 个输出点，点时间范围约 0.004864–105.207809 ms；输入 2,261 个雷达消息和 45,213 个 IMU 消息全部通过完整性检查。第一次无效捕获的时间值全为零，已清理，未用于地图评估。

新后端地图的 SHA256 与原 baseline 完全相同，2,253 行优化轨迹的数值最大差异为 4.44e-15；轨迹文本哈希不同，因此比较额外检查数值，不混用两套轨迹的证据缓存。该实验只修改射线原点证据，不修改地图点坐标。

| 项目 | 角度保护候选 | 点时间原点候选 |
| --- | ---: | ---: |
| 固定 baseline 点数 | 589414 | 589414 |
| 保留点数 | 575002 | 578924 |
| 移除点数 | 14412 | 10490 |
| 几何平面抽样移除 / 30719 | 47 | 45 |

两版共同移除 10,115 点；新候选恢复 4,297 点、新增移除 375 点。第一帧扫描开始时间早于优化轨迹起点，该帧保持未知，报告记录 `point_time_skipped_frames=1`。本次 633 个采样证据帧包含这一个跳过帧，113 个时间段；过滤耗时约 974.7 s，峰值 RSS 516.1 MiB，输出约 12.3 MB（不包含审计图）。新捕获及后端合计约 667 MiB。它仍是离线实验，删除量及平面抽样变化不能证明语义动态检出率或零误删。

新候选通过精确点子集分区审计；未删除具有至少四个命中时间段的点。第一包没有已有人工确认的静态表面标注，几何平面抽样不是静态真值。其余三包没有带点时间的新缓存，当前四张默认角度保护候选继续保留。

本机独立三维入口：

- 保留地图：`http://127.0.0.1:8765/viewer/?map=dynamicPointTimeFiltered215247`
- 删除点：`http://127.0.0.1:8765/viewer/?map=dynamicPointTimeRemoved215247`

这两个入口完成桌面/手机非空画布、点数、旋转像素变化、无页面异常及单图加载检查，桌面另检查三个正交方向与切面。查看器检查报告在实验根目录 `results/dynamic_filtering/point_time_viewer_validation/report.json`，地图与完整审计在发布仓库的 `results/point_time_capture_v2/215247/point_time_dynamic_v2/`。本机地图和审计产物不随源码上传。

第一包从原 bag 重新运行的命令（生成新输出，不覆盖已有结果）：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/run_faster_robust_mapping.py \
  --bag /path/to/test_1301_2026-10-01-21-52-47.bag \
  --calibration raw --rate 1 --backend-profile observation_geometry \
  --dynamic-filter --dynamic-point-time-groups-ms 5 \
  --output-dir "$PWD/results/point_time_first_bag_rerun"
```

真实实验完成后的 `bash scripts/test_release.sh` 通过 95 项 Python 测试、100 轮原生 MID360 输入回归和全部帮助入口。地图质量仍待实际场景复核，Goal 保持 active。

### 分组与原点修正的对照

把整帧回波分成最多四个时间段，会同时改变邻域回波密度和射线原点。因此第一包从 14,412 点减少到 10,490 点的移除统计，不能全部归因于更准确的原点。低层过滤器新增实验参数 `--point-time-origin-mode frame-end-control`：保留与插值版相同的点时间分组、未知首帧、采样间隔和分类阈值，但每组使用同一个帧末雷达原点。默认模式仍为 `interpolated`；控制参数必须与非零 `--point-time-groups-ms` 一起使用，两种模式的证据缓存不能混用。

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/filter_dynamic_map.py \
  results/point_time_capture_v2/215247 results/point_time_capture_v2/215247/backend \
  --output-dir "$PWD/results/point_time_capture_v2/215247/grouped_frame_end_control" \
  --frame-step .3 --angle-deg .7 --max-hit-bins 3 --require-angular-support \
  --point-time-groups-ms 5 --point-time-origin-mode frame-end-control

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/compare_ray_origin_ablation.py \
  results/point_time_capture_v2/215247/grouped_frame_end_control \
  results/point_time_capture_v2/215247/point_time_dynamic_v2 \
  --output-dir "$PWD/results/point_time_capture_v2/215247/origin_ablation"
```

比较工具在写输出前校验共同地图与轨迹哈希、来源目录、分类阈值、帧选择和每帧时间组数，重算分类并核对两套 PCD 的精确点子集。输出 `changed_points.npz` 的索引对应固定 baseline；非空差异分别导出 `restored_by_origin.pcd` 和 `newly_removed_by_origin.pcd`，不改变已有地图。报告将几何平面抽样、命中/自由时间段变化和原运行耗时分开记录；恢复点与新增移除点都不是语义标签。这项对照不修改四包默认候选。

第一包完整对照已经运行：相同的 2,253 个输入捕获帧、633 个采样证据帧（其中首帧保持未知）和 113 个时间段；两套来源地图、轨迹、参数及采样表一致。

| 项目 | 相同分组 + 帧末原点 | 相同分组 + 插值原点 |
| --- | ---: | ---: |
| 保留点数 | 579000 | 578924 |
| 移除点数 | 10414 | 10490 |
| 几何平面抽样移除 / 30719 | 45 | 45 |
| 实际过滤墙钟时间 | 983.9 s | 974.7 s |

两版共同移除 9,556 点，插值原点恢复 858 点、新增移除 934 点；平面抽样中恢复 1 点、新增移除 1 点。命中时间段计数在 11,519 点上减少、14,937 点上增加；自由时间段计数在 12,620 点上减少、13,350 点上增加。这是射线原点变化的实际影响，并不证明变化后的语义分类更正确。

相对未分组角度保护候选的删除量下降，也出现在保留帧末原点的分组控制中；不能把这个下降解释为原点插值带来的地图质量提升。与未分组旧结果相比，分组、首帧未知处理和新捕获也是变化因素。本轮对照没有提供足够的结构证据替换四包默认候选，不能从两次耗时相近的运行推断性能优势。

本机新增独立三维入口：

- 分组帧末原点控制地图：`http://127.0.0.1:8765/viewer/?map=dynamicOriginControlFiltered215247`
- 仅原点变化恢复的 858 点：`http://127.0.0.1:8765/viewer/?map=dynamicOriginRestored215247`
- 仅原点变化新增移除的 934 点：`http://127.0.0.1:8765/viewer/?map=dynamicOriginNewlyRemoved215247`

控制地图及证据约 12.3 MB；差异 PCD、NPZ 和比较报告的文件内容合计约 38 KB，不复制前端扫描缓存。完整回归通过 101 项 Python 测试、100 轮 MID360 输入回归及全部帮助入口。两个恢复/移除差异文件可直接用仓库的 `scripts/view_pcd.py` 查看。新增入口的 4 项桌面/手机检查通过，覆盖正确单图选择、点数、非空画布、旋转像素变化及页面无异常，桌面控制地图另检查三正交视图和切面。原地图、四张默认过滤候选和轨迹保持不变。
