# 动态过滤研究与候选

日期：2026-10-03。状态：四包运行及初步几何检查完成，场景验收进行中。baseline 为 `9a11f2e`；开发分支为 `feature/dynamic-object-filtering`。

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
  --allow-angular-extrapolation --output-root "$PWD/results/dynamic_delivery"
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
