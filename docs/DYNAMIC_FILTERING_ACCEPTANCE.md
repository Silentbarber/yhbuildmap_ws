# Dynamic Filtering Scene Review

日期：2026-10-04。分支：`feature/dynamic-object-filtering`。待验收实现包含提交 `b5939c4`；baseline 固定为 `handheld-mid360-2026-10-03` / `9a11f2e`。当前候选为四包 `angular_guard`，点时间实验没有替换它们。

## 已完成与剩余项

| Goal 要求 | 当前证据 | 状态 |
| --- | --- | --- |
| 保留 baseline，独立分支研究 | 固定 baseline tag/commit、地图与轨迹哈希；独立开发分支 | 已完成 |
| 方法研究与 ROS1 实现 | Removert/ERASOR/Dynablox 资料筛选；自定义可见性过滤、ROS1 参数入口及复现命令 | 已完成 |
| 四包全部输出帧覆盖 | 四包 2253 / 1261 / 2387 / 879 帧关联检查，共 6780 帧；完整帧缓存未裁剪 | 已完成 |
| 四张过滤 PCD 与移除点 | 575002 / 511128 / 637932 / 497206 个保留点；精确点子集分区 | 已完成 |
| 单图三维查看与时间复核 | 四包整图、21 个局部区域、约 10 Hz 源窗口、桌面/手机检查 | 已完成 |
| 几何一致性、运行与存储成本 | 固定平面与局部 PCA、参数对照、射线间距、原点对照及资源报告 | 已完成，指标不是语义精度 |
| 实际动态残留及静态误删 | 当前是时间不一致候选与几何样本；没有逐对象人工标签 | 待场景确认 |
| 固定正式动态过滤版本 | baseline 已验收，动态过滤候选尚未得到场景反馈 | 待场景确认 |

四包已有完整扫描缓存，因此其他三个原始 bag 当前缺失，不影响已交付的角度保护候选。只有第一包的新点时间实验完成了原 bag 重新回放；其他三包不能声称完成逐点时间原点实验。该追加实验不是四包默认候选的前提。

## 四张整图

| 包时间 | 当前过滤图 | 同坐标 baseline | 全部移除点 |
| --- | --- | --- | --- |
| 10-01 21:52:47 | [Filtered](http://127.0.0.1:8765/viewer/?map=dynamicGuardedFiltered215247) | [Baseline](http://127.0.0.1:8765/viewer/?map=dynamicBaseline215247) | [Removed](http://127.0.0.1:8765/viewer/?map=dynamicGuardedRemoved215247) |
| 10-02 16:21:02 | [Filtered](http://127.0.0.1:8765/viewer/?map=dynamicGuardedFiltered162102) | [Baseline](http://127.0.0.1:8765/viewer/?map=dynamicBaseline162102) | [Removed](http://127.0.0.1:8765/viewer/?map=dynamicGuardedRemoved162102) |
| 10-02 16:23:42 | [Filtered](http://127.0.0.1:8765/viewer/?map=dynamicGuardedFiltered162342) | [Baseline](http://127.0.0.1:8765/viewer/?map=dynamicBaseline162342) | [Removed](http://127.0.0.1:8765/viewer/?map=dynamicGuardedRemoved162342) |
| 10-02 16:27:44 | [Filtered](http://127.0.0.1:8765/viewer/?map=dynamicGuardedFiltered162744) | [Baseline](http://127.0.0.1:8765/viewer/?map=dynamicBaseline162744) | [Removed](http://127.0.0.1:8765/viewer/?map=dynamicGuardedRemoved162744) |

## 优先确认区域

以下时间以首个估计器输出帧为零点。每个入口读取已有完整帧窗口，源头是 Faster-LIO 实际输出扫描，应用同一优化轨迹修正；颜色表示地图候选分类，不是动态/静态真值。局部区域中的移除数量包括区域内所有点，不能直接称为一个物体的点数。

| 包 | 区域与源窗口 | 区域 baseline / 移除点 | 三维时间入口 |
| --- | --- | ---: | --- |
| 215247 | `removal_00`，75.3–83.1 s | 29448 / 7692 | [79.2 s](http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=215247&region=removal_00&t=79.2) |
| 162102 | `removal_00`，13.1–20.9 s | 26730 / 3254 | [17.0 s](http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=162102&region=removal_00&t=17.0) |
| 162342 | `removal_00`，221.3–229.2 s | 25534 / 3738 | [225.4 s](http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=162342&region=removal_00&t=225.4) |
| 162744 | `removal_00`，57.7–65.6 s | 34880 / 2975 | [61.6 s](http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=162744&region=removal_00&t=61.6) |
| 162342 | `uncertain_patch_14`，221.3–229.2 s | 5409 / 1402 | [225.4 s](http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=162342&region=uncertain_patch_14&t=225.4) |

最后一行是包围面片的整个区域，其中单独固定的 `reference_11_plane_1_patch_14` 几何面片为 78 点、移除 75 点；区域 1402 点与面片 75 点是不同统计单位。该面片短暂出现，但平面形状和短暂可见都不足以确认它属于动态物体。

## 静态结构对照

这些是几何控制区域，仍需结合实物确认墙面、地面、门框等类别；区域内可以混有其他物体，不能把区域移除点全数视为静态误删。

| 包 | 优先静态控制 | 三维时间入口 |
| --- | --- | --- |
| 215247 | `planar_vertical`，174.5–182.4 s | [178.4 s](http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=215247&region=planar_vertical&t=178.4) |
| 162102 | `planar_vertical`，42.9–50.8 s | [46.8 s](http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=162102&region=planar_vertical&t=46.8) |
| 162342 | `planar_vertical`，120.5–128.4 s | [124.4 s](http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=162342&region=planar_vertical&t=124.4) |
| 162744 | `planar_vertical`，13.7–21.6 s | [17.6 s](http://127.0.0.1:8765/viewer/temporal.html?data=/results/dynamic_filtering/motion_review/&bag=162744&region=planar_vertical&t=17.6) |

## 反馈与版本选择

反馈记录需要包时间、区域或位置、实际物体，以及过滤前后观察：动态物体残留是否减少，固定结构是否出现缺口。无法判断的区域保留“未知”，不根据点数或颜色自动赋予动静态标签。可采用格式：`162342 / uncertain_patch_14 / 实际物体：... / 过滤后：...`。

目前没有把任何用户场景判断写入结果。原点插值在相同分组对照中没有显示明确的几何平面保留优势，故不自动提升为选定版本；更多实验不会替代这项场景证据。收到反馈后再据具体失败区域调整或选定版本。全部实现与实测记录见 [DYNAMIC_FILTERING.md](DYNAMIC_FILTERING.md)。
