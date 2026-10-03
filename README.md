# yhbuildmap_ws

面向 Livox MID360 手持建图的 ROS1 Faster-LIO 适配与离线地图优化。这是 2026-10-03 在四个录制包上验证、经用户肉眼验收的版本，默认后端为 `observation_geometry`。

当前 `feature/dynamic-object-filtering` 分支开展动态物体过滤研究。baseline 固定为 `handheld-mid360-2026-10-03` / `9a11f2e`；离线可见性过滤及角度保护候选均完成四包运行，尚待用户场景验收。方法、参数、结果和复现命令见 [动态过滤候选](docs/DYNAMIC_FILTERING.md)，完整目标见 [动态过滤 Goal](docs/DYNAMIC_FILTERING_GOAL.md)。建图入口默认仍运行 baseline；增加 `--dynamic-filter` 现在自动启用射线角度包围保护，原 `--dynamic-angular-support` 仍兼容，`--dynamic-max-hit-bins 1` 可生成保守对照。四包批处理 `run_dynamic_filtering.py` 也默认开启角度保护；旧版外插仅通过明确的实验参数复现。局部三维时间检查工具在 `viewer/temporal.html`。另有设备随动补充研究工具，四包没有得到持续的追加删除证据，未合入默认链路。

`scripts/audit_ray_footprint.py` 可重算全部已删除点及保留控制点的原始采样证据，检查三条实测支撑射线的物理间距，并对照固定几何样本。四包间距审计没有提供足够证据替换当前候选，因此该工具只生成诊断数据，不修改地图。

新的 ROS1 捕获会保留 Livox 点时间；在完成带 `point_time_ms` 的新缓存验证后，可通过 `--dynamic-point-time-groups-ms 5`（或批处理的 `--point-time-groups-ms 5`）试验逐点雷达原点插值。该开关要求完整点时间数组，默认关闭，不改变现有四包候选。

## 算法与处理流程

1. 将录包中的 `/livox/lidar` PointCloud2 转换为 Livox CustomMsg，保留点时间并按偏移时间排序；IMU 使用 `/livox/imu`。
2. ROS1 Faster-LIO 估计轨迹并记录匹配点数、距离、航向可观测性等诊断字段。
3. 离线后端使用 1 s 关键帧、局部扫描几何约束、经双向配准和留出空间块验证的回环，以及观测加权的里程计边进行位姿图优化。
4. 使用全部捕获的估计器输出帧重建 3 cm 体素 PCD，输出轨迹、候选约束、优化报告和一致性检查。

前端实时话题 `/cloud_registered`、`/Odometry` 仍是 Faster-LIO 原始输出；离线后端在录包回放完成后运行，不会实时修正这些话题。观测权重属于启发式估计，没有外部真值时不能把内部一致性指标解释为绝对定位精度。该版本保留完整输出帧，没有删除末段 30 s 或按建筑范围裁剪地图。

## 环境与安装

目标环境为 Ubuntu 20.04、ROS1 Noetic、系统 Python 3.8、支持 C++17 的编译器。发布路径编译验证使用 GCC 11.4；并行算法依赖系统 TBB。先安装 ROS Noetic，再安装依赖：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake python3-pip \
  libeigen3-dev libpcl-dev libyaml-cpp-dev libgoogle-glog-dev libgflags-dev libtbb-dev \
  ros-noetic-pcl-ros ros-noetic-eigen-conversions ros-noetic-tf \
  ros-noetic-message-generation ros-noetic-rosbag ros-noetic-rospy \
  ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-rosgraph-msgs
/usr/bin/python3 -m pip install --user -r requirements.txt
```

Python 依赖已固定到实测版本；ROS Python 包由 apt 提供。请使用 `/usr/bin/python3`，避免 Conda 的 Python 和共享库干扰 ROS。`src/livox_ros_driver` 仅包含兼容原类型名的消息定义，不包含连接设备的驱动；实时采集需另行部署 MID360 硬件驱动。

## 编译与检查

```bash
git clone https://github.com/Silentbarber/yhbuildmap_ws.git
cd yhbuildmap_ws
bash scripts/build_ros1.sh
bash scripts/test_release.sh
```

默认 `BUILD_JOBS=2`。算法在 `algorithms/gaoxiang12__faster-lio` 独立 catkin 工作区编译。消息和转换节点通过相对符号链接加入该工作区；catkin 顶层 CMakeLists 在编译时生成。测试覆盖单平面退化回环拒绝、三正交面回环通过、平面信息退化方向、弱观测权重、时间戳抖动、CustomMsg Header 读取以及 MID360 点过滤。

## 完整录包建图

将 `--bag` 指向本机的实际文件。原始 bag、PCD、NPZ、日志和编译产物不随代码仓库分发。回放和后端会生成逐帧缓存，完整运行前应预留足够磁盘空间。

**外参必须与录制时的坐标处理一致：**

| 录制包 | 参数 | 对应配置 |
| --- | --- | --- |
| `test_1301_2026-10-01-21-52-47.bag` | `--calibration raw` | `handheld_mid360.yaml` |
| `test_1301_2026-10-02-16-21-02.bag` | `--calibration pitch25` | `handheld_mid360_cloud_pitch25.yaml` |
| `test_1301_2026-10-02-16-23-42.bag` | `--calibration pitch25` | 同上 |
| `test_1301_2026-10-02-16-27-44.bag` | `--calibration pitch25` | 同上 |

`pitch25` 只用于已确认点云经过约 +25° 俯仰旋转而 IMU 未同样旋转的录制：`p_cloud = Ry(+25°) p_raw`，因此使用 `R_IL = Ry(-25°)`。这是这批数据的录制补偿，不是所有 MID360 都适用的通用外参。其他设备或录制配置需要实际标定。

第三包的完整命令：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/run_faster_robust_mapping.py \
  --bag /path/to/test_1301_2026-10-02-16-23-42.bag \
  --calibration pitch25 --rate 0.75 \
  --backend-profile observation_geometry \
  --output-dir "$PWD/results/162342_release"
```

其他三个包替换路径和表中 `--calibration`，各自使用新的输出目录。脚本启动独立 ROS master 并检查完整输入计数和末帧覆盖，不需要手动启动 roscore。输出目录必须在本仓库内且尚不存在，防止覆盖既有结果。

主要输出：

| 文件 | 用途 |
| --- | --- |
| `backend/optimized_3cm.pcd` | 后端完成后的完整地图 |
| `backend/report.json` | 约束验证、位姿图及重建统计 |
| `frontend/run.json`、`frontend/evaluation.json` | 输入完整性、配置、代码哈希和原前端评估 |
| `frontend/state_trace.csv` | 观测诊断字段 |
| `frontend/capture/` | 逐帧重建所需缓存和轨迹 |
| `validation/` | 早晚重访一致性检查 |
| `pipeline.json` | 本次实际使用的地图路径和运行状态 |

没有通过验证的回环时，脚本保留原前端地图并在 `pipeline.json` 中给出实际路径；不要假定后端一定产生了优化地图。每次新场景仍需整体和局部复核。

两个前端实验开关 `--experimental-hard-guard` 和 `--experimental-raw-scan-end` 默认关闭，曾在回归中造成变差，不属于当前验收配置。`--backend-profile legacy` 仅用于旧后端对照。

## 只重跑后端

保留完整 `frontend/capture/` 和本版 `state_trace.csv` 时，可以避免再次回放 bag：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 scripts/refine_independent_pose_graph.py \
  results/162342_release/frontend \
  --submap-half-window 0.15 --max-candidates 8 --validate-loops \
  --keyframe-step 1 --local-geometric-edges --plane-information \
  --observation-weighted-odometry --loop-search-radius 5 \
  --loop-overlap 0.35 --heldout-overlap 0.30 \
  --output-dir results/162342_backend_rerun
```

旧版缺少观测诊断字段的缓存不能直接用于观测加权后端，需要本版前端重新回放。后端不使用其他算法的轨迹。

## 三维查看

在有桌面显示的机器上，每次独立打开一张地图：

```bash
/usr/bin/python3 scripts/view_pcd.py results/162342_release/backend/optimized_3cm.pcd
```

Open3D 窗口支持鼠标旋转、拖动和缩放。`--voxel 0.06` 可以减少显示点数，不修改源 PCD。该查看器需要图形显示环境；仓库未打包原实验目录中依赖本机地图路径的网页。

## 四包实测记录

以下为发布前本地完整运行的记录，来源为 `results/faster_lio_robustness/refined_summary.json`。全部输入计数通过检查，所有捕获的估计器输出帧均用于重建。它们不是此次源码打包时重新运行得到的结果。数据和截图留在原本机工作区。

| 包时间 | 输出帧 | 3 cm 地图点数 | 原/优化后诊断旋转 | 原/优化后留出点 NN 中位数 |
| --- | ---: | ---: | ---: | ---: |
| 10-01 21:52:47 | 2253 | 589414 | 0.0157° / 0.0170° | 1.775 / 2.265 cm |
| 10-02 16:21:02 | 1261 | 517521 | 0.0524° / 0.0986° | 3.592 / 3.341 cm |
| 10-02 16:23:42 | 2387 | 645399 | 7.2793° / 0.0661° | 25.022 / 4.094 cm |
| 10-02 16:27:44 | 879 | 503873 | 0.3611° / 0.0737° | 2.626 / 2.330 cm |

旋转来自重访区域的诊断配准，NN 中位数是留出点在原输出坐标中的最近邻一致性，均不是外部真值误差。第三包的建筑整体错位明显减少；部分原本较好的包存在小幅指标退步，不能声称所有指标都提高。动态物体投影和局部残差仍可能存在。第三包局部固定表面筛选后的跨时段 p95 残差中位数约 1.095 cm，最大值约 29.95 cm，不能据低中位数保证全图无错误。

## 来源与许可证

Faster-LIO 基于 [gaoxiang12/faster-lio](https://github.com/gaoxiang12/faster-lio)，固定上游提交 `8038fbac95283708fe2aa30658b7e116a7ace6d8`。论文：Chunge Bai et al., *Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels*, IEEE Robotics and Automation Letters, 2022。

保留上游 `LICENSE` 的 GPLv2 文本和代码作者声明。上游 `package.xml` 的 BSD 标记与其实际 LICENSE 不一致，本版将包元数据改为 GPLv2，与保留的许可证一致。本仓库新建的优化、运行和检查脚本采用根目录 GPLv2；Livox 消息接口保留 MIT，转换节点采用其包声明的 BSD-3-Clause，第三方已有声明继续适用。修改清单、来源和边界见 [MODIFICATIONS.md](MODIFICATIONS.md)。
