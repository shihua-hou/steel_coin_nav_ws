# 钢镚四足机器狗导航工作区（steel_coin_nav_ws）

智身「钢镚」L1（AgiBot D1 Edu-Ultra，12 自由度）的上层导航软件栈：**建图 → 保存地图 → 定位 → Nav2 导航**，配一套平板/浏览器可用的网页控制台。

| 项 | 内容 |
|---|---|
| 主控 | RK3588S，Ubuntu 22.04 aarch64，ROS 2 Humble |
| 雷达 | Livox MID-360S，躯干前方 0.25 m、离地 0.45 m，**向前低头 19.63°** |
| 底盘 | AgiBot SDK（`mc_sdk`），经 `genisom_bridge` 接入 ROS |
| 导航链路 | FAST-LIO → 里程计滤波 → 点云转激光（stage2，含近场补点）→ Nav2（AMCL + NavFn + RPP） |
| 建图链路 | Super-LIO → `map.pcd` → pcd2pgm → 2D 占用栅格 |

整合来源：[robot-dog-console](https://github.com/shihua-hou/robot-dog-console)（SDK 桥、TF、建图、网页控制台）+ [Go2W-navigate-project](https://github.com/zhuaoyuRobo/Go2W-navigate-project)（FAST-LIO→Nav2 的节点拓扑，Apache-2.0）。只借用 Go2W 的节点拓扑；外参、滤波参数一律用本机标定，**钢镚 ≠ Go2**。

> **开发前必读**：[开发注意事项.md](./开发注意事项.md)（踩过的坑、红线、版本迭代清单）· [系统测试流程.md](./系统测试流程.md)（现场逐步测试命令）

---

## 目录

1. [系统架构](#1-系统架构)
2. [目录结构](#2-目录结构)
3. [硬件、网络与端口](#3-硬件网络与端口)
4. [编译](#4-编译)
5. [快速开始](#5-快速开始)
6. [网页控制台](#6-网页控制台)
7. [命令行启动模式](#7-命令行启动模式)
8. [坐标系、外参与关键参数](#8-坐标系外参与关键参数)
9. [Web API](#9-web-api)
10. [运维与故障排查](#10-运维与故障排查)
11. [已知限制与后续计划](#11-已知限制与后续计划)
12. [与旧工程 robot_ws 并存](#12-与旧工程-robot_ws-并存)

---

## 1. 系统架构

### 导航（默认，`odom_source:=fastlio`）

```
Livox MID-360S ──/livox/lidar,/livox/imu──▶ FAST-LIO ──/Odometry、/cloud_registered
      │                                         │
      │        ┌────────────────────────────────┴───────────────────────┐
      │        ▼                                                        ▼
      │  fastlio_nav2_odom_filter                         stage2_pointcloud_to_laserscan
      │  调平 + /odom_nav + TF odom→base_footprint         点云 → body → 外参 → base_footprint
      │        │                                          高度带 0.05~0.85 m → /scan
      └──▶ near_field_cloud（C++）──/near_cloud──────────────────▶ 并入 /scan（补 0.5 m 内盲区）
               │                                                        │
               └───────────────────────┬────────────────────────────────┘
                                       ▼
          Nav2：map_server + AMCL + NavFn（全局，含实时障碍层）+ RPP（跟线）
          有 maps/<图名>.datum.yaml 时，map→odom 改由 global_pose_arbiter 发（见下）
                                       │ /cmd_vel → velocity_smoother
                                       ▼
                     genisom_bridge ──AgiBot SDK (UDP 43988)──▶ 狗端 mc_ctrl
```

- **定位方式**：没有 `maps/<图名>.datum.yaml` 时，人在地图上给初始位姿（`/initialpose`）→ AMCL 跟踪。有这份对齐文件、且户外 **RTK 固定**（GGA 质量 4 且 RMC 模式 R）时，位置跟卫星坐标走，不用再设初始位姿；浮点解和单点不解不动 `map→odom`。没有自动全局重定位（2D 盲搜、3D-BBS 都已移除）。不做「记住上次位姿」：换电池时常会手动遥控把狗挪走，上次位姿不可靠；换电池后再开导航，固定解会按当前卫星位置重新放下狗。
- **里程计**：FAST-LIO（Go2W 做法），**不用狗本体里程计**。odom_filter 用外参把低头 19.6° 的 `camera_init` 调平，并吸收单帧跳变。
- **近场补点**：FAST-LIO `blind 0.5` 会丢掉雷达 0.5 m 内的点（不能调小，近点会让 LIO 抖）；`near_field_cloud` 直接从原始 `/livox/lidar` 取 0.25~0.70 m 的点发 `/near_cloud`，stage2 并入 `/scan`，狗头前站人 / 贴墙才看得见。
- **跟线与避障**：NavFn 全局规划（全局代价图带 `/scan` 实时障碍层，人进来会绕）；RPP 只追全局路线，不侧移、不倒车。行为树只在「目标变了或路线被挡」时才重规划；被挡先等，不原地转圈、不自动清局部代价图。
- **NDT 点云定位**（`lidar_localization_ros2` + `ndt_omp_ros2`）的源码已经放在仓库里，但默认不编译（`COLCON_IGNORE`），属于后续计划，见 §11。

### 建图

```
Livox ──▶ Super-LIO ──/lio/cloud_world──▶ map_building_node（网页实时 2D 预览 /map_building）
              └── 停止时落盘 src/SUPER_LIO/super_lio/map/map.pcd ──▶ pcd2pgm ──▶ maps/<名>.pgm + .yaml
```

建图（Super-LIO）和导航定位（FAST-LIO）**互斥**，不能同时运行。

保存地图只生成局部栅格，**不会**自动写出卫星对齐。对齐文件是同目录的 `maps/<图名>.datum.yaml`（地图原点的 WGS84、地图 +x 相对正东的转角、天线航向零偏）。有这个文件，启动导航才会拉起 `rtk_to_map` 和 `global_pose_arbiter`。2026-10-03 的 `map_liveground261003` 已用一段户外直走轨迹写好对齐，位置残差大约 4 cm。建图全程都是 RTK 固定时，本可以用那一遍建图轨迹在保存时直接算出对齐，这一步还没接到保存按钮上。

### 网页控制台（三个服务缺一不可）

| 端口 | 进程 | 作用 |
|---|---|---|
| 8080 | `scripts/web_http_nocache.py` | 静态页面 `web_ui/index.html` |
| 8090 | `web_ui/web_ops_node.py` | HTTP API：启停建图/导航、地图管理、电量、遥控、图传、自愈 |
| 9090 | `rosbridge_websocket` | 网页订阅话题（激光、位姿、路径、代价地图等） |

由 systemd 服务 `dog-web-console.service`（看门狗 `scripts/web_console_watchdog.py`）开机自启并保活。

---

## 2. 目录结构

```
steel_coin_nav_ws/
├── README.md                 # 本文件：项目总入口
├── 开发注意事项.md            # 踩坑记录 / 红线 / 版本迭代清单（改代码前必读）
├── 系统测试流程.md            # 现场分阶段测试命令
├── build_workspace.sh        # 一键编译（含 apt 依赖）
├── start_quad.sh             # 一键启动：nav / mapping / localization / dog / lio / sensors
├── start_ui.sh               # 手动拉起网页三件套（8080 / 8090 / 9090）
├── start_scan_node.sh        # 单独起旧链路 nav_scan_node（调试用）
├── stop_old_stack.sh         # 清理旧工程 robot_ws 的残留进程
├── config/
│   ├── lidar_extrinsic.yaml      # IMU 重力标定结果（pitch_down 19.629°，见 §8 符号说明）
│   ├── obstacle_height_band.yaml # 障碍高度带 0.05~0.85 m（stage2 与 pcd2pgm 共用）
│   ├── fastdds_udp_only.xml      # FastDDS 只走 UDP（所有 ROS 进程必须带，见 §8）
│   ├── nav_speed.json            # 设置页「导航速度 / 勇敢模式」保存值（web_ops 写，启动导航时套用）
│   ├── oem700_cors.yaml.example  # 六分 CORS 账号样例（无密码）
│   └── oem700_cors.yaml          # 本机 CORS 账号（gitignore，权限 600，勿提交）
├── maps/                     # 导航地图 <名>.pgm + <名>.yaml；可选 <名>.datum.yaml（RTK 对齐）
├── scripts/                  # 运维脚本（见 §10）
│   └── diag/                     # 只读诊断小工具：定位偏差、碰撞快照、路线稳定性（见 §10）
├── web_ui/
│   ├── index.html                # 网页控制台（单文件）
│   ├── web_ops_node.py           # 后端 HTTP API + ROS 节点
│   └── icons/ models/ roslib.min.js three.min.js
└── src/
    ├── livox_ros_driver2         # Livox 官方驱动（MID-360S 配置）
    ├── FAST_LIO                  # 导航用 LIO（config/mid360.yaml）
    ├── oem700_rtk                # OEM700：NMEA 驱动、WGS84→地图、map→odom 仲裁
    ├── near_field_cloud          # C++：雷达 0.25~0.70 m 原始点 → /near_cloud，补 FAST-LIO blind 近场盲区
    ├── SUPER_LIO                 # 建图用 LIO（输出 /lio/odom、/lio/cloud_world、map.pcd）
    ├── genisom_bridge            # AgiBot SDK ↔ ROS（cmd_vel、状态、站立/趴下服务）
    ├── isaac_go2_nav2            # Go2W 移植：odom_filter、stage2、steel_coin_nav2.launch.py、Nav2 参数、behavior_trees/
    ├── nav2_tools                # map_building_node、pcd2pgm、nav_scan_node（旧链路）
    ├── lio_tf_bridge             # 旧链路里程计/TF 桥（odom_source:=dog|lio 时用）
    ├── lidar_localization_ros2   # NDT 定位（COLCON_IGNORE，未启用）
    ├── ndt_omp_ros2              # NDT/GICP OpenMP 加速（COLCON_IGNORE，未启用）
    └── auto_relocalize           # 已废弃的 2D 盲搜重定位（COLCON_IGNORE，勿再启用）
```

> 2026-09-30 早期联调留下的 `diag*.txt`、`status*.txt`、`verify*.txt`、`probe*.txt`、`build*.log`、`launch_*.txt`、`rosbridge.log` 等诊断输出已收在 `archive/2026-09-30/`，运行时不依赖它们。`log/` 是 colcon 编译日志，不要挪走。

---

## 3. 硬件、网络与端口

| 网卡 | 地址 | 对端 / 用途 |
|---|---|---|
| eth0 | `192.168.1.150/24` | Livox MID-360S（`192.168.1.179`） |
| eth0 | `192.168.168.150/24` | 狗本体 `192.168.168.168`：SDK（UDP 43988）、图传 RTSP `:8554` |
| wlan0 | `10.89.39.22` | SSH、网页、平板 |

- SSH：主控 `linaro@10.89.39.22`；狗 `firefly@192.168.168.168`。
- 网页地址：`http://10.89.39.22:8080`。**要用 wlan IP 打开**，不要用 192.168.168.x 网段。
- SDK 三处配置必须一致：狗端 `/opt/export/config/sdk_config.yaml`（`target_ip: 192.168.168.150`, `target_port: 43988`）、狗端 `start_motion_control.sh` 的 `SDK_CLIENT_IP=192.168.168.168`、主控 `bridge.launch.py`。运控升级后常被重置，详见《开发注意事项》§3.5。

---

## 4. 编译

```bash
cd /home/linaro/steel_coin_nav_ws
bash build_workspace.sh            # 首次：装依赖 + colcon build（--symlink-install）
```

日常只改了某一个包时：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select isaac_go2_nav2 --symlink-install
source install/setup.bash
```

- 用的是 `--symlink-install`，**只改 Python 源码或 launch 文件时不需要重新编译**，重启对应进程即可。
- `web_ui/` 下的文件直接从源码运行。改了 `index.html` 只需在浏览器里刷新；改了 `web_ops_node.py` 需要重启 web_ops（§10）。

---

## 5. 快速开始

### 网页（推荐）

主控开机后，网页三件套会由 `dog-web-console.service` 自动拉起。浏览器打开 `http://10.89.39.22:8080`：

1. **建图**：首页 → 建图 → 「开始建图」→ 用摇杆让狗慢走覆盖场地 → 「停止建图」→ 等提示「PCD 就绪」→ 填名字 →「保存地图」（十几秒）。
2. **导航**：首页 → 导航 → 选地图 →「启动导航」（约 15–25 秒）
   → 「设初始位姿」：在狗的实际位置**按下**，朝狗头方向**拖动**，**松开**
   → 红色激光点贴合黑色墙线后点「确认定位」
   → 「设目标点」：按下目标点、拖出朝向、松开下发。
3. 建图和导航不能同时开：建图前先关闭导航。

如果网页打不开或首页数据为空：`bash /home/linaro/steel_coin_nav_ws/start_ui.sh`。

### 命令行

```bash
cd /home/linaro/steel_coin_nav_ws
bash start_quad.sh mapping            # 建图
bash start_quad.sh                    # 导航（默认 Go2W：FAST-LIO + odom_filter + stage2 + Nav2）

# 或直接调 launch
ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=mapping
ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=nav map:=/home/linaro/steel_coin_nav_ws/maps/map_live.yaml
```

让狗站起来：网页「站立」，或者

```bash
ros2 service call /bridge/stand_up std_srvs/srv/Trigger
```

> `genisom_bridge` 连上狗之后，遥控器会失效，这是正常现象。断开 SDK 的瞬间狗会趴下，所以所有脚本都**刻意不杀** `genisom_bridge`。

---

## 6. 网页控制台

| 页面 | 内容 |
|---|---|
| 首页 | 时钟、CPU / 内存 / 硬盘 / 电量、GNSS、运动模式、控制状态（APP/SDK 切换）；状态胶囊：激光 · 底盘 · Go2W · LIO · 扫描 · 导航 · 桥接 · **SDK** · **RTK** |
| 导航 | 见下文 |
| 建图 | Super-LIO 实时 3D 点云 + 狗身相机画面；开始 / 停止 / 保存地图 |
| 地图库 | 地图缩略图、设为导航地图、编辑（画笔修图）、删除 |
| 运控 | 全屏相机画面 + 双摇杆；阻尼 / 站立 / 匍匐；低 / 中 / 高速；拍照 / 录像 |
| 设置 | 连接地址、**导航速度**、狗动作、系统日志 |

**SDK 胶囊**：绿色表示狗的状态数据是新鲜的；红色表示没连通，鼠标悬停可以看到数据多少秒没更新了。变红时的处理见 §10。

**RTK 胶囊**（SDK 右侧）：绿色表示 OEM700 驱动进程在跑，灰色表示没启动。它只表示驱动在不在，不表示有没有固定解。固定解看首页 GNSS 磁贴（RTK固定 / RTK浮点 / 无信号）。

**GNSS 磁贴**：驱动订阅 `/gnss/fix` 与 `/gnss/nmea`。数据年龄超过 3 秒显示无信号。导航页「来源」为 RTK 固定 / AMCL / 保持上一帧。

**导航速度（设置页卡片）**：导航中也能改，点「应用」约 4 秒生效、不用重启；值存 `config/nav_speed.json`，每次启动导航自动套用。

| 项 | 范围 / 默认 | 说明 |
|---|---|---|
| 前进速度 | 0.10–0.45 m/s，默认 0.22 | 同时调 RPP 前视距离 = max(0.45, 1.4×速度)。窄门、人多的场地建议 ≤0.30；上限 0.45 是因为雷达 0.5 m 内有盲区 |
| 转向速度 | 0.25–0.80 rad/s，默认 0.45 | 原地转向（掉头、偏差 >30° 时先转正）的速度 |
| 预设 | 慢 0.15 / 标准 0.22 / 快 0.32 | |
| 勇敢模式 | 默认**开** | 开：大胆沿全局路线走，只在机身轮廓真压到障碍时才停（人旁 0.8 m 缝也过）；关：离障碍约 10 cm 就提前停。开着时离人会很近，真碰上会原地卡住（不倒车）要人挪开 |

### 导航页布局

- **标题下方一排按钮**：启动导航 · 关闭导航 · 地图选择 · 换地图 ｜ 交互 · 设初始位姿 · 设目标点 · 测距 · 跟随机器人 · 适应地图 · 显示/隐藏摇杆。
  快捷键：`I` 交互、`P` 设初始位姿、`G` 设目标点、`M` 测距、`F` 跟随、`Z` 适应地图、`J` 摇杆、`Esc` 回到交互。
- **左侧 Nav2 面板**：任务状态与进度、剩余距离、预计剩余时间、恢复次数、定位状态、AMCL 误差 σ、机器人位姿；「确认定位」「取消导航」。
- **右侧实时画面**：狗身相机画面，可折叠，折叠后停止拉流。
- **画面**：地图、网格、激光点、当前位姿、初始位姿 / 目标点**固定显示**。
- **底部图例**：网格标尺（一格的实际尺寸）· 激光点 · 代价地图 · 局部路径 · 全局路径。后三项可以点击开关。
- 初始位姿、目标点、取消导航走 HTTP 下发（rosbridge 只作兜底），避免 rosbridge 首次发布时丢包。

---

## 7. 命令行启动模式

`bash start_quad.sh <模式>`：

| 模式 | 启动内容 | 用途 |
|---|---|---|
| `nav`（默认） | Livox + FAST-LIO + odom_filter + near_field_cloud + stage2 + Nav2 + SDK 桥 | 生产导航 |
| `localization` | 同上但**不含 Nav2**；单实例锁 `/tmp/steel_coin_localization.lockdir` | 网页「启动导航」和自愈调用；救急重建定位链 |
| `mapping` | Livox + Super-LIO + map_building | 建图 |
| `dog` | 狗里程计（lio_tf_bridge）+ nav_scan_node（约 19.6° 校平） | 旧链路，对照用 |
| `lio` | Super-LIO 平面里程计 + nav_scan_node | 调试 |
| `sensors` | 仅传感器 + SDK 桥 | 硬件自检 |

launch 参数（`steel_coin_nav2.launch.py`）：`mode:=nav|mapping`、`odom_source:=fastlio|dog|lio`、`map:=<yaml>`、`rviz:=false`。

| `odom_source` | 里程计 | /scan 来源 | 适用 |
|---|---|---|---|
| `fastlio`（默认） | FAST-LIO + odom_filter（`/odom_nav`） | stage2 | 生产 |
| `dog` | 狗底盘里程计（lio_tf_bridge） | nav_scan_node | 对照 |
| `lio` | Super-LIO 平面里程计（容易漂） | nav_scan_node | 调试 |

---

## 8. 坐标系、外参与关键参数

### 坐标系

| 链 | 帧 |
|---|---|
| Nav2，无 datum | `map →(AMCL) odom →(odom_filter) base_footprint` |
| Nav2，有 datum 且 RTK 固定 | `map →(global_pose_arbiter) odom →(odom_filter) base_footprint`；AMCL 不再发 TF |
| LIO | `camera_init →(FAST-LIO) body`；`body → base_link` 为静态 TF，仅用于显示 |

`odom → base_footprint` 只能由 odom_filter 一家发布。**禁止**再发 `odom ≡ camera_init` 之类的静态 TF，否则会抢父帧，位姿和激光会乱飘。

有 datum 时 `map → odom` 只能由 `global_pose_arbiter` 发布（启动导航时把 AMCL 的 `tf_broadcast` 改成 false，只改 `/tmp` 里的参数副本，源文件保持 true）。OEM700 驱动读 by-id 的 `…-if06`（UM982 NMEA），不要去打开另外三个 CDC 口，尤其是 RTCM 那路。差分账号在板子上的 `config/oem700_cors.yaml`，不进仓库。话题：`/gnss/fix`、`/gnss/nmea`、`/gnss/heading`、`/gnss/rtk_fixed`、`/rtk/map_pose`、`/gnss/pose_source`。

### 雷达外参（2026-10-01 修正，务必注意符号）

- 安装位置：雷达在 `base_footprint` 中位于 (0.25, 0, 0.45) m，**向前低头 19.63°**，横滚 -0.78°。
- IMU 重力实测（狗静止时）：`body` 系的「上」方向 = (-0.33, 0, 0.94)，即低头。
- `body → base_link`（base 在 body 中的位姿）的正确值：

  ```
  t = (-0.084303, 0.006341, -0.507792)
  q = (0.006523, -0.170463, 0.000000, 0.985343)    # qy 为负！
  ```

- 这组值同时出现在 4 处，必须一起改：`steel_coin_nav2.launch.py`（静态 TF + stage2 参数）、`start_quad.sh`（同上）、`web_ui/web_ops_node.py`（`ensure_scan_node`）、`src/lio_tf_bridge/launch/bridge.launch.py`。
- ⚠️ `config/lidar_extrinsic.yaml` 里的 `lidar_pitch: -0.3426` 是「负号表示低头」的写法，**和 ROS 约定相反**（ROS 绕 y 轴转正角才是低头）。旧外参就是照着它写成了抬头，导致前方激光全被高度带滤掉、后方地面被当成障碍。验证方法：`/scan` 前方 ±45° 每帧应有几十束（修正前只有 3–4 束）。

### DDS 通信（必须）

所有 ROS 进程都要带上环境变量 `FASTRTPS_DEFAULT_PROFILES_FILE=/home/linaro/steel_coin_nav_ws/config/fastdds_udp_only.xml`（关闭 FastDDS 共享内存，只走 UDP）。启动脚本、网页看门狗、`~/.bashrc` 已经设置好；**自己写新的启动脚本或 systemd 服务时也要加上**。不加的话，进程之间可能「看得到话题、收不到数据」：SDK 误报未通、网页没数据、Nav2 启动卡住、`ros2 node list` 为空。

**内核 UDP 缓冲也必须调大**（`/etc/sysctl.d/60-ros2-dds.conf`：`rmem_max`/`wmem_max` 32 MB、`rmem_default` 8 MB）。一帧 `/livox/lidar` 约 0.4 MB，默认 `rmem_max` 只有 0.2 MB，会大量丢帧，FAST-LIO 靠 IMU 硬推直到发散（导航中定位飘、乱走乱撞）。检查：`cat /proc/sys/net/core/rmem_max` 应为 33554432。改完后进程要重启才生效。

**`ros2` 命令行加 `--no-daemon`**：主控重启后 `ros2` daemon 常处于坏状态（报 `!rclpy.ok()`），走 daemon 的 `ros2 topic echo` 会永远读不到，看起来像「话题没数据」。手工排查和脚本里一律用 `ros2 topic echo --once --no-daemon …`、`ros2 lifecycle get --no-daemon …`、`ros2 param … --no-daemon`（web_ops 和 `fix_sdk_link.sh` 已改）。`ros2 topic hz` 不受影响。

### 关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| 障碍高度带 | 0.05 ~ 0.85 m | `config/obstacle_height_band.yaml`；stage2 和 pcd2pgm 共用 |
| 局部代价地图 `/scan` 源 | `min_obstacle_height -0.10`、`max_obstacle_height 0.50` | `/scan` 是平面激光（z=0），**下限不能大于 0**，否则墙全被滤掉 |
| 机身轮廓 footprint | 局部 0.65 × 0.38 m（Nav2 再加 padding 0.01，发布 0.67 × 0.40） | D1 实测 0.635 × 0.36 m。**防撞用这个** |
| 全局规划轮廓 | 0.65 × **0.26** m（发布 0.67 × 0.28） | 只给全局规划用的「瘦」轮廓：人旁 0.8 m 缝不再被激光抖动「堵死」→ 路线不来回跳 |
| 膨胀 | 局部 0.45 m / 系数 3.0；全局 **0.90 m / 系数 4.0** | 全局半径盖到走廊中线（走廊宽 ~1.7 m）让路线居中；系数 4.0 让窄缝代价别太高 |
| 全局代价图障碍层 | `/scan`，`obstacle_max_range 2.5`，更新 2 Hz | 人/临时障碍进全局图，规划器会绕；只收近处，免得远处墙点把门洞描窄 |
| 局部代价图 `/scan` 源 | `min_obstacle_height -0.10`、`max_obstacle_height 0.50` | `/scan` 是平面激光（z=0），**下限不能大于 0**，否则墙全被滤掉 |
| stage2 `range_min / range_max` | 0.25 / 12 m | 关闭孤立束滤波、关闭腿部自滤波（雷达装在狗头上方，照不到腿） |
| stage2 近场 | `near_topic /near_cloud`、`near_self_box` x±0.34 / y±0.20 | 合并 near_field_cloud 的点；机身轮廓内的点丢弃（兜底） |
| stage2 `angle_increment` | 0.5° | |
| stage2 `stamp_mode` | **`tf`**（最新 odom TF 时间 - 0.1 s） | **禁止改回 `now`**：会触发 Humble tf2 MessageFilter 死锁，Nav2 收不到激光、初始位姿设不上 |
| `/scan` QoS | BEST_EFFORT | 只能有一个 stage2 实例 |
| Nav2 参数 | `src/isaac_go2_nav2/config/steel_coin_nav2_params.yaml` | AMCL + NavFn + RPP；每个 `ros__parameters:` 下至少要有一个真实键 |
| AMCL | `sigma_hit 0.10`、`max_beams 360`、`update_min_d/a 0.05 / 0.08`、alpha1–4 0.15 | 收紧后门口实测残差 3.5 cm / 0°（原 11 cm / 6°，6° 会让进门转弯甩出去贴门框） |
| 控制器 RPP | 前视 0.45 m（随速度变长）、偏差 **>30°** 先原地转正、弯道 `regulated_linear_scaling_min_radius 0.9` | 前视别短于 0.45：狗转向响应慢，0.30 时进门 90° 弯冲过头。**别开** `use_cost_regulated_linear_velocity_scaling`（低速时碰撞预判失效，会蹭到人卡死） |
| RPP 碰撞预判 | 勇敢模式 0.1 s（接触才停）/ 保守 0.6 s（提前 ~10 cm 停） | 设置页开关；在线 `ros2 param set /controller_server FollowPath.max_allowed_time_to_collision_up_to_carrot <值>` |
| 行为树 | `behavior_trees/*_no_backup.xml` | 只在目标变了或路线被挡（`IsPathValid`）时重规划；被挡先 Wait（规划失败 1 s、跟随失败 1 s、外层 2 s），兜底只清**全局**图；无 Spin、无 BackUp；**局部代价图永不自动清** |
| 禁止倒车 | RPP `allow_reversing: false`、velocity_smoother vx 下限 0、行为树无 BackUp | 正后方是盲区；身后的目标会先原地转向再前进 |
| FAST-LIO | `blind 0.5`、体素 `filter_size_surf/map 0.3`、`point_filter_num 2` | **blind 别调小**（0.25 试过：LIO 抖、激光跟着飘），近处靠 near_field_cloud；体素 0.3 防窄走廊打滑 |
| FAST-LIO `pcd_save_en` | **false** | 打开会把每帧点云攒在内存里，约半小时耗尽内存被杀 |
| odom_filter | `base_extrinsic_*`（调平）、`max_abs_z 3.0`、单帧跳变吸收、2 s 内跳 >4 次或连续拒收 >1 s 停发 TF | 斜系直接取 x/y/yaw 转 90° 后差 0.3 m / 7°；FAST-LIO 真发散时让 Nav2 停车而不是带着冻结里程计乱走 |

---

## 9. Web API

后端 `web_ops_node.py`，端口 8090。

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/sysinfo` | CPU / 内存 / 硬盘 / 各服务状态 |
| GET | `/api/battery` | 电量与 `sdk` 字段（状态数据是否新鲜） |
| GET | `/api/maps` · `/api/map/pgm?name=` · `/api/map/preview?name=` | 地图列表 / 原图 / PNG 缩略图 |
| POST | `/api/map/edit` · `/api/map/delete` | 保存编辑 / 删除地图 |
| POST | `/api/mapping/start` · `stop` · `save` · `discard` | 建图：开始 / 停止并落盘 PCD / 转 PGM 保存 / 丢弃 |
| GET | `/api/mapping/status` | 建图状态 |
| POST | `/api/nav2/start` `{map}` · `/api/nav2/stop` | 启动（定位链 + 单例 Nav2）/ 关闭导航 |
| GET | `/api/nav2/map` | 当前导航地图（`/map` 缓存，没有时读磁盘） |
| POST | `/api/nav2/init_pose` `{x,y,yaw}` | 设初始位姿 |
| POST | `/api/nav2/accept_pose` | 确认定位，允许下发目标 |
| POST | `/api/nav2/goal` `{x,y,yaw}` · `/api/nav2/cancel` | 下发 / 取消导航目标 |
| GET / POST | `/api/nav2/speed` `{linear, angular, brave}` | 读取 / 保存并在线应用导航速度（设置页「导航速度」卡片） |
| POST | `/api/nav2/reload_map` `{map}` | 热换图（换完要重新设初始位姿） |
| POST | `/api/ctrl` `{mode:APP\|SDK, force_restart}` | 切换控制模式 / 重启 bridge |
| POST | `/api/sdk/heal` | 重启狗端 mc_ctrl 和 bridge（**狗会趴下**） |
| POST | `/api/cmd_vel` `{vx,vy,wz}` | 网页遥控 |
| POST | `/api/restart_rosbridge` | 重启 rosbridge |
| GET | `/api/video/mjpeg` · `/api/video/snapshot` | 狗身相机 MJPEG / 单帧 |
| POST | `/api/video/stop` · `resume` · `profile` | 图传控制 |
| GET | `/api/log?file=&n=` | 读日志 |

网页额外订阅的话题：`/web/robot_pose`、`/web/scan_map`、`/web/local_costmap`、`/web/local_plan`、`/plan`、`/web/nav_status`、`/web/nav_feedback`、`/web/sys_status`。

---

## 10. 运维与故障排查

### 一键脚本

| 场景 | 命令 |
|---|---|
| 首页 SDK 变红 / 狗站不起来 | `bash scripts/fix_sdk_link.sh`（只诊断加 `--check`） |
| 网页打不开 / 首页无数据 | `bash start_ui.sh` |
| 定位链卡死、重建（不碰 SDK，狗不动） | `rm -rf /tmp/steel_coin_localization.lockdir; bash start_quad.sh localization` |
| 狗端出现多个 mc_ctrl | `bash scripts/ensure_dog_mc_ctrl.sh`（`--restart` 强制重启，狗会趴下） |
| 安装 / 重装网页开机自启 | `sudo bash scripts/install_web_console_service.sh` |
| 只重启 web_ops | `bash scripts/fix_webops.sh` |

### 诊断小工具（`scripts/diag/`，只读，不会让狗动）

先 `source /opt/ros/humble/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/linaro/steel_coin_nav_ws/config/fastdds_udp_only.xml`。**调导航参数前先量，别靠猜**——2026-10-02 两次凭感觉调参都调坏了，量清楚后一次就好。

| 脚本 | 回答的问题 | 用法 |
|---|---|---|
| `scan_map_align.py` | 定位准不准？把当前 `/scan` 在静态地图上小范围平移/旋转找最贴合位置，输出 AMCL 还差多少。残差 ≤5 cm、≤1° 算准 | 狗静止时先 `ros2 service call /request_nomotion_update std_srvs/srv/Empty` 几次，再运行 |
| `collision_snapshot.py` | 狗为什么停？每次 RPP 报 `detected collision ahead`，记录机身轮廓 0.3 m 内的障碍格（+x 前 +y 左）→ `/tmp/collprobe.log` | `setsid nohup python3 collision_snapshot.py &`，然后让狗走（20 分钟后自动退出） |
| `plan_probe.py` | 路线稳不稳？记录每次全局路线变化（长度、横向偏移）和狗停下/原地转的时段 → `/tmp/planprobe.log` | 同上 |
| `gap_probe.py` | 狗卡住时正前方地图里能走多宽 → `/tmp/gapprobe.log` | 同上（15 分钟） |

### 常见现象

| 现象 | 原因 | 处理 |
|---|---|---|
| SDK 显示未通，但狗其实正常 | web_ops / rosbridge 没收到 bridge 的话题（运行久了的进程和新拉起的进程没建立 DDS 连接） | `fix_sdk_link.sh` 会自动识别，只重启 web_ops + rosbridge。**先别重启狗端运控** |
| 设初始位姿没反应、网页激光卡住 | Nav2 的 tf2 MessageFilter 死锁（stage2 `stamp_mode` 被改回了 `now`） | 确认 stage2 是 `stamp_mode:=tf`，然后整个重启 Nav2 |
| 导航页没有激光点、机器人位姿不动，但 Nav2 状态显示运行中 | rosbridge 连得上但收不到 ROS 数据（常见于开机后） | `curl -X POST http://127.0.0.1:8090/api/restart_rosbridge` |
| 建图后开导航：激光冻结、显示无 map 坐标系 | 建图残留的孤儿雷达驱动和导航的驱动抢雷达，FAST-LIO 收不到 IMU 卡死 | 已修复：停止建图会清孤儿驱动，启动导航会检查 FAST-LIO 输出。手动处理：重建定位链 |
| FAST-LIO 运行半小时左右被杀、导航中断 | `pcd_save_en: true` 导致内存泄漏（已修复为 false） | 确认 `src/FAST_LIO/config/mid360.yaml` 里是 false |
| `/Odometry` 0 Hz，FAST-LIO 进程在但内存一直涨 | FAST-LIO 卡死，日志里有 `IMU and LiDAR not Synced`；常见原因是有两个雷达驱动（`pgrep -af livox_ros_driver2_node` 查看） | web_ops 看门狗约 40 秒内会自动重建；也可手动重建定位链，然后重新设初始位姿 |
| 导航时贴墙走、撞墙 | 局部代价地图没标出障碍（已修复：`/scan` 源的高度下限曾是 0.10，把平面激光全滤掉了）；机身轮廓偏小 | 查 `/local_costmap/costmap` 是否有值为 100 的格子；网页图例打开「代价地图」应能看到墙边的彩色区域 |
| 走廊里走着走着定位丢、日志 `xy jump`/`xy speed` 很多 | FAST-LIO 在窄走廊沿走廊方向打滑 | 已调细 FAST-LIO 体素并让 odom_filter 吸收单帧跳变；仍丢就在走廊里放些有特征的物体（箱子、椅子），或建图时走慢一点 |
| 导航中定位越走越飘、乱走乱撞 | FAST-LIO 发散（`/Odometry` 坐标上千米，`/tmp/localization.log` 大量 `Rejected FAST-LIO odom`）。根因是 UDP 缓冲太小丢点云帧（`grep RcvbufErrors` 见 `/proc/net/snmp`） | 确认 `rmem_max` 已调大（见 §8「DDS 通信」）；重建定位链，重设初始位姿。odom_filter 连续拒收超过 1 秒会停发 odom/TF，让 Nav2 停车，而不是带着冻结的里程计继续开 |
| 换完电池（主控和狗一起开机）SDK 不通 | 运控没起完 bridge 就去连，运控被连「僵死」 | **一般不用管**：web_ops 会等运控跑满 40 s 再连；仍不通会在狗开机 1~2 分钟内自动重启运控一次（狗本来就趴着）。约 3 分钟还不通再跑 `bash scripts/fix_sdk_link.sh` |
| 网页 SDK 未通、`ros2 node list` 为空、各种「收不到数据」 | 进程没带 UDP 配置，共享内存端口被误删 | 确认环境变量（见 §8「DDS 通信」）；重启 `dog-web-console.service`、定位链、Nav2 |
| 启动导航后一直「无 map 坐标系」，AMCL 节点不存在 | Nav2 定位部分加载卡住（`ros2 lifecycle get /amcl` 显示 Node not found） | 关闭导航再启动导航 |
| 设目标点后立刻提示「导航失败」 | Nav2 半启动：`bt_navigator` / `velocity_smoother` 没被激活（启动时 TF 没准备好，导致启动流程中止） | web_ops 会自动补激活；手动：`ros2 lifecycle set /bt_navigator activate` 和 `ros2 lifecycle set /velocity_smoother activate`（定位不丢，狗不动） |
| 狗前方没有激光点、身后一大堆 | 外参俯仰符号反了 | 核对 §8 的 `qy` 为负 |
| 人站在狗头前 / 前左方 0.5 m 内，网页上没有激光点 | near_field_cloud 没起来（FAST-LIO blind 0.5 m 内的点全靠它补） | `ros2 topic hz /near_cloud` 应 10 Hz；没有就重建定位链（`start_quad.sh localization` 会拉起它） |
| 进门 / 转弯时贴门框卡住 | 多半是定位偏（尤其航向偏几度），转弯甩出去 | 用 `scripts/diag/scan_map_align.py` 量残差；>5 cm 或 >1° 就重设初始位姿；还偏再查 AMCL 参数或重建地图 |
| 有人站在路线上：狗停下不走 | 正常：路线被挡先等（1–2 s 一轮），人让开就走；全局路线会尝试绕开 | 一直堵死约半分钟后本次导航失败 |
| 人旁窄缝（~0.8 m）走走停停 | 每边只剩 ~20 cm，几厘米定位误差就会蹭到 | 勇敢模式开着能过，偶尔停 1 s；1 m 以上的缝顺畅。想更稳就关勇敢模式（但 0.8 m 会过不去） |
| `ros2 topic echo` 读不到，但 `ros2 topic hz` 有频率 | `ros2` daemon 坏了（主控重启后常见） | 加 `--no-daemon`（见 §8） |
| 网页地图上下 / 左右镜像 | 磁盘兜底读 PGM 时没做行翻转（已修复） | 刷新页面；确认 `occupancy_map_from_disk` 有行翻转 |

### 自愈机制（web_ops）

- **定位链**：FAST-LIO / stage2 挂了会自动重拉（`heal_nav_deps`）；FAST-LIO 进程在但连续约 40 秒没有 `/Odometry` 输出，也会重建（重建后需重设初始位姿）。启动导航时若发现雷达驱动不止一个或 FAST-LIO 无输出，会先重建定位链。
- **Nav2 半启动**：Nav2 运行超过 90 秒后，每 30 秒检查 `bt_navigator`、`velocity_smoother`，停在 inactive 就自动激活；目标被拒绝时也会立即检查修复。
- **SDK**：
  - **拉 bridge 前一律先等狗端 mc_ctrl 跑满 40 s**（SSH 查运行时长）：运控没起完就握 SDK 会把它连「僵死」。换电池时主控和狗一起开机，以前每次都会踩。
  - **开机快速通道**：狗端运控运行 60 s~15 min（刚开机，狗必然趴着）且 bridge 读不到狗 → 直接重启运控、等 40 s、再拉 bridge。每个运控进程只救一次（`/tmp/web_ops_boot_heal_mc_pid`）。运控已跑 >15 min 的不走这里（狗可能正被遥控站着）。
  - 常规：电量数据 ≥90 s 没更新时，**先用新进程（`--no-daemon`）探测 bridge 是否还能读到狗**：能读到 → 只重启 rosbridge，并让 web_ops 自己退出、由看门狗重新拉起（10 分钟内最多一次），**不碰狗**；读不到 → 重启 bridge，同一小时内第 2、3 次才重启狗端 mc_ctrl，每小时最多 3 次。

### 日志位置

`/tmp/localization.log`（定位链）· `/tmp/nav2.log` · `/tmp/bridge_run.log`（SDK 桥）· `/tmp/webops_new.log`（web_ops）· `/tmp/rosbridge_new.log` · `/tmp/scan_node.log` · `/tmp/mapping.log` · `/tmp/oem700.log`（RTK 驱动）· `/tmp/oem700_map.log` · `/tmp/oem700_arbiter.log` · `/tmp/web_console_watchdog.log`。会话记录 `logs/rtk_session.jsonl`（`scripts/rtk_session_log.py`，gitignore）。ROS 日志目录 `ROS_HOME=/tmp/ros_home`。

### 操作红线

- 不要在命令行里用模糊的 `pkill -f <名字>`：命令自身的 shell 也会被匹配到。要按 PID 杀，或者用 `^` 锚定可执行路径。
- 不要杀 `genisom_bridge`，除非你确定要让狗趴下。
- 不要在狗上手动再起一个 `./mc_ctrl`；改狗端脚本用 `scp` + `sudo install`，不要用 `tee` 加管道密码。
- 同一时间只能有一个 stage2、一个 FAST-LIO；建图和导航互斥。

---

## 11. 已知限制与后续计划

| 项 | 现状 | 计划 |
|---|---|---|
| 正后方盲区 | 雷达低头安装，向后的光束全部朝上，只能扫到天花板；**狗正后方在障碍物高度上看不到** | 导航已禁止倒车；手动遥控后退仍需小心；考虑补后向传感器 |
| 初始定位 | 无 datum：人工设初始位姿 + AMCL（门口残差 ~3.5 cm）。有 datum 且 RTK 固定：开导航后按卫星位置放下，不用设初始位姿。换电池挪狗后同样靠下一次固定解。室外空旷时纯 AMCL 航向会飘、狗原地转 | 建图全程 RTK 固定时，在「保存地图」里直接写出 datum，省掉事后补走。航向在静止时仍不稳，站立方向继续跟里程计 |
| 人旁可通行宽度 | 勇敢模式下 ~0.8 m 能过但会走走停停，≥1 m 顺畅；保守模式下 0.8 m 过不去 | 再窄需要更准的定位（NDT）或更细的局部代价图（2.5 cm，CPU ×4） |
| 动态避障 | 全局规划会绕人；RPP 不做局部绕障，路线被挡时停下等 | 如需「边走边让」，考虑 MPPI（RK3588 上之前带不动，需降采样数） |
| 换电池后 SDK 自动恢复 | 2026-10-02 已实现（等运控 40 s + 开机快速通道），**还没在真实换电池时验证过** | 下次换电池确认 1–2 分钟内 SDK 自己变绿 |
| DDS「收不到」问题 | **已查明并处理**：FastDDS 共享内存端口会被误删，老进程从此收不到新进程数据。已改为只走 UDP（`config/fastdds_udp_only.xml`） | 内核 UDP 缓冲已调到 32 MB（2026-10-01）；FAST-LIO 输入稳定 10 Hz |
| FAST-LIO 卡死 | 已查明常见诱因是双雷达驱动；已加看门狗（`/Odometry` 停发约 40 s 自动重建定位链） | 观察是否还有其他诱因 |
| 开机自启的 rosbridge | 偶尔连得上但不转发数据 | 看门狗增加「有没有实际转发数据」的检查 |

---

## 12. 与旧工程 robot_ws 并存

主控上还有旧工程 `/home/linaro/robot_ws`。本工作区独立部署，不改动它的任何文件。

- **禁止同时运行两套**：两边共用雷达、狗 SDK 端口 43988、网页端口 8080/8090/9090、默认 `ROS_DOMAIN`，会互相抢占。
- 一个终端里只 `source` 一套：`source /home/linaro/steel_coin_nav_ws/install/setup.bash`。
- **`dog-sensing.service`**（开机拉起 robot_ws 的传感链）已于 2026-10-01 **disable**。如果重新启用，启动新栈导航前必须先停掉它，并按 PID 清理它拉起的子进程：这个服务是 oneshot 类型且 `KillMode=process`，`systemctl stop` 不会结束子进程。
- `dog-web-console.service` 现在指向本工作区（看门狗 → 网页三件套），**不要 disable**。用 `systemctl cat dog-web-console.service | grep ExecStart` 可以确认它指向哪套。
