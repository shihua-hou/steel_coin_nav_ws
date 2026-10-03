#!/bin/bash
# 「钢镚」一键启动入口
#   bash start_quad.sh              → 导航（默认 Go2W：FAST-LIO + odom_filter + stage2 + Nav2）
#   bash start_quad.sh mapping      → 建图（Super-LIO + 2D 栅格预览）
#   bash start_quad.sh localization → 定位依赖（Go2W，无 Nav2；供网页自愈）
#   bash start_quad.sh dog          → 旧生产链路：狗里程计 + nav_scan(19.6°校平)
#   bash start_quad.sh lio          → Super-LIO 平面里程计 + nav_scan（调试）
#   bash start_quad.sh sensors      → 仅传感器 + SDK 桥
#
# 注意：钢镚 ≠ Unitree Go2。雷达在躯干前倾安装（标定 pitch≈19.6°，非 Go2 平装），
#       Go2W 节点已用本机外参/关腿部自滤波；勿套用 Go2 默认外参。
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/linaro/steel_coin_nav_ws/config/fastdds_udp_only.xml  # FastDDS 只走 UDP（SHM 端口会被误删导致进程互相收不到数据，2026-10-01）
source "$PWD/install/setup.bash"

MODE="${1:-nav}"
case "$MODE" in
  mapping)
    # 与 localization/nav 互斥：双开会抢雷达、双 genisom
    exec 201>/tmp/steel_coin_mapping.lock
    if ! flock -n 201; then
      echo "[start_quad] mapping already running (flock)"
      exit 0
    fi
    ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=mapping
    ;;
  localization)
    # 单实例：重复拉起会叠两套 FAST-LIO/stage2 → TF 断树、odom 非单调、Nav2 秒崩
    # 用 mkdir 锁（勿用 flock：子进程会继承 fd，FAST-LIO 挂掉后锁永远不放）
    LOCKDIR=/tmp/steel_coin_localization.lockdir
    _loc_healthy() {
      pgrep -f 'lib/fast_lio/fastlio_mapping' >/dev/null \
        && pgrep -f 'fastlio_nav2_odom_filter' >/dev/null \
        && [ "$(pgrep -fc 'lib/livox_ros_driver2/livox_ros_driver2_node')" = "1" ]   # 双驱动=不健康
    }
    if ! mkdir "$LOCKDIR" 2>/dev/null; then
      if _loc_healthy; then
        echo "[start_quad] localization already healthy"
        exit 0
      fi
      # 另一个 localization 正在启动（锁主活着且锁 <90s）：链路本来就还没起齐，勿抢锁。
      # 2026-10-01：手动重建与 web_ops 自愈 2s 内先后进来，后者抢锁 → odom_filter/stage2 各叠两份。
      owner=$(cat "$LOCKDIR/pid" 2>/dev/null || true)
      age=$(( $(date +%s) - $(stat -c %Y "$LOCKDIR" 2>/dev/null || echo 0) ))
      # 锁主还没来得及写 pid（mkdir 与写 pid 之间的窗口，实测 2 s 内并发命中）也算「正在启动」
      if [ "$age" -lt 90 ] && { [ -z "$owner" ] || kill -0 "$owner" 2>/dev/null; }; then
        echo "[start_quad] localization starting by pid ${owner:-?} (${age}s ago); skip"
        exit 0
      fi
      echo "[start_quad] localization lock stale (stack unhealthy); reclaiming…"
      me=$$
      for pid in $(pgrep -f 'start_quad.sh localization' 2>/dev/null); do
        [ "$pid" = "$me" ] && continue
        kill -9 "$pid" 2>/dev/null || true
      done
      # 清掉仍占旧 flock fd 的残留（历史问题）
      for pid in $(fuser /tmp/steel_coin_localization.lock 2>/dev/null); do
        cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
        case "$cmd" in
          *genisom_bridge*) continue ;;
          *web_ops_node*) continue ;;
        esac
        kill -9 "$pid" 2>/dev/null || true
      done
      rm -rf "$LOCKDIR" /tmp/steel_coin_localization.lock
      if ! mkdir "$LOCKDIR" 2>/dev/null; then
        echo "[start_quad] localization lock reclaim failed" >&2
        exit 1
      fi
    fi
    trap 'rm -rf /tmp/steel_coin_localization.lockdir' EXIT
    echo $$ > "$LOCKDIR/pid"
    echo $$ > /tmp/loc_shell.pid
    date +%s > /tmp/loc_ready.flag

    # Go2W 定位依赖（无 Nav2）：livox + FAST-LIO + odom_filter + stage2 + 钢镚静态 TF
    # 绝不 pkill genisom_bridge —— 断 SDK 会让狗瞬间趴下。
    # 杀进程用「可执行路径」匹配，避免 pkill -f 误杀含同名字符串的调试 shell。
    _sk() {
      local pat="$1" pid cmd
      for pid in $(pgrep -f "$pat" 2>/dev/null); do
        cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
        case "$cmd" in
          *extglob*|*COMMAND_EXIT_CODE*|*cursorsandbox*|*cursor-agent*) continue ;;
        esac
        kill -9 "$pid" 2>/dev/null || true
      done
    }
    _sk 'steel_coin_nav2.launch.py'
    _sk 'start_quad.sh mapping'
    _sk 'map_building_node'
    _sk 'livox_ros_driver2_node'
    _sk 'lib/fast_lio/fastlio_mapping'
    _sk 'fastlio_nav2_odom_filter'
    _sk 'stage2_pointcloud_to_laserscan'
    _sk 'lib/near_field_cloud/near_field_cloud_node'
    _sk 'nav_scan_node'
    _sk 'super_lio_node'
    _sk 'lib/lio_tf_bridge/lio_tf_bridge'
    _sk 'imu_to_base_link_static'
    _sk 'body_to_base_link_static'
    _sk 'odom_to_camera_init_static'
    _sk 'base_link_to_base_footprint'
    _sk 'msg_MID360s_launch'
    sleep 1

    ros2 launch livox_ros_driver2 msg_MID360s_launch.py &
    sleep 3

    if ! pgrep -f 'lib/genisom_bridge/genisom_bridge' >/dev/null; then
      ros2 launch genisom_bridge bridge.launch.py &
      sleep 2
    fi

    # 仅 body→base_link（显示用）。Nav2 的 odom→base_footprint 由 odom_filter 独占。
    ros2 run tf2_ros static_transform_publisher \
      -0.084303 0.006341 -0.507792 \
      0.006523 -0.170463 0.000000 0.985343 \
      body base_link --ros-args -r __node:=body_to_base_link_static &

    ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false &
    sleep 4
    if ! pgrep -f 'lib/fast_lio/fastlio_mapping' >/dev/null; then
      echo "[start_quad] FAST-LIO failed to start; see localization.log" >&2
    else
      echo "[start_quad] FAST-LIO up"
    fi

    # 近场补点（C++）：FAST-LIO blind 丢掉的雷达 0.25~0.70 m 点 → /near_cloud → stage2 并入 /scan
    # （狗头前左方站人、贴墙时原来看不见，2026-10-02）
    ros2 run near_field_cloud near_field_cloud_node --ros-args -r __node:=near_field_cloud &

    ros2 run isaac_go2_nav2 fastlio_nav2_odom_filter --ros-args \
      -r __node:=fastlio_nav2_odom_filter \
      -p input_odom_topic:=/Odometry \
      -p output_odom_topic:=/odom_nav \
      -p odom_frame:=odom \
      -p nav_base_frame:=base_footprint \
      -p reject_out_of_map:=false \
      -p max_abs_z:=3.0 \
      -p base_extrinsic_x:=-0.084303 -p base_extrinsic_y:=0.006341 \
      -p base_extrinsic_z:=-0.507792 \
      -p base_extrinsic_qx:=0.006523 -p base_extrinsic_qy:=-0.170463 \
      -p base_extrinsic_qz:=0.000000 -p base_extrinsic_qw:=0.985343 \
      -p publish_tf:=true \
      -p stamp_mode:=now &
    sleep 1

    # stage2：TF→body 再乘外参→base_footprint；高度带 0.05~0.85；关腿滤
    # 外参 2026-10-01 修正俯仰符号（雷达低头 19.63°；旧值是抬头，前方无激光、身后一堆）
    ros2 run isaac_go2_nav2 stage2_pointcloud_to_laserscan --ros-args \
      -r __node:=stage2_pointcloud_to_laserscan \
      -r cloud_in:=/cloud_registered \
      -r scan:=/scan \
      -p target_frame:=body \
      -p output_frame:=base_footprint \
      -p body_extrinsic_x:=-0.084303 -p body_extrinsic_y:=0.006341 \
      -p body_extrinsic_z:=-0.507792 \
      -p body_extrinsic_qx:=0.006523 -p body_extrinsic_qy:=-0.170463 \
      -p body_extrinsic_qz:=0.000000 -p body_extrinsic_qw:=0.985343 \
      -p use_tf:=true \
      -p stamp_mode:=tf -p stamp_tf_frame:=odom -p stamp_tf_lag:=0.10 \
      -p min_height:=0.05 -p max_height:=0.85 \
      -p angle_increment:=0.00872664626 \
      -p range_min:=0.25 -p range_max:=12.0 \
      -p self_filter_enabled:=false \
      -p isolate_filter_enabled:=false \
      -p transform_tolerance:=0.50 &

    # OEM700：板子自己走 4G NTRIP，这里只读 NMEA。不发 map→odom（那要等导航且有 datum）。
    if ! pgrep -f 'lib/oem700_rtk/oem700_driver' >/dev/null; then
      ros2 run oem700_rtk oem700_driver --ros-args -r __node:=oem700_driver &
    fi

    wait
    ;;
  dog)
    ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=nav odom_source:=dog
    ;;
  lio)
    ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=nav odom_source:=lio
    ;;
  fastlio|nav|*)
    ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=nav odom_source:=fastlio
    ;;
  sensors)
    if ! pgrep -f 'lib/oem700_rtk/oem700_driver' >/dev/null; then
      ros2 run oem700_rtk oem700_driver --ros-args -r __node:=oem700_driver &
    fi
    ros2 launch livox_ros_driver2 msg_MID360s_launch.py &
    sleep 3
    ros2 launch genisom_bridge bridge.launch.py
    ;;
esac
