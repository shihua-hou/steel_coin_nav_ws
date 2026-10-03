#!/usr/bin/env python3
"""steel_coin_nav2.launch.py — 「钢镚」四足机器狗一体化导航/建图 launch。

整合自：
  - Go2W-navigate-project 的 isaac_go2_nav2（fastlio_nav2_odom_filter /
    stage2_pointcloud_to_laserscan / external_pose_map_corrector）
  - robot-dog-console（genisom_bridge / lio_tf_bridge / nav2_tools / SUPER_LIO / FAST_LIO）

用法：
  # 建图（Super-LIO + 2D 栅格预览）
  ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=mapping
  # 导航（生产默认 = Go2W 移植：FAST-LIO + odom_filter + stage2 + Nav2）
  ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=nav map:=<map.yaml>
  # 可选：狗里程计 / Super-LIO 里程计
  ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=nav odom_source:=dog map:=<map.yaml>
  ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=nav odom_source:=lio map:=<map.yaml>

坐标系：
  Nav2 侧：map → odom → base_footprint
  FAST-LIO：camera_init → imu(=body)；再经钢镚标定静态 TF → base_link/base_footprint
  雷达外参（钢镚≠Go2 平装）：x=0.25 y=0 z=0.45 pitch≈-19.63° roll≈-0.78°
  （config/lidar_extrinsic.yaml；勿套用 Go2W 默认平装外参）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _share(pkg):
    return get_package_share_directory(pkg)


def _find_launch(pkg, launch_file, subdirs):
    """按候选安装子目录查找 launch 文件（livox 装在 launch_ROS2 而非 launch）。"""
    share = _share(pkg)
    for sub in subdirs:
        cand = os.path.join(share, sub, launch_file)
        if os.path.exists(cand):
            return cand
    return os.path.join(share, subdirs[0], launch_file)


def _inc(pkg, launch_file, args=None, cond=None, subdirs=("launch",)):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_find_launch(pkg, launch_file, subdirs)),
        # Humble 的 IncludeLaunchDescription 不接受 dict（tuple(dict) 只取 key 会炸），
        # 必须传 list of (key, value) 二元组。
        launch_arguments=list((args or {}).items()),
        condition=cond,
    )


def _nav2_with_rtk(context):
    """Nav2 plus OEM700. Fusion (AMCL TF off, arbiter on) only if a datum exists."""
    if LaunchConfiguration("mode").perform(context) != "nav":
        return []
    map_yaml = LaunchConfiguration("map").perform(context)
    rtk_mode = LaunchConfiguration("rtk_fusion").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    params_src = os.path.join(
        _share("isaac_go2_nav2"), "config", "steel_coin_nav2_params.yaml")
    try:
        from oem700_rtk.fusion_launch import fusion_requested, write_nav2_params
        fuse = fusion_requested(map_yaml, rtk_mode)
        params_file = write_nav2_params(params_src, fuse)
    except Exception:
        fuse = False
        params_file = params_src
    actions = [
        Node(
            package="oem700_rtk",
            executable="oem700_driver",
            name="oem700_driver",
            output="screen",
        ),
    ]
    if fuse:
        actions.append(Node(
            package="oem700_rtk",
            executable="rtk_to_map",
            name="rtk_to_map",
            output="screen",
            parameters=[{"map_yaml": map_yaml}],
        ))
        actions.append(Node(
            package="oem700_rtk",
            executable="global_pose_arbiter",
            name="global_pose_arbiter",
            output="screen",
        ))
    actions.append(_inc(
        "nav2_bringup", "bringup_launch.py",
        {
            "params_file": params_file,
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "autostart": "True",
        },
    ))
    return actions


def generate_launch_description():
    # 工作区根探测（本 launch 打包进 isaac_go2_nav2，地图路径按部署目录）
    ws_root = "/home/linaro/steel_coin_nav_ws"
    if not os.path.isdir(ws_root):
        for cand in ("/home/linaro/robot_ws", "/home/linaro/quad_nav_ws"):
            if os.path.isdir(cand):
                ws_root = cand
                break

    mode = LaunchConfiguration("mode")
    odom_source = LaunchConfiguration("odom_source")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    external_pose_correction = LaunchConfiguration("external_pose_correction")

    is_mapping = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    is_nav = IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    # 导航里程计三选一：仅 mode:=nav 时启动。建图默认 odom_source=fastlio，
    # 若这里不限制 mode，会在 mapping 时同时拉起 FAST-LIO + Super-LIO，
    # 停止建图后留下 mode:=mapping launch / 双 LIO，网页无法开导航。
    src_dog = IfCondition(PythonExpression(
        ["'", mode, "' == 'nav' and '", odom_source, "' == 'dog'"]))
    src_lio = IfCondition(PythonExpression(
        ["'", mode, "' == 'nav' and '", odom_source, "' == 'lio'"]))
    src_fastlio = IfCondition(PythonExpression(
        ["'", mode, "' == 'nav' and '", odom_source, "' == 'fastlio'"]))

    # Super-LIO 只在 mapping 或 nav/lio 模式下启动
    start_super_lio = IfCondition(PythonExpression(
        ["('", mode, "' == 'mapping') or ('", mode, "' == 'nav' and '",
         odom_source, "' == 'lio')"]))

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="nav",
                              description="nav | mapping"),
        DeclareLaunchArgument("odom_source", default_value="fastlio",
                              description="fastlio(默认/Go2W) | dog | lio"),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(ws_root, "maps", "map_live.yaml"),
            description="Nav2 地图 yaml（绝对路径）"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "external_pose_correction", default_value="false",
            description="启用 Go2W external_pose_map_corrector（与 AMCL 二选一；"
                        "默认 false 由 AMCL 发 map→odom）"),
        DeclareLaunchArgument(
            "rtk_fusion", default_value="auto",
            description="auto：maps/<图名>.datum.yaml 存在才用 RTK 接管 map→odom；"
                        "false：始终 AMCL；true：有 datum 才接管"),

        # ============ 1) 传感器驱动 ============
        _inc("livox_ros_driver2", "msg_MID360s_launch.py",
             subdirs=("launch_ROS2", "launch")),

        # ============ 2) 底盘桥接（SDK 连接即接管控制权） ============
        _inc("genisom_bridge", "bridge.launch.py"),

        # ============ 3) Super-LIO（建图 / lio 里程计模式） ============
        _inc("super_lio", "Livox_mid360.py",
             {"rviz": "false"}, cond=start_super_lio),

        # ============ 4) 建图模式：2D 栅格预览 ============
        Node(
            package="nav2_tools",
            executable="map_building_node",
            name="map_building_node",
            output="screen",
            condition=is_mapping,
            parameters=[{
                "resolution": 0.05,
                # 与 config/obstacle_height_band.yaml / pcd2pgm / stage2 一致
                "z_min": 0.05, "z_max": 0.85,
                "occ_min": 0.10, "ground_z": 0.08,
                "publish_rate": 2.0, "frame_id": "world",
            }],
        ),

        # ============ 5) Nav 模式：里程计与 TF ============
        # 5a) dog：狗底盘里程计（可选）
        _inc("lio_tf_bridge", "bridge.launch.py",
             {"odom_source": "dog"}, cond=src_dog),
        # 5b) lio：Super-LIO 平面里程计（易漂，仅调试）
        _inc("lio_tf_bridge", "bridge.launch.py",
             {"odom_source": "lio"}, cond=src_lio),
        # 5c) fastlio：Go2W 移植链路（默认）— 节点来自 Go2W，外参按钢镚标定
        _inc("fast_lio", "mapping.launch.py",
             {"config_file": "mid360.yaml", "rviz": "false"},
             cond=src_fastlio),
        # TF 拆成两棵树，避免乱飘：
        #   A) FAST-LIO: camera_init→body；静态 body→base_link（仅给雷达模型/显示）
        #   B) Nav2: odom_filter 发 odom→base_footprint（平面滤波）；AMCL 发 map→odom
        # stage2：先 TF 到 body，再乘钢镚外参，输出 /scan(frame=base_footprint)，
        # 不查 camera_init→base_footprint（那条路会和 B 抢父帧）。
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="body_to_base_link_static",
            output="screen",
            condition=src_fastlio,
            arguments=[
                "-0.084303", "0.006341", "-0.507792",
                "0.006523", "-0.170463", "0.000000", "0.985343",
                "body", "base_link",
            ],
        ),
        Node(
            package="isaac_go2_nav2",
            executable="fastlio_nav2_odom_filter",
            name="fastlio_nav2_odom_filter",
            output="screen",
            condition=src_fastlio,
            parameters=[{
                "input_odom_topic": "/Odometry",
                "output_odom_topic": "/odom_nav",
                "odom_frame": "odom",
                "nav_base_frame": "base_footprint",
                "map_yaml": map_file,
                "initial_x": 0.0, "initial_y": 0.0, "initial_yaw": 0.0,
                "reject_out_of_map": False,
                # 已调平到水平机体系，z 只在 FAST-LIO 发散时才会大
                "max_abs_z": 3.0,
                # body→base_link 外参（与静态 TF 同值）：把斜的 camera_init 调平
                "base_extrinsic_x": -0.084303, "base_extrinsic_y": 0.006341,
                "base_extrinsic_z": -0.507792,
                "base_extrinsic_qx": 0.006523, "base_extrinsic_qy": -0.170463,
                "base_extrinsic_qz": 0.000000, "base_extrinsic_qw": 0.985343,
                "publish_tf": True,
                "stamp_mode": "now",
            }],
        ),

        # ============ 6) Nav 模式：/scan ============
        # 6a) dog/lio：nav_scan_node（按标定 ~19.6° 校平 /lio/cloud_world → /scan）
        Node(
            package="nav2_tools",
            executable="nav_scan_node",
            name="nav_scan_node",
            output="screen",
            condition=IfCondition(PythonExpression(
                ["'", mode, "' == 'nav' and '", odom_source, "' != 'fastlio'"])),
        ),
        # 6b) fastlio（默认）：Go2W stage2；钢镚安装≠Go2：
        #  - 前倾约 19.6°（非平装），垂直 FoV 下翻，实测照不到腿 → 关 self_filter
        #  - 高度带与 pcd2pgm 共用 config/obstacle_height_band.yaml：0.05~0.85m（同 Go2W）
        #  - range_min=0.25、关孤立束滤波（过滤太狠 /scan 只剩十几束，AMCL 定位差）；杂红点由网页显示层再滤
        # 近场补点（C++）：雷达 0.25~0.70 m 原始点 → /near_cloud（stage2 默认订阅并入 /scan）
        Node(
            package="near_field_cloud",
            executable="near_field_cloud_node",
            name="near_field_cloud",
            output="screen",
            condition=src_fastlio,
        ),
        Node(
            package="isaac_go2_nav2",
            executable="stage2_pointcloud_to_laserscan",
            name="stage2_pointcloud_to_laserscan",
            output="screen",
            condition=src_fastlio,
            remappings=[
                ("cloud_in", "/cloud_registered"),
                ("scan", "/scan"),
            ],
            parameters=[{
                "target_frame": "body",
                "output_frame": "base_footprint",
                # body→base_link 外参（与 static body_to_base_link 同一组）。2026-10-01 修正俯仰符号：
                # 雷达低头 19.63°（IMU 重力实测 body「上」=(-0.33,0,0.94)），旧值写成抬头 → 前方激光被高度带滤掉、身后地面成障碍
                "body_extrinsic_x": -0.084303,
                "body_extrinsic_y": 0.006341,
                "body_extrinsic_z": -0.507792,
                "body_extrinsic_qx": 0.006523,
                "body_extrinsic_qy": -0.170463,
                "body_extrinsic_qz": 0.000000,
                "body_extrinsic_qw": 0.985343,
                "use_tf": True,
                "stamp_mode": "tf",  # 戳=最新 odom TF-0.1s，避开 Humble tf2 MessageFilter 死锁
                "min_height": 0.05, "max_height": 0.85,
                "angle_min": -3.14159265359,
                "angle_max": 3.14159265359,
                "angle_increment": 0.00872664626,
                "scan_time": 0.10,
                "range_min": 0.25, "range_max": 12.0,
                "use_inf": True, "inf_epsilon": 1.0,
                "self_filter_enabled": False,
                "self_filter_min_x": -0.45, "self_filter_max_x": 0.45,
                "self_filter_min_y": -0.32, "self_filter_max_y": 0.32,
                # 孤立束默认关（稀疏 /scan 不利于 AMCL）
                "isolate_filter_enabled": False,
                "transform_tolerance": 0.50,
            }],
        ),

        # ============ 7) Nav2（map_server + AMCL + planner/controller） ============
        # 6c) Go2W external_pose_map_corrector（默认休眠 external_pose_correction:=false）
        #  与 AMCL 二选一：都是 map→odom 发布者。AMCL 在跑时勿同时启用，否则 TF 冲突。
        #  若启用：AMCL 应关闭，本节点用 /amcl_pose 或其它 map 系外部位姿做增量修正。
        Node(
            package="isaac_go2_nav2",
            executable="external_pose_map_corrector",
            name="external_pose_map_corrector",
            output="screen",
            condition=IfCondition(external_pose_correction),
            parameters=[{
                "local_odom_topic": "/lio/odom",
                "external_pose_topic": "/amcl_pose",
                "external_pose_type": "pose",
                "external_pose_mode": "map",
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "correction_alpha": 0.2,
                "publish_rate": 20.0,
                "max_abs_roll_pitch": 0.8,
                "max_correction_step": 0.5,
                "max_yaw_correction_step": 0.6,
                "log_period": 2.0,
            }],
        ),
        OpaqueFunction(function=_nav2_with_rtk),

        # ============ 8) RViz（可选） ============
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(rviz),
        ),
    ])
