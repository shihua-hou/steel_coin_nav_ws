# 导航诊断小工具（只读，不会让狗动）

运行前：`source /opt/ros/humble/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/linaro/steel_coin_nav_ws/config/fastdds_udp_only.xml`

| 脚本 | 用途 | 用法 |
|---|---|---|
| `scan_map_align.py` | 定位准不准：把当前 /scan 在静态地图上小范围平移/旋转，找和墙线最贴合的位置，输出 AMCL 还差多少（dx/dy/dθ）。残余 ≤5 cm、≤1° 算准 | 狗静止时：先 `ros2 service call /request_nomotion_update std_srvs/srv/Empty` 几次，再 `python3 scan_map_align.py`。地图路径写在脚本顶部 `MAP` |
| `collision_snapshot.py` | 狗为什么停：盯 /tmp/nav2.log，每次 RPP 报 `detected collision ahead` 就记录狗轮廓 0.3 m 内的致命格（机体系，+x 前 +y 左）到 /tmp/collprobe.log，20 分钟后退出 | `setsid nohup python3 collision_snapshot.py &` 然后让狗走，看 /tmp/collprobe.log |
| `plan_probe.py` | 路线稳不稳：记录每次全局路线变化（长度、前 3 m 最大横向偏移）和狗停下/原地转的时段，写 /tmp/planprobe.log，20 分钟后退出 | `setsid nohup python3 plan_probe.py &` 然后让狗走 |
| `gap_probe.py` | 狗卡住时，局部代价图里正前方 0.35~0.85 m 各横截面的可通行宽度，写 /tmp/gapprobe.log，15 分钟后退出 | 同上 |

2026-10-02 用这几个工具查清：人旁 0.8 m 缝卡住是人贴在机身侧；进门失败是 AMCL 航向偏 6° + 转弯甩出（不是路线问题）。
