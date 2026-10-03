#!/bin/bash
exec > /tmp/tf_dump_run.out 2>&1
set -x
sleep 1
source /opt/ros/humble/setup.bash
source /home/linaro/robot_ws/install/setup.bash
echo reloc=$!

# ensure nav_scan
if ! pgrep -f nav_scan_node.py >/dev/null; then
  bash /home/linaro/robot_ws/start_scan_node.sh
fi
sleep 2

python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry

rclpy.init()
n = Node('tf_dump')
buf = Buffer()
lis = TransformListener(buf, n)
end = time.time() + 5
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
for a,b in [('odom','base_link'),('world','imu'),('odom','base_footprint'),('world','base_link'),('map','odom'),('odom','world')]:
    try:
        t = buf.lookup_transform(a,b,Time())
        tr=t.transform.translation
        print(f'OK {a}->{b} t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f})')
    except Exception as e:
        print(f'FAIL {a}->{b}: {type(e).__name__}: {e}')
state={'cloud':0,'scan':0,'odom':0}
n.create_subscription(PointCloud2,'/lio/cloud_world',lambda _: state.__setitem__('cloud', state['cloud']+1),10)
n.create_subscription(LaserScan,'/scan',lambda _: state.__setitem__('scan', state['scan']+1),10)
n.create_subscription(Odometry,'/lio/odom',lambda _: state.__setitem__('odom', state['odom']+1),10)
end=time.time()+4
while time.time()<end:
    rclpy.spin_once(n, timeout_sec=0.2)
print('counts', state)
# list nodes briefly
print('nav_scan log:'); 
open('/tmp/tf_dump.txt','w').write(open('/tmp/tf_dump_run.out').read() if False else '')
n.destroy_node(); rclpy.shutdown()
PY

echo '==== scan log ===='
cat /tmp/scan_node.log
echo '==== procs ===='
pgrep -af 'nav_scan|lio_tf|super_lio|auto_reloc' | grep -v grep | grep -v sandbox
echo DONE
