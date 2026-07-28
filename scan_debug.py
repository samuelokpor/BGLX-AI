import math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=1)
THRESHOLD = 1.0

class ScanDebug(Node):
    def __init__(self):
        super().__init__('scan_debug')
        self.create_subscription(LaserScan, '/etrike/scan', self.cb, QOS)
        self.done = False

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        close = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if r < THRESHOLD:
                ang = math.degrees(msg.angle_min + i * msg.angle_increment)
                close.append((i, (ang + 180) % 360 - 180, r))
        total = len(msg.ranges)
        print("\n%d rays, %d nearer than %.1fm (%.1f%%)\n"
              % (total, len(close), THRESHOLD, 100.0*len(close)/total))
        if not close:
            print("No close returns at all.")
            return
        runs, cur = [], [close[0]]
        for prev, item in zip(close, close[1:]):
            if item[0] - prev[0] <= 2:
                cur.append(item)
            else:
                runs.append(cur); cur = [item]
        runs.append(cur)
        print("%d contiguous group(s):\n" % len(runs))
        for run in sorted(runs, key=lambda r: min(x[2] for x in r)):
            a0, a1 = run[0][1], run[-1][1]
            rmin = min(x[2] for x in run); rmax = max(x[2] for x in run)
            width = abs(a1 - a0)
            kind = ("ISOLATED - self-hit or thin geometry" if len(run) <= 3
                    else "narrow - thin object" if width < 5
                    else "broad - real surface")
            print("  %4d rays  bearing %7.1f to %7.1f deg (%5.1f wide)  "
                  "range %.2f-%.2fm   %s"
                  % (len(run), a0, a1, width, rmin, rmax, kind))
        print("\n0 = ahead, +90 = left, -90 = right, 180 = behind.")

rclpy.init()
n = ScanDebug()
while rclpy.ok() and not n.done:
    rclpy.spin_once(n, timeout_sec=0.5)
rclpy.shutdown()
