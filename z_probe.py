#!/usr/bin/env python3
"""Z-axis probe: steps the Z slide up/down and prints commanded vs actual.
Run while the simulation is READY (Start Simulation), with nothing else moving.
"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ["z_axis_joint", "tilt_joint", "left_cup_joint", "right_cup_joint", "reclamp_joint"]
# (label, z command in metres, seconds to hold)
# gentle steps, never near the -100 mm limit
STEPS = [("zero", 0.0, 3.0), ("down 20", -0.02, 4.0), ("down 40", -0.04, 4.0), ("down 60", -0.06, 5.0),
         ("up to 40", -0.04, 5.0), ("up to 20", -0.02, 5.0), ("zero", 0.0, 5.0)]


class Probe(Node):
    def __init__(self):
        super().__init__("z_probe")
        self.pub = self.create_publisher(Float64MultiArray, "/axis_position_controller/commands", 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.z = None
        self.vz = None
        self.n = 0
        self.names = None

    def cb(self, m):
        self.n += 1
        self.names = list(m.name)
        if "z_axis_joint" in m.name:
            i = m.name.index("z_axis_joint")
            self.z = m.position[i]
            self.vz = m.velocity[i] if len(m.velocity) > i else float("nan")

    def send(self, z):
        msg = Float64MultiArray()
        msg.data = [float(z), 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    p = Probe()
    t_wait = time.time()
    while p.z is None and time.time() - t_wait < 10:
        rclpy.spin_once(p, timeout_sec=0.1)
    if p.z is None:
        print("NO /joint_states received - is the simulation running (READY)?")
        return
    print("joints seen:", p.names)
    try:
        print("publishers on /joint_states: %d   subscribers on commands: %d  (each should be 1)" % (
            p.count_publishers("/joint_states"), p.count_subscribers("/axis_position_controller/commands")))
    except Exception as exc:
        print("count failed:", exc)
    print("start z = %.4f m" % p.z)
    print("%-12s %8s %9s %9s %9s %7s" % ("step", "t(s)", "cmd(mm)", "act(mm)", "vel", "msgs/s"))
    for label, cmd, hold in STEPS:
        t0 = time.time()
        z_before = p.z
        n0, tn = p.n, time.time()
        next_print = 0.0
        while time.time() - t0 < hold:
            p.send(cmd)
            rclpy.spin_once(p, timeout_sec=0.02)
            t = time.time() - t0
            if t >= next_print:
                rate = (p.n - n0) / max(1e-6, time.time() - tn)
                print("%-12s %8.1f %9.1f %9.1f %9.4f %7.0f" % (label, t, cmd * 1000, p.z * 1000, p.vz, rate))
                next_print += 1.0
        print("   -> %s: moved %.1f mm (commanded change from previous target)" % (label, (p.z - z_before) * 1000))
    print("DONE. Paste everything above.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
