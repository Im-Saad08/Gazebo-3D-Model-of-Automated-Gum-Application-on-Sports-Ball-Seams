#!/usr/bin/env python3
# usage: python3 sequence.py [time_scale]   (use 3 if Gazebo shows ~30% real-time)
import sys, math, time
import rclpy
from std_msgs.msg import Float64MultiArray

SCALE = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
HZ = 50
Z_UP, Z_WORK, TILT2 = 0.05, -0.12, 0.5
TWO_PI = 2 * math.pi

rclpy.init()
node = rclpy.create_node('gum_sequence')
pub = node.create_publisher(Float64MultiArray, '/axis_position_controller/commands', 10)
state = [Z_UP, 0.0, 0.0, 0.0]   # [z, tilt, left_cup, right_cup]

def send():
    m = Float64MultiArray(); m.data = list(state); pub.publish(m)

def move(z, tilt, spin, secs, linear=False, label=""):
    if label: print(label)
    target = [z, tilt, spin, spin]
    start = list(state)
    n = max(1, int(secs * SCALE * HZ))
    for i in range(1, n + 1):
        s = i / n
        if not linear: s = s * s * (3 - 2 * s)
        for k in range(4):
            state[k] = start[k] + (target[k] - start[k]) * s
        send(); time.sleep(1.0 / HZ)

time.sleep(1.0)
move(Z_UP, 0, 0, 2, label="Home")
move(Z_WORK, 0, 0, 4, label="Lower nozzle to working gap")
move(Z_WORK, 0, TWO_PI, 10, linear=True, label="Seam 1: cups rotate ball, nozzle fixed")
move(Z_UP, 0, TWO_PI, 3, label="Raise nozzle")
move(Z_UP, TILT2, TWO_PI, 4, label="Tilt ball support to next seam")
move(Z_WORK, TILT2, TWO_PI, 4, label="Lower nozzle")
move(Z_WORK, TILT2, 2 * TWO_PI, 10, linear=True, label="Seam 2: cups rotate ball")
move(Z_UP, TILT2, 2 * TWO_PI, 3, label="Raise nozzle")
move(Z_UP, 0, 0, 10, label="Return home")
print("Done")
node.destroy_node(); rclpy.shutdown()
