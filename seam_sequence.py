#!/usr/bin/env python3
# Seam-following sequence: the nozzle stays still; the ball is spun (cups) and tilted
# so that every point of the seam route passes under the nozzle.
#
# usage: python3 seam_sequence.py [time_scale] [edges]
#   time_scale : 1.0 = normal speed, 1.3 = 30% slower (match Gazebo real-time %)
#   edges      : how many seam edges to trace (default 50 = the whole closed loop;
#                use 12 for a short ~15 s demo)
import sys, math, time, json
from pathlib import Path

DRY = "--dry" in sys.argv
args = [a for a in sys.argv[1:] if not a.startswith("--")]
SCALE = float(args[0]) if len(args) > 0 else 1.0
NEDGES = int(args[1]) if len(args) > 1 else 10 ** 6

HZ = 50
SPEED = 0.08          # nozzle speed along the seam, metres per second (before time_scale)
Z_UP, Z_WORK = 0.05, -0.12

route = json.loads((Path(__file__).resolve().parent / "seam_path.json").read_text())
R = route["radius"]
V = [tuple(v) for v in route["vertices"]]
NEDGES = min(NEDGES, len(V) - 1)

def dot(a, b):
    return sum(x * y for x, y in zip(a, b))

def slerp(a, b, t):
    w = math.acos(max(-1.0, min(1.0, dot(a, b))))
    s = math.sin(w)
    return tuple((math.sin((1 - t) * w) * x + math.sin(t * w) * y) / s for x, y in zip(a, b))

def ik(p, prev_spin):
    """Tilt and spin that bring ball-frame point p under the nozzle (world top of ball)."""
    tilt = -math.asin(max(-1.0, min(1.0, p[0])))
    spin = math.pi / 2 - math.atan2(p[2], p[1])
    while spin - prev_spin > math.pi:
        spin -= 2 * math.pi
    while spin - prev_spin < -math.pi:
        spin += 2 * math.pi
    return tilt, spin

# --- sample the route (about 1 cm apart) and convert every sample to joint values
STEP = 0.01 / R
pts = []
for a, b in zip(V[:NEDGES], V[1:NEDGES + 1]):
    w = math.acos(max(-1.0, min(1.0, dot(a, b))))
    n = max(2, int(w / STEP))
    pts += [slerp(a, b, k / n) for k in range(n)]
pts.append(V[NEDGES])

# first sample is reached from the home pose (spin 0), so unwrap relative to 0
tilts, spins, prev = [], [], 0.0
for p in pts:
    t, s = ik(p, prev)
    prev = s
    tilts.append(t)
    spins.append(s)
LENGTH = sum(R * math.acos(max(-1.0, min(1.0, dot(a, b)))) for a, b in zip(pts[:-1], pts[1:]))
TRACE_SECS = LENGTH / SPEED

# --- publishing
sent = []
if DRY:
    def publish(state):
        sent.append(list(state))
else:
    import rclpy
    from std_msgs.msg import Float64MultiArray
    rclpy.init()
    node = rclpy.create_node("seam_sequence")
    pub = node.create_publisher(Float64MultiArray, "/axis_position_controller/commands", 10)
    def publish(state):
        m = Float64MultiArray()
        m.data = list(state)
        pub.publish(m)

state = [Z_UP, 0.0, 0.0, 0.0]            # [z, tilt, left_cup, right_cup]

def send(z, tilt, spin):
    state[:] = [z, tilt, spin, spin]
    publish(state)

def run(seconds, fn):
    """Call fn(u) for u in 0..1 over `seconds` (scaled), at HZ."""
    seconds *= SCALE
    n = max(1, int(seconds * HZ))
    t0 = time.monotonic()
    for i in range(1, n + 1):
        fn(i / n)
        if not DRY:
            time.sleep(max(0.0, t0 + i / HZ - time.monotonic()))

def ease(u):
    return u * u * (3 - 2 * u)

def move(target, secs, label):
    print(label)
    start = list(state[:3])
    def fn(u):
        s = ease(u)
        send(*[start[k] + (target[k] - start[k]) * s for k in range(3)])
    run(secs, fn)

def trace():
    N = len(pts) - 1
    def fn(u):
        f = u * N
        i = min(int(f), N - 1)
        r = f - i
        send(Z_WORK, tilts[i] + (tilts[i + 1] - tilts[i]) * r,
             spins[i] + (spins[i + 1] - spins[i]) * r)
    run(TRACE_SECS, fn)

if not DRY:
    time.sleep(1.0)
print(f"Seam route: {NEDGES} edges, {LENGTH:.2f} m, about {TRACE_SECS * SCALE:.0f} s of tracing")
move([Z_UP, 0.0, 0.0], 2, "Home")
move([Z_UP, tilts[0], spins[0]], 5, "Position ball so the first seam point is under the nozzle")
move([Z_WORK, tilts[0], spins[0]], 3, "Lower nozzle to working gap")
mark = len(sent)
print("Tracing seams: cups spin + cradle tilts so the nozzle follows the panel edges")
trace()
traced = sent[mark:]
move([Z_UP, tilts[-1], spins[-1]], 3, "Raise nozzle")
move([Z_UP, 0.0, 0.0], 8, "Return home")
print("Done")

if DRY:
    # check: the ball-frame point under the nozzle must lie on the route
    worst = 0.0
    ref = pts
    for z, t, sp, _ in traced:
        # world top (0,0,1) seen from the ball frame: Rx(-spin) applied to Ry(-tilt)(0,0,1) = (-sin t, 0, cos t)
        x, y, zz = -math.sin(t), 0.0, math.cos(t)
        q = (x, math.cos(sp) * y + math.sin(sp) * zz, -math.sin(sp) * y + math.cos(sp) * zz)
        d = min(math.acos(max(-1.0, min(1.0, dot(q, p)))) for p in ref)
        worst = max(worst, d)
    print(f"DRY RUN: {len(sent)} commands, worst nozzle-to-route gap {worst * R * 1000:.3f} mm (route sampled every 10 mm)")
    print(f"tilt range {min(x[1] for x in sent):.3f}..{max(x[1] for x in sent):.3f} rad, "
          f"spin range {min(x[2] for x in sent):.3f}..{max(x[2] for x in sent):.3f} rad, "
          f"z range {min(x[0] for x in sent):.3f}..{max(x[0] for x in sent):.3f}")
    jt = max(abs(a[1] - b[1]) for a, b in zip(sent[:-1], sent[1:])) * HZ
    js = max(abs(a[2] - b[2]) for a, b in zip(sent[:-1], sent[1:])) * HZ
    print(f"max tilt rate {jt:.2f} rad/s, max spin rate {js:.2f} rad/s (at time_scale {SCALE})")
else:
    node.destroy_node()
    rclpy.shutdown()
