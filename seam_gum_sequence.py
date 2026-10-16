#!/usr/bin/env python3
# Seam-following sequence WITH gum bead.
# The nozzle stays still; the ball is spun (cups) and tilted so every point of the seam
# route passes under the nozzle. Amber gum beads appear on the ball right where the nozzle
# is, and stay attached to the ball (they spin and tilt with it).
#
# usage: python3 seam_gum_sequence.py [time_scale] [edges] [--no-gum] [--clear]
#   time_scale : 1.0 = normal speed; use 2.5 when Gazebo shows ~40% real-time
#   edges      : seam edges to trace (default 50 = whole closed loop, ~4.5 m; 12 = short demo)
#   --no-gum   : motion only
#   --clear    : just remove all gum from the ball and exit
#   --parent=model::link : override which Gazebo link the gum is attached to
import sys, math, time, json, subprocess, threading
from pathlib import Path

DRY = "--dry" in sys.argv
GUM = "--no-gum" not in sys.argv
CLEAR_ONLY = "--clear" in sys.argv
args = [a for a in sys.argv[1:] if not a.startswith("--")]
_parent_override = [a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--parent=")]
SCALE = float(args[0]) if len(args) > 0 else 1.0
NEDGES = int(args[1]) if len(args) > 1 else 10 ** 6

HZ = 50
SPEED = 0.08          # nozzle speed along the seam, metres per second (before time_scale)
Z_UP, Z_WORK = 0.05, -0.12

# ---- gum bead settings (Gazebo marker attached to the link that carries the ball)
PARENT = "football_gumming_machine::left_cup"   # model::link that carries the ball
BALL_X = 0.25          # ball centre in that link's frame (x)
BEAD_RADIUS = 0.2215   # bead centre distance from ball centre (ball radius is 0.22)
BEAD_SIZE = 0.010      # bead diameter, metres
BATCH_SECONDS = 0.4    # how often new beads are pushed to Gazebo
if _parent_override:
    PARENT = _parent_override[0]     # e.g. --parent=football_gumming_machine::left_cup

# ---------------------------------------------------------------- gum markers
def marker_text(bead_id, p):
    x, y, z = BALL_X + BEAD_RADIUS * p[0], BEAD_RADIUS * p[1], BEAD_RADIUS * p[2]
    return (f'marker {{ ns: "gum" id: {bead_id} action: ADD_MODIFY type: SPHERE '
            f'parent: "{PARENT}" '
            f'pose {{ position {{ x: {x:.5f} y: {y:.5f} z: {z:.5f} }} }} '
            f'scale {{ x: {BEAD_SIZE} y: {BEAD_SIZE} z: {BEAD_SIZE} }} '
            f'material {{ ambient {{ r: 0.95 g: 0.70 b: 0.05 a: 1 }} '
            f'diffuse {{ r: 0.95 g: 0.70 b: 0.05 a: 1 }} '
            f'emissive {{ r: 0.30 g: 0.20 b: 0.0 a: 1 }} }} }}')

CLEAR_TEXT = 'marker { ns: "gum" action: DELETE_ALL }'
sent_markers = []      # (dry run) every request text

def call_markers(text):
    """Send one Marker_V request to Gazebo. Returns True on success."""
    if DRY:
        sent_markers.append(text)
        return True
    try:
        out = subprocess.run(
            ["gz", "service", "-s", "/marker_array", "--reqtype", "gz.msgs.Marker_V",
             "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", text],
            capture_output=True, text=True, timeout=15)
        return out.returncode == 0 and "true" in out.stdout.lower()
    except Exception:
        return False

class GumSender(threading.Thread):
    """Pushes beads to Gazebo in batches so the motion loop never waits for the gz tool."""
    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.pending = []
        self.done = False
        self.ok = True
        self.count = 0

    def add(self, bead_id, p):
        with self.lock:
            self.pending.append(marker_text(bead_id, p))

    def run(self):
        while True:
            with self.lock:
                batch, self.pending = self.pending[:60], self.pending[60:]
                finished = self.done and not self.pending
            if batch:
                if call_markers(" ".join(batch)):
                    self.count += len(batch)
                else:
                    self.ok = False
            elif finished:
                return
            if not DRY:
                time.sleep(BATCH_SECONDS)
            elif not batch:
                time.sleep(0.001)

if CLEAR_ONLY:
    print("Gum cleared" if call_markers(CLEAR_TEXT) else "Could not reach Gazebo markers (is Gazebo open?)")
    sys.exit(0)

# ---------------------------------------------------------------- route + kinematics
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

# sample the route (about 1 cm apart) and convert every sample to joint values
STEP = 0.01 / R
pts = []
for a, b in zip(V[:NEDGES], V[1:NEDGES + 1]):
    w = math.acos(max(-1.0, min(1.0, dot(a, b))))
    n = max(2, int(w / STEP))
    pts += [slerp(a, b, k / n) for k in range(n)]
pts.append(V[NEDGES])

tilts, spins, prev = [], [], 0.0     # first sample is reached from the home pose (spin 0)
for p in pts:
    t, s = ik(p, prev)
    prev = s
    tilts.append(t)
    spins.append(s)
LENGTH = sum(R * math.acos(max(-1.0, min(1.0, dot(a, b)))) for a, b in zip(pts[:-1], pts[1:]))
TRACE_SECS = LENGTH / SPEED

# ---------------------------------------------------------------- publishing
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

gum = None
bead_log = []            # (sample index, command index when emitted)

def trace():
    N = len(pts) - 1
    nxt = [0]
    def fn(u):
        f = u * N
        i = min(int(f), N - 1)
        r = f - i
        send(Z_WORK, tilts[i] + (tilts[i + 1] - tilts[i]) * r,
             spins[i] + (spins[i + 1] - spins[i]) * r)
        if gum is not None:
            while nxt[0] <= f and nxt[0] <= N:       # nozzle has reached sample nxt -> drop a bead
                gum.add(nxt[0] + 1, pts[nxt[0]])
                bead_log.append((nxt[0], len(sent)))
                nxt[0] += 1
    run(TRACE_SECS, fn)

# ---------------------------------------------------------------- the show
if not DRY:
    time.sleep(1.0)
print(f"Seam route: {NEDGES} edges, {LENGTH:.2f} m, about {TRACE_SECS * SCALE:.0f} s of tracing")
if GUM:
    if call_markers(CLEAR_TEXT):
        print("Gum: ready (old gum cleared)")
        gum = GumSender()
        gum.start()
    else:
        print("Gum: could not reach Gazebo markers (Gazebo GUI open? run `gz service -l | grep marker`). "
              "Continuing without gum.")
move([Z_UP, 0.0, 0.0], 2, "Home")
move([Z_UP, tilts[0], spins[0]], 5, "Position ball so the first seam point is under the nozzle")
move([Z_WORK, tilts[0], spins[0]], 3, "Lower nozzle to working gap")
mark = len(sent)
print("Tracing seams: cups spin + cradle tilts, gum bead follows the nozzle")
trace()
traced = sent[mark:]
if gum is not None:
    gum.done = True
    gum.join(timeout=60)
    print(f"Gum: {gum.count} bead markers placed" + ("" if gum.ok else " (some batches failed)"))
move([Z_UP, tilts[-1], spins[-1]], 3, "Raise nozzle")
move([Z_UP, 0.0, 0.0], 8, "Return home")
print("Done  (remove the gum with:  python3 seam_gum_sequence.py --clear)")

# ---------------------------------------------------------------- self-checks (dry run only)
if DRY:
    worst = 0.0
    for z, t, sp, _ in traced:
        x, y, zz = -math.sin(t), 0.0, math.cos(t)          # world top as seen from the ball frame
        q = (x, math.cos(sp) * y + math.sin(sp) * zz, -math.sin(sp) * y + math.cos(sp) * zz)
        worst = max(worst, min(math.acos(max(-1.0, min(1.0, dot(q, p)))) for p in pts))
    print(f"DRY RUN: {len(sent)} commands; nozzle stays within {worst * R * 1000:.2f} mm of a route sample")
    if GUM:
        beads = [m for txt in sent_markers for m in txt.split("marker {")[1:]]
        print(f"DRY RUN: {len(beads)} bead markers in {len(sent_markers)} requests (incl. the clear request)")
        # each bead was emitted while the nozzle was within 1.5 cm of its position (nozzle point == route)
        bad = 0
        for k, cmd_idx in bead_log:
            z, t, sp, _ = sent[cmd_idx - 1]
            x, y, zz = -math.sin(t), 0.0, math.cos(t)
            q = (x, math.cos(sp) * y + math.sin(sp) * zz, -math.sin(sp) * y + math.cos(sp) * zz)
            if math.acos(max(-1.0, min(1.0, dot(q, pts[k])))) * R > 0.015:
                bad += 1
        print(f"DRY RUN: beads dropped away from the nozzle: {bad}")
else:
    node.destroy_node()
    rclpy.shutdown()
