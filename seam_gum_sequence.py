#!/usr/bin/env python3
# Seam-following sequence WITH gum bead.
# The nozzle stays still; the ball is spun (cups) and tilted so every point of the seam
# route passes under the nozzle. Amber gum beads appear on the ball right where the nozzle
# is, and stay attached to the ball (they spin and tilt with it).
#
# usage: python3 seam_gum_sequence.py [time_scale] [edges] [--no-gum] [--clear] [--route=FILE]
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
_route_arg = [a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--route=")]
route_file = Path(_route_arg[0]) if _route_arg else Path(__file__).resolve().parent / "seam_path.json"
route = json.loads(route_file.read_text())
R = route["radius"]
K = R / 0.22                       # all constants above were tuned for a 0.22 m ball
BALL_X *= K; BEAD_RADIUS *= K; BEAD_SIZE *= K; SPEED *= K; Z_UP *= K; Z_WORK *= K
RECLAMP = bool(route.get("reclamp"))      # ball hangs on a re-clamp joint -> controller has 5 joints
NJ = 5 if RECLAMP else 4
if RECLAMP:
    BALL_X = 0.0                                  # beads are placed in the football's own frame
    PARENT = _parent_override[0] if _parent_override else "football_gumming_machine::football"

RAW_PASSES = route.get("passes") or [{"name": "Clamp 1", "reclamp_angle": 0.0, "vertices": route["vertices"],
                                      "cup_vertices": route["vertices"], "gum": route.get("gum")}]
if not RECLAMP:
    RAW_PASSES = RAW_PASSES[:1]
if NEDGES < len(RAW_PASSES[0]["vertices"]) - 1:
    RAW_PASSES = RAW_PASSES[:1]            # short demo: first clamp only

def dot(a, b):
    return sum(x * y for x, y in zip(a, b))

def ang(a, b):
    return math.acos(max(-1.0, min(1.0, dot(a, b))))

def slerp(a, b, t):
    w = ang(a, b)
    s = math.sin(w)
    return tuple((math.sin((1 - t) * w) * x + math.sin(t * w) * y) / s for x, y in zip(a, b))

def ik(p, prev_spin):
    """Tilt and spin that bring cup-frame point p under the nozzle (world top of ball)."""
    tilt = -math.asin(max(-1.0, min(1.0, p[0])))
    spin = math.pi / 2 - math.atan2(p[2], p[1])
    while spin - prev_spin > math.pi:
        spin -= 2 * math.pi
    while spin - prev_spin < -math.pi:
        spin += 2 * math.pi
    return tilt, spin

STEP = 0.01 * K / R                # samples about 1 cm (scaled) apart

def prepare(ps):
    V = [tuple(v) for v in ps["vertices"]]
    C = [tuple(v) for v in ps.get("cup_vertices", ps["vertices"])]
    G = ps.get("gum") or [True] * (len(V) - 1)
    ne = min(NEDGES, len(V) - 1)
    pts, cpts, gfl = [], [], []
    for e in range(ne):
        n = max(2, int(ang(V[e], V[e + 1]) / STEP))
        for k in range(n):
            pts.append(slerp(V[e], V[e + 1], k / n))      # football frame (where the bead sticks)
            cpts.append(slerp(C[e], C[e + 1], k / n))     # cup frame (what the nozzle must reach)
            gfl.append(G[e])
    pts.append(V[ne]); cpts.append(C[ne]); gfl.append(G[ne - 1])
    tilts, spins, prev = [], [], 0.0
    for p in cpts:
        t, s = ik(p, prev)
        prev = s
        tilts.append(t); spins.append(s)
    length = sum(R * ang(a, b) for a, b in zip(pts[:-1], pts[1:]))
    return {"name": ps.get("name", "pass"), "angle": ps.get("reclamp_angle", 0.0), "pts": pts, "cpts": cpts,
            "gum": gfl, "tilts": tilts, "spins": spins, "length": length}

PASSES = [prepare(ps) for ps in RAW_PASSES]

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
        sent.append(list(state))
        m = Float64MultiArray()
        m.data = list(state)
        pub.publish(m)

state = [Z_UP, 0.0, 0.0, 0.0] + ([0.0] if RECLAMP else [])   # [z, tilt, left_cup, right_cup(, reclamp)]

def send(z, tilt, spin):
    state[0:4] = [z, tilt, spin, spin]
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

def reclamp_to(angle, secs, label):
    print(label)
    start = state[4]
    def fn(u):
        state[4] = start + (angle - start) * ease(u)
        publish(state)
    run(secs, fn)

gum = None
bead_log = []            # (pass index, sample index, command index when emitted)
bead_id = [0]

def trace(pi, P):
    N = len(P["pts"]) - 1
    nxt = [0]
    tilts, spins = P["tilts"], P["spins"]
    def fn(u):
        f = u * N
        i = min(int(f), N - 1)
        r = f - i
        send(Z_WORK, tilts[i] + (tilts[i + 1] - tilts[i]) * r,
             spins[i] + (spins[i + 1] - spins[i]) * r)
        while nxt[0] <= f and nxt[0] <= N:               # nozzle has reached sample nxt
            if gum is not None and P["gum"][nxt[0]]:     # valve open only on glue edges
                bead_id[0] += 1
                gum.add(bead_id[0], P["pts"][nxt[0]])
                bead_log.append((pi, nxt[0], len(sent)))
            nxt[0] += 1
    run(P["length"] / SPEED, fn)

# ---------------------------------------------------------------- the show
if not DRY:
    time.sleep(1.0)
for P in PASSES:
    print(f"{P['name']}: {P['length']:.2f} m of nozzle travel, about {P['length'] / SPEED * SCALE:.0f} s")
if GUM:
    if call_markers(CLEAR_TEXT):
        print("Gum: ready (old gum cleared)")
        gum = GumSender()
        gum.start()
    else:
        print("Gum: could not reach Gazebo markers (Gazebo GUI open? run `gz service -l | grep marker`). "
              "Continuing without gum.")
move([Z_UP, 0.0, 0.0], 2, "Home")
marks, ends = [], []
for pi, P in enumerate(PASSES):
    if pi > 0:
        move([Z_UP, PASSES[pi - 1]["tilts"][-1], PASSES[pi - 1]["spins"][-1]], 3, "Raise nozzle")
        move([Z_UP, 0.0, 0.0], 6, "Return cups to centre")
        reclamp_to(P["angle"], 8, f"RE-CLAMP: release the ball, turn it {math.degrees(P['angle']):.0f} deg, grip the other panel pair")
    print(f"{P['name']}: position ball so the first seam point is under the nozzle")
    move([Z_UP, P["tilts"][0], P["spins"][0]], 5, "Position ball")
    move([Z_WORK, P["tilts"][0], P["spins"][0]], 3, "Lower nozzle to working gap")
    marks.append(len(sent))
    print("Tracing seams: cups spin + cradle tilts, gum only on glue edges, valve closed while travelling")
    trace(pi, P)
    ends.append(len(sent))
if gum is not None:
    gum.done = True
    gum.join(timeout=60)
    print(f"Gum: {gum.count} bead markers placed" + ("" if gum.ok else " (some batches failed)"))
last = PASSES[-1]
move([Z_UP, last["tilts"][-1], last["spins"][-1]], 3, "Raise nozzle")
move([Z_UP, 0.0, 0.0], 8, "Return home")
if RECLAMP:
    reclamp_to(0.0, 6, "Release ball back to start position")
print("Done  (remove the gum with:  python3 seam_gum_sequence.py --clear)")

# ---------------------------------------------------------------- self-checks (dry run only)
if DRY:
    def nozzle_pt(t, sp):
        x, y, zz = -math.sin(t), 0.0, math.cos(t)          # world top as seen from the cup frame
        return (x, math.cos(sp) * y + math.sin(sp) * zz, -math.sin(sp) * y + math.cos(sp) * zz)
    worst = 0.0
    for pi, P in enumerate(PASSES):
        for s in sent[marks[pi]:ends[pi]]:
            q = nozzle_pt(s[1], s[2])
            worst = max(worst, min(ang(q, p) for p in P["cpts"]))
    print(f"DRY RUN: {len(sent)} commands, {NJ} joints each; nozzle stays within {worst * R * 1000:.2f} mm of a route sample")
    print(f"DRY RUN: max tilt used {max(abs(s[1]) for s in sent):.3f} rad (limit 0.9), "
          f"max cup spin {max(abs(s[2]) for s in sent):.2f} rad (limit 12.6), "
          f"z range {min(s[0] for s in sent):.3f}..{max(s[0] for s in sent):.3f} m")
    if RECLAMP:
        print(f"DRY RUN: re-clamp joint used {min(s[4] for s in sent):.3f}..{max(s[4] for s in sent):.3f} rad")
    if GUM:
        beads = [m for txt in sent_markers for m in txt.split("marker {")[1:]]
        expect = sum(sum(P["gum"]) for P in PASSES)
        print(f"DRY RUN: {len(beads)} bead markers in {len(sent_markers)} requests (incl. the clear request); "
              f"samples flagged for gum: {expect}")
        bad = 0
        for pi, k, cmd_idx in bead_log:
            s = sent[cmd_idx - 1]
            if ang(nozzle_pt(s[1], s[2]), PASSES[pi]["cpts"][k]) * R > 0.015 * K:
                bad += 1
        print(f"DRY RUN: beads dropped away from the nozzle: {bad}")
else:
    node.destroy_node()
    rclpy.shutdown()
