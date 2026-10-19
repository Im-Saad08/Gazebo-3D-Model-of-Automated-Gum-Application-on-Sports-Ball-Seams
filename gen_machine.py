import json, math, shutil, sys
from pathlib import Path

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
RECLAMP = "--reclamp" in sys.argv
# ---- ball catalogue: what the operator can choose (radius in metres, approx. from standard circumferences) ----
BALL_CATALOG = {
    "football":   {"label": "Football",   "enabled": True,  "sizes": {3: 0.095, 4: 0.103, 5: 0.110}},
    "volleyball": {"label": "Volleyball", "enabled": False, "sizes": {}},
    "basketball": {"label": "Basketball", "enabled": False, "sizes": {}},
}
if "--list-balls" in sys.argv:        # the panel asks this, so new balls appear there without editing the panel
    print(json.dumps({k: {"label": v["label"], "enabled": v["enabled"], "sizes": sorted(v["sizes"])} for k, v in BALL_CATALOG.items()}))
    sys.exit(0)
BALL_TYPE = "football"
BALL_SIZE = 5
for a in sys.argv[1:]:
    if a.startswith("--ball-size="):
        BALL_SIZE = int(a.split("=", 1)[1])
    if a.startswith("--ball="):
        BALL_TYPE = a.split("=", 1)[1].lower()
assert BALL_TYPE in BALL_CATALOG, f"unknown ball '{BALL_TYPE}'. Choose: {', '.join(BALL_CATALOG)}"
assert BALL_CATALOG[BALL_TYPE]["enabled"], f"'{BALL_TYPE}' is not available yet"
BALL_RADII = BALL_CATALOG[BALL_TYPE]["sizes"]
GAIN = 0.4      # gz_ros2_control position gain (Gazebo default 0.1 is sluggish: joints lag their commands)
for a in sys.argv[1:]:
    if a.startswith("--gain="):
        GAIN = float(a.split("=", 1)[1])
assert BALL_SIZE in BALL_RADII, f"{BALL_TYPE} sizes are: {sorted(BALL_RADII)}"
BALL_R = BALL_RADII[BALL_SIZE]
S = 0.5      # the MACHINE is one fixed size (built around the size-5 ball); only ball + cup spacing change
def _mmv(v):
    return v / 1000.0 / S
def _p(xm, ym, zm):
    return f"{_mmv(xm):.5f} {_mmv(ym):.5f} {_mmv(zm):.5f}"
default = Path.home() / "FYP_ball_gum/src/football_gumming_sim/urdf/football_gumming.urdf.xacro"
urdf = Path(ARGS[0]) if len(ARGS) > 0 else default
if urdf.exists():
    bak = Path(str(urdf) + ".v3.bak")
    if not bak.exists():
        shutil.copy(urdf, bak)

import re
def scale_urdf(text, k):
    """Uniformly scale every length (xyz, size, radius, length) and the Z-axis travel."""
    if abs(k - 1.0) < 1e-9:
        return text
    def nums(m):
        vals = [float(t) * k for t in m.group(2).split()]
        return f'{m.group(1)}="' + " ".join(f"{v:.6g}" for v in vals) + '"'
    text = re.sub(r'<axis xyz="([^"]*)"/>', r'<axis KEEP="\1"/>', text)      # joint axes are directions: never scale them
    text = re.sub(r'(xyz|size)="([^"]*)"', nums, text)
    text = re.sub(r'(radius|length)="([^"]*)"', nums, text)
    text = re.sub(r'<axis KEEP="([^"]*)"/>', r'<axis xyz="\1"/>', text)
    # z slide: strong effort and NO small velocity cap (a velocity cap on a vertical prismatic joint makes Gazebo drop it to the limit)
    z = ('<limit lower="-0.20" upper="0.10" effort="60" velocity="0.15"/>',
         f'<limit lower="{-0.20 * k:.4f}" upper="{0.10 * k:.4f}" effort="1000" velocity="10"/>')
    assert z[0] in text, "z limit line not found"
    return text.replace(z[0], z[1])

def inertia(m, x, y, z):
    return (f'<inertia ixx="{m*(y*y+z*z)/12:.6f}" ixy="0" ixz="0" '
            f'iyy="{m*(x*x+z*z)/12:.6f}" iyz="0" izz="{m*(x*x+y*y)/12:.6f}"/>')

def vbox(n, xyz, size, mat, rpy="0 0 0"):
    return (f'    <visual name="{n}"><origin xyz="{xyz}" rpy="{rpy}"/>'
            f'<geometry><box size="{size}"/></geometry><material name="{mat}"/></visual>\n')

def vcyl(n, xyz, r, l, mat, rpy="0 0 0"):
    return (f'    <visual name="{n}"><origin xyz="{xyz}" rpy="{rpy}"/>'
            f'<geometry><cylinder radius="{r}" length="{l}"/></geometry><material name="{mat}"/></visual>\n')

def vsph(n, xyz, r, mat):
    return (f'    <visual name="{n}"><origin xyz="{xyz}"/>'
            f'<geometry><sphere radius="{r}"/></geometry><material name="{mat}"/></visual>\n')

def box_link(name, x, y, z, m, mat, extra=""):
    return f'''
  <link name="{name}">
    <inertial><mass value="{m}"/>{inertia(m, x, y, z)}</inertial>
    <visual><geometry><box size="{x} {y} {z}"/></geometry><material name="{mat}"/></visual>
    <collision><geometry><box size="{x} {y} {z}"/></geometry></collision>
{extra}  </link>'''

def fixed(name, parent, child, xyz):
    return f'''
  <joint name="{name}" type="fixed">
    <parent link="{parent}"/><child link="{child}"/><origin xyz="{xyz}"/>
  </joint>'''

# ---------- football panel pattern (truncated icosahedron) ----------
def norm(v):
    n = math.sqrt(sum(t * t for t in v))
    return tuple(t / n for t in v)

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def slerp(a, b, t):
    w = math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)))))
    s = math.sin(w)
    return tuple((math.sin((1 - t) * w) * x + math.sin(t * w) * y) / s for x, y in zip(a, b))

def rpy_from_cols(xc, yc, zc):
    pitch = math.asin(max(-1.0, min(1.0, -xc[2])))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(yc[2], zc[2])
        yaw = math.atan2(xc[1], xc[0])
    else:
        roll = math.atan2(-zc[1], yc[1])
        yaw = 0.0
    return roll, pitch, yaw

phi = (1 + 5 ** 0.5) / 2
pts = set()
for (a, b, c) in [(0, 1, 3 * phi), (1, 2 + phi, 2 * phi), (phi, 2, 2 * phi + 1)]:
    for sa in (1, -1):
        for sb in (1, -1):
            for sc in (1, -1):
                v = (sa * a, sb * b, sc * c)
                for p in (v, (v[1], v[2], v[0]), (v[2], v[0], v[1])):
                    pts.add(tuple(round(t, 6) + 0.0 for t in p))
verts = sorted(pts)
assert len(verts) == 60, len(verts)
units0 = [norm(v) for v in verts]

def dot3(a, b):
    return sum(x * y for x, y in zip(a, b))

def rodrigues(k, ang):
    K = [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]
    K2 = [[sum(K[i][m] * K[m][j] for m in range(3)) for j in range(3)] for i in range(3)]
    return [[(1 if i == j else 0) + math.sin(ang) * K[i][j] + (1 - math.cos(ang)) * K2[i][j]
             for j in range(3)] for i in range(3)]

def matvec(M, v):
    return tuple(sum(M[i][j] * v[j] for j in range(3)) for i in range(3))

_pc = norm((0, 1, (1 + 5 ** 0.5) / 2))          # a pentagon centre of the pattern
_ax = norm(cross(_pc, (1, 0, 0)))
R0 = rodrigues(_ax, math.acos(dot3(_pc, (1, 0, 0))))
units = [matvec(R0, u) for u in units0]          # cups now grip pentagon centres
edges = []
for i in range(60):
    for j in range(i + 1, 60):
        d = math.sqrt(sum((x - y) ** 2 for x, y in zip(verts[i], verts[j])))
        if abs(d - 2.0) < 1e-3:
            edges.append((i, j))
assert len(edges) == 90, len(edges)

# --- second-clamp geometry: pick the panel-centre pair (pentagon or hexagon) whose clamp ----
# --- makes every edge left over from clamp 1 reachable --------------------------------------
_phi = (1 + 5 ** 0.5) / 2
TILT_LIMIT = 0.9
XMAX = math.sin(TILT_LIMIT)

def edge_ok(u, v, rot):
    a, b = matvec(rot, u), matvec(rot, v)
    return all(abs(slerp(a, b, t / 8)[0]) <= XMAX for t in range(9))

_I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
_reach1 = {e for e in edges if edge_ok(units[e[0]], units[e[1]], _I3)}
_left = set(edges) - _reach1
_cands = []
for _s1 in (1, -1):
    for _s2 in (1, -1):
        _v = (0.0, _s1 * 1.0, _s2 * _phi)
        for _p in (_v, (_v[1], _v[2], _v[0]), (_v[2], _v[0], _v[1])):
            _cands.append(("pentagon", _p))
        for _s3 in (1, -1):
            _cands.append(("hexagon", (_s1 * 1.0, _s2 * 1.0, _s3 * 1.0)))
        _w = (0.0, _s1 / _phi, _s2 * _phi)
        for _p in (_w, (_w[1], _w[2], _w[0]), (_w[2], _w[0], _w[1])):
            _cands.append(("hexagon", _p))
_best = None
for _kind, _p in _cands:
    _c = matvec(R0, norm(_p))
    _d = dot3(_c, (1, 0, 0))
    if abs(abs(_d) - 1) < 1e-6:
        continue
    _ang = math.acos(max(-1.0, min(1.0, _d)))
    _rot = rodrigues(norm(cross(_c, (1, 0, 0))), _ang)
    if all(edge_ok(units[e[0]], units[e[1]], _rot) for e in _left):
        if _best is None or _ang < _best[0]:
            _best = (_ang, _c, _kind)
assert _best is not None, "no second clamp reaches every remaining edge"
CLAMP_ANGLE, _B, CLAMP_KIND = _best
CLAMP_AXIS = norm(cross(_B, (1, 0, 0)))
CLAMP_ROT = rodrigues(CLAMP_AXIS, CLAMP_ANGLE)       # maps the 2nd clamp axis onto the cup axis

R = BALL_R / S       # ball radius in pre-scale drawing units
SEG = 3
seam_vis = []
k = 0
for (i, j) in edges:
    a, b = units[i], units[j]
    for s in range(SEG):
        p0 = slerp(a, b, s / SEG)
        p1 = slerp(a, b, (s + 1) / SEG)
        mid = norm(tuple(x + y for x, y in zip(p0, p1)))
        d = tuple(y - x for x, y in zip(p0, p1))
        dn = sum(x * m for x, m in zip(d, mid))
        d = norm(tuple(x - dn * m for x, m in zip(d, mid)))
        yv = cross(mid, d)
        length = R * math.sqrt(sum((y - x) ** 2 for x, y in zip(p0, p1)))
        pos = tuple(R * m for m in mid)
        roll, pitch, yaw = rpy_from_cols(d, yv, mid)
        seam_vis.append(vbox(f"seam_{k}", f"{pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f}",
                             f"{length + 0.002:.4f} 0.005 0.003", "seam_black",
                             f"{roll:.5f} {pitch:.5f} {yaw:.5f}"))
        k += 1

guide = []

# ---------- URDF ----------
x = '<?xml version="1.0"?>\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="football_gumming_machine">\n'
PALETTE = [
    # structure
    ("frame_gray", "0.17 0.19 0.22 1"),        # dark graphite: base slab, pillar, brackets
    ("alu_frame", "0.74 0.76 0.79 1"),         # anodised aluminium extrusion: posts and beams
    ("rail_silver", "0.84 0.86 0.89 1"),       # polished steel: rails, shafts, couplings, caps
    ("alu_anodised", "0.66 0.69 0.73 1"),      # tilt-cradle plates and bars
    ("steel_dark", "0.40 0.42 0.46 1"),        # lead screw
    ("stainless", "0.90 0.91 0.92 1"),         # drip tray
    ("galvanised", "0.60 0.63 0.67 1"),        # cabinet backplate
    ("rubber_black", "0.05 0.05 0.05 1"),
    ("motor_dark", "0.07 0.08 0.10 1"),        # stepper bodies, carriage plates
    # moving parts / process
    ("cup_red", "0.82 0.10 0.08 1"),
    ("glue_amber", "0.98 0.74 0.08 1"),
    ("barrel_clear", "0.80 0.90 0.97 0.30"),   # clear polyethylene barrel
    ("tip_blue", "0.12 0.38 0.88 1"),          # 22 G tip colour code
    ("luer_white", "0.92 0.93 0.95 1"),
    ("hose_blue", "0.12 0.45 0.90 1"),         # air hoses
    ("dryer_teal", "0.08 0.55 0.55 1"),
    # football
    ("football_white", "0.97 0.97 0.95 1"),
    ("patch_black", "0.04 0.04 0.05 1"),
    ("seam_black", "0.12 0.12 0.13 1"),
    # electrics
    ("enclosure_grey", "0.80 0.81 0.80 1"),    # RAL 7035 light grey
    ("duct_grey", "0.88 0.89 0.89 1"),
    ("pcb_green", "0.05 0.38 0.14 1"),
    ("hmi_blue", "0.08 0.30 0.62 1"),
    ("cabinet_blue", "0.10 0.28 0.55 1"),
    ("safety_yellow", "0.98 0.80 0.03 1"),
    ("green_led", "0.10 0.85 0.25 1"),
    ("red_btn", "0.85 0.08 0.08 1"),
    ("nozzle_orange", "0.96 0.45 0.06 1"),
    # kept so older generators' names still resolve
    ("nozzle_orange_old", "0.95 0.35 0.06 1"),
]
for n, c in PALETTE:
    x += f'  <material name="{n}"><color rgba="{c}"/></material>\n'

# pin the machine to the ground (base slab bottom sits at z=0)
x += '''
  <link name="world"/>
  <joint name="world_fixed" type="fixed">
    <parent link="world"/><child link="base_link"/><origin xyz="0 0 -0.14"/>
  </joint>
'''

base_extra = ""
base_extra += vbox("rail_a", "0 -0.30 0.315", "0.90 0.03 0.03", "rail_silver")
base_extra += vbox("rail_b", "0 0.30 0.315", "0.90 0.03 0.03", "rail_silver")
# ---------------- control cabinet: outside the frame, right side, mid height ----------------
def _v3(a, b, c):
    return f"{a:.5f} {b:.5f} {c:.5f}"
CAB_W, CAB_D, CAB_H, CAB_T = 0.70, 0.40, 0.80, 0.012       # pre-scale = 350 x 200 x 400 mm (W x D x H)
CAB_X0, CAB_YB, CAB_Z0 = 0.68, 0.37, 0.40                   # machine-side face, rear plane, bottom (base_link frame)
cx, cy, cz = CAB_X0 + CAB_W / 2, CAB_YB - CAB_D / 2, CAB_Z0 + CAB_H / 2
yf = CAB_YB - CAB_D                                         # front plane (operator side)
def ci(u, d, w):                                            # point inside: u across, d depth from back wall, w height
    return _v3(cx + u, CAB_YB - CAB_T - d, cz + w)
base_extra += vbox("cab_back", _v3(cx, CAB_YB - CAB_T / 2, cz), _v3(CAB_W, CAB_T, CAB_H), "enclosure_grey")
base_extra += vbox("cab_left", _v3(cx - CAB_W / 2 + CAB_T / 2, cy, cz), _v3(CAB_T, CAB_D, CAB_H), "enclosure_grey")
base_extra += vbox("cab_right", _v3(cx + CAB_W / 2 - CAB_T / 2, cy, cz), _v3(CAB_T, CAB_D, CAB_H), "enclosure_grey")
base_extra += vbox("cab_top", _v3(cx, cy, cz + CAB_H / 2 - CAB_T / 2), _v3(CAB_W, CAB_D, CAB_T), "enclosure_grey")
base_extra += vbox("cab_bottom", _v3(cx, cy, cz - CAB_H / 2 + CAB_T / 2), _v3(CAB_W, CAB_D, CAB_T), "enclosure_grey")
for _w in (-0.26, 0.26):                                    # brackets bolting the cabinet to the right-back post
    base_extra += vbox(f"cab_bracket_{_w}", _v3(0.617, 0.32, cz + _w), _v3(0.14, 0.10, 0.03), "frame_gray")
base_extra += vbox("cab_shelf", _v3(0.617, 0.30, CAB_Z0 - 0.015), _v3(0.14, 0.20, 0.03), "frame_gray")
# galvanised backplate with DIN rails
base_extra += vbox("cab_backplate", ci(0, 0.004, 0), _v3(0.66, 0.008, 0.76), "galvanised")
base_extra += vbox("din_rail_top", ci(0, 0.012, 0.19), _v3(0.62, 0.006, 0.03), "duct_grey")
for _w in (0.06, -0.05, -0.16, -0.27):
    base_extra += vbox(f"din_rail_drive_{_w}", ci(-0.17, 0.012, _w - 0.045), _v3(0.30, 0.006, 0.02), "duct_grey")
# row A: 24 V SMPS power supply (terminal notches) and central PLC
base_extra += vbox("psu_smps", ci(-0.19, 0.06, 0.25), _v3(0.24, 0.09, 0.12), "rail_silver")
for _k in range(6):
    base_extra += vbox(f"psu_notch_top_{_k}", ci(-0.28 + 0.036 * _k, 0.06, 0.25 + 0.062), _v3(0.02, 0.05, 0.01), "rubber_black")
    base_extra += vbox(f"psu_notch_bot_{_k}", ci(-0.28 + 0.036 * _k, 0.06, 0.25 - 0.062), _v3(0.02, 0.05, 0.01), "rubber_black")
base_extra += vbox("psu_label", ci(-0.19, 0.106, 0.25), _v3(0.14, 0.004, 0.05), "nozzle_orange")
base_extra += vbox("plc_module", ci(0.13, 0.06, 0.25), _v3(0.36, 0.08, 0.12), "motor_dark")
base_extra += vbox("plc_cpu", ci(0.03, 0.102, 0.25), _v3(0.10, 0.004, 0.09), "pcb_green")
for _k in range(10):
    base_extra += vsph(f"plc_led_{_k}", ci(0.09 + 0.026 * _k * 0.9, 0.102, 0.25 + 0.03), 0.006, "green_led" if _k % 3 else "nozzle_orange")
# slotted wiring ducts above and below the drives
for _nm, _w in (("top", 0.35), ("mid", 0.145), ("bottom", -0.35)):
    base_extra += vbox(f"duct_{_nm}", ci(0, 0.04, _w), _v3(0.62, 0.07, 0.05), "duct_grey")
    for _k in range(16):
        base_extra += vbox(f"duct_{_nm}_slot_{_k}", ci(-0.28 + 0.037 * _k, 0.0745, _w), _v3(0.014, 0.004, 0.03), "rubber_black")
# four drives stacked vertically: silver heat-sink body, dark front face
for _i, _w in enumerate((0.06, -0.05, -0.16, -0.27)):
    base_extra += vbox(f"drive_{_i}_body", ci(-0.17, 0.06, _w), _v3(0.26, 0.09, 0.09), "rail_silver")
    for _f in (-0.08, 0.0, 0.08):
        base_extra += vbox(f"drive_{_i}_fin_{_f}", ci(-0.17 + _f, 0.06, _w + 0.047), _v3(0.004, 0.07, 0.006), "rail_silver")
    base_extra += vbox(f"drive_{_i}_front", ci(-0.17, 0.108, _w), _v3(0.26, 0.006, 0.09), "motor_dark")
    base_extra += vsph(f"drive_{_i}_led", ci(-0.09, 0.114, _w + 0.025), 0.007, "green_led")
# terminal blocks and relay modules on the right
for _w in (0.06, -0.05, -0.16):
    base_extra += vbox(f"terminal_strip_{_w}", ci(0.14, 0.05, _w), _v3(0.30, 0.03, 0.04), "nozzle_orange")
for _k, _u in enumerate((0.07, 0.21)):
    base_extra += vbox(f"relay_{_k}", ci(_u, 0.05, -0.27), _v3(0.12, 0.06, 0.07), "cabinet_blue")
# door (shown open by default; pass --door-closed to close it): bezel, hinges, handle, latch, 7-inch HMI
DOOR_OPEN = "--door-closed" not in sys.argv
if DOOR_OPEN:
    _yaw = math.radians(100.0)
    _hx, _hy = cx + CAB_W / 2, yf
    _dcx, _dcy = _hx + (CAB_W / 2) * math.cos(_yaw + math.pi), _hy + (CAB_W / 2) * math.sin(_yaw + math.pi)
    _dcx, _dcy = _hx + (CAB_W / 2) * 0.17365, _hy - (CAB_W / 2) * 0.98481
else:
    _yaw, _dcx, _dcy = 0.0, cx, yf - 0.01
def dp(u, v, w):                                            # door-local point -> base_link frame
    return _v3(_dcx + u * math.cos(_yaw) - v * math.sin(_yaw), _dcy + u * math.sin(_yaw) + v * math.cos(_yaw), cz + w)
_drpy = f"0 0 {_yaw:.5f}"
_dcrpy = f"1.5708 0 {_yaw:.5f}"
base_extra += vbox("cab_door", dp(0, 0, 0), _v3(CAB_W, 0.02, CAB_H), "enclosure_grey", _drpy)
base_extra += vbox("door_bezel_top", dp(0, -0.014, 0.385), _v3(CAB_W, 0.012, 0.03), "rail_silver", _drpy)
base_extra += vbox("door_bezel_bottom", dp(0, -0.014, -0.385), _v3(CAB_W, 0.012, 0.03), "rail_silver", _drpy)
base_extra += vbox("door_bezel_left", dp(-0.335, -0.014, 0), _v3(0.03, 0.012, CAB_H), "rail_silver", _drpy)
base_extra += vbox("door_bezel_right", dp(0.335, -0.014, 0), _v3(0.03, 0.012, CAB_H), "rail_silver", _drpy)
base_extra += vbox("hmi_frame", dp(-0.04, -0.019, 0.12), _v3(0.38, 0.012, 0.25), "motor_dark", _drpy)
base_extra += vbox("hmi_screen_7in", dp(-0.04, -0.0255, 0.12), _v3(0.32, 0.004, 0.19), "hmi_blue", _drpy)
base_extra += vbox("door_handle", dp(-0.30, -0.035, -0.05), _v3(0.035, 0.03, 0.22), "rail_silver", _drpy)
base_extra += vcyl("door_latch", dp(-0.30, -0.022, -0.26), 0.02, 0.02, "rubber_black", _dcrpy)
for _w in (0.30, 0.0, -0.30):
    base_extra += vcyl(f"door_hinge_{_w}", dp(0.35, 0.0, _w), 0.012, 0.09, "rail_silver")
# slanted operator control strip on top front: outside the frame envelope, at hand height
_TH = 0.45
_scx, _scy, _scz = cx, yf - 0.06, cz + CAB_H / 2 + 0.035
def strip(u, v, w):
    return _v3(_scx + u, _scy + v * math.cos(_TH) - w * math.sin(_TH), _scz + v * math.sin(_TH) + w * math.cos(_TH))
_srpy = f"{_TH} 0 0"
base_extra += vbox("strip_base", _v3(cx, _scy + 0.03, cz + CAB_H / 2 + 0.012), _v3(CAB_W, 0.14, 0.03), "enclosure_grey")
base_extra += vbox("ctrl_strip", strip(0, 0, 0), _v3(CAB_W, 0.17, 0.02), "motor_dark", _srpy)
base_extra += vcyl("estop_yellow_ring", strip(0.22, 0.0, 0.016), 0.07, 0.012, "safety_yellow", _srpy)
base_extra += vcyl("estop_button", strip(0.22, 0.0, 0.040), 0.045, 0.03, "red_btn", _srpy)
base_extra += vcyl("btn_start", strip(-0.22, 0.0, 0.02), 0.026, 0.02, "green_led", _srpy)
base_extra += vcyl("btn_stop", strip(-0.14, 0.0, 0.02), 0.026, 0.02, "red_btn", _srpy)
base_extra += vcyl("btn_reset", strip(-0.06, 0.0, 0.02), 0.026, 0.02, "cabinet_blue", _srpy)
base_extra += vcyl("key_switch", strip(0.06, 0.0, 0.02), 0.03, 0.025, "rubber_black", _srpy)
# status beacon tower on the cabinet roof
base_extra += vcyl("beacon_base", _v3(cx - 0.27, cy, cz + CAB_H / 2 + 0.03), 0.02, 0.06, "motor_dark")
for _k, (_m, _z) in enumerate((("red_btn", 0.085), ("nozzle_orange", 0.135), ("green_led", 0.185))):
    base_extra += vcyl(f"beacon_light_{_k}", _v3(cx - 0.27, cy, cz + CAB_H / 2 + _z), 0.028, 0.05, _m)
# drip tray under the ball (catches any drip that runs off the ball)
base_extra += vbox("drip_tray_floor", "0 0.05 0.325", "0.50 0.40 0.01", "stainless")
base_extra += vbox("drip_tray_front", "0 -0.145 0.35", "0.50 0.01 0.05", "stainless")
base_extra += vbox("drip_tray_back", "0 0.245 0.35", "0.50 0.01 0.05", "stainless")
base_extra += vbox("drip_tray_left", "-0.245 0.05 0.35", "0.01 0.40 0.05", "stainless")
base_extra += vbox("drip_tray_right", "0.245 0.05 0.35", "0.01 0.40 0.05", "stainless")
for fx in (-0.50, 0.50):
    for fy in (-0.35, 0.35):
        base_extra += vcyl(f"foot_{fx}_{fy}", f"{fx} {fy} 0.17", 0.04, 0.06, "rubber_black")
x += f'''
  <link name="base_link">
    <inertial><mass value="30.0"/><inertia ixx="3.0" ixy="0" ixz="0" iyy="3.0" iyz="0" izz="3.0"/></inertial>
    <visual><origin xyz="0 0 0.25"/><geometry><box size="1.2 0.9 0.10"/></geometry><material name="frame_gray"/></visual>
    <collision><origin xyz="0 0 0.25"/><geometry><box size="1.2 0.9 0.10"/></geometry></collision>
{base_extra}  </link>'''

for nm, px, py in [("left_front_post", -0.52, -0.37), ("left_back_post", -0.52, 0.37),
                   ("right_front_post", 0.52, -0.37), ("right_back_post", 0.52, 0.37)]:
    x += box_link(nm, 0.07, 0.07, 1.4, 3.0, "alu_frame")
    x += fixed(nm + "_mount", "base_link", nm, f"{px} {py} 0.95")
x += box_link("top_front_beam", 1.1, 0.07, 0.07, 3.0, "alu_frame") + fixed("front_beam_mount", "base_link", "top_front_beam", "0 -0.37 1.65")
x += box_link("top_back_beam", 1.1, 0.07, 0.07, 3.0, "alu_frame") + fixed("back_beam_mount", "base_link", "top_back_beam", "0 0.37 1.65")
x += box_link("top_cross_beam", 0.07, 0.74, 0.07, 3.0, "alu_frame") + fixed("cross_beam_mount", "base_link", "top_cross_beam", "0 0 1.65")
# lower frame rails
x += box_link("low_rail_front", 1.0, 0.05, 0.05, 2.0, "alu_frame") + fixed("low_rail_front_mount", "base_link", "low_rail_front", "0 -0.37 0.40")
x += box_link("low_rail_back", 1.0, 0.05, 0.05, 2.0, "alu_frame") + fixed("low_rail_back_mount", "base_link", "low_rail_back", "0 0.37 0.40")
x += box_link("low_rail_left", 0.05, 0.69, 0.05, 2.0, "alu_frame") + fixed("low_rail_left_mount", "base_link", "low_rail_left", "-0.52 0 0.40")
x += box_link("low_rail_right", 0.05, 0.69, 0.05, 2.0, "alu_frame") + fixed("low_rail_right_mount", "base_link", "low_rail_right", "0.52 0 0.40")

# nozzle vertical slide: back plate, 2 guide rods, lead screw, end blocks, motor, limit switches
def _mmv(v):
    return v / 1000.0 / S
def _p(xm, ym, zm):                       # millimetres -> "x y z" string in pre-scale units
    return f"{_mmv(xm):.5f} {_mmv(ym):.5f} {_mmv(zm):.5f}"
SLIDE_Y = -33.0                           # rods / screw sit this far in front of the back plate
rail_extra = vcyl("guide_rod_a", _p(22, SLIDE_Y, 0), f"{_mmv(3):.5f}", f"{_mmv(320):.5f}", "rail_silver")
rail_extra += vcyl("guide_rod_b", _p(-22, SLIDE_Y, 0), f"{_mmv(3):.5f}", f"{_mmv(320):.5f}", "rail_silver")
rail_extra += vcyl("lead_screw", _p(0, SLIDE_Y, 0), f"{_mmv(4):.5f}", f"{_mmv(320):.5f}", "steel_dark")
rail_extra += vbox("end_block_top", _p(0, -18.5, 153), _p(70, 45, 14), "motor_dark")
rail_extra += vbox("end_block_bottom", _p(0, -18.5, -153), _p(70, 45, 14), "motor_dark")
rail_extra += vcyl("z_coupling", _p(0, SLIDE_Y, 175), f"{_mmv(7.5):.5f}", f"{_mmv(25):.5f}", "rail_silver")
rail_extra += vcyl("z_motor", _p(0, SLIDE_Y, 210), f"{_mmv(20):.5f}", f"{_mmv(45):.5f}", "motor_dark")
rail_extra += vbox("limit_switch_top", _p(38, -10, 132), _p(12, 10, 16), "safety_yellow")
rail_extra += vbox("limit_switch_bottom", _p(38, -10, 45), _p(12, 10, 16), "safety_yellow")
x += box_link("nozzle_rail", _mmv(70), _mmv(8), 0.60, 1.0, "rail_silver", rail_extra) + fixed("rail_mount", "base_link", "nozzle_rail", f"0 {_mmv(88):.5f} 1.35")
car_extra = vbox("front_plate", f"0 {-_mmv(28):.5f} 0", f"{_mmv(56):.5f} {_mmv(6):.5f} {_mmv(70):.5f}", "motor_dark")
for _bx in (-22, 22):
    for _bz in (-28, 28):
        car_extra += vcyl(f"plate_bolt_{_bx}_{_bz}", f"{_mmv(_bx):.5f} {-_mmv(31.5):.5f} {_mmv(_bz):.5f}",
                          f"{_mmv(2.5):.5f}", f"{_mmv(3):.5f}", "rail_silver", "1.5708 0 0")
# pneumatic block on the carriage's left side: dry-air filter, regulator + gauge, dispense solenoid
car_extra += vbox("pneu_plate", _p(-28, 0, 0), _p(6, 46, 70), "alu_anodised")
car_extra += vcyl("air_dryer", _p(-45, -12, -5), f"{_mmv(13):.5f}", f"{_mmv(55):.5f}", "dryer_teal")
car_extra += vcyl("air_regulator", _p(-45, 12, 5), f"{_mmv(11):.5f}", f"{_mmv(40):.5f}", "rail_silver")
car_extra += vcyl("regulator_knob", _p(-45, 12, 28), f"{_mmv(8):.5f}", f"{_mmv(10):.5f}", "motor_dark")
car_extra += vcyl("pressure_gauge", _p(-45, 12, 40), f"{_mmv(17):.5f}", f"{_mmv(8):.5f}", "luer_white", "1.5708 0 0")
car_extra += vbox("gauge_needle", _p(-45, 7.5, 42), _p(1.5, 1, 12), "red_btn")
car_extra += vbox("dispense_solenoid", _p(-72, 0, -5), _p(24, 18, 34), "motor_dark")
car_extra += vcyl("solenoid_coil", _p(-72, 0, 19), f"{_mmv(8):.5f}", f"{_mmv(14):.5f}", "rail_silver")
car_extra += vcyl("air_inlet_hose", _p(-80, -14, 25), f"{_mmv(3):.5f}", f"{_mmv(70):.5f}", "hose_blue", "0 1.5708 0")
car_extra += vcyl("dryer_inlet_fitting", _p(-45, -14, 25.5), f"{_mmv(4):.5f}", f"{_mmv(6):.5f}", "rail_silver")
car_extra += vbox("frl_manifold", _p(-45, 0, -14), _p(10, 24, 8), "rail_silver")          # dryer -> regulator
car_extra += vcyl("reg_outlet_hose_a", _p(-64, 14, 0), f"{_mmv(2):.5f}", f"{_mmv(16):.5f}", "hose_blue", "0 1.5708 0")
car_extra += vcyl("reg_outlet_hose_b", _p(-72, 11.5, 0), f"{_mmv(2):.5f}", f"{_mmv(5):.5f}", "hose_blue", "1.5708 0 0")   # into solenoid
car_extra += vcyl("air_inlet_coupler", _p(-118, -14, 25), f"{_mmv(5):.5f}", f"{_mmv(10):.5f}", "rail_silver", "0 1.5708 0")
car_extra += vbox("limit_flag", _p(30, 20, 35), _p(10, 16, 20), "red_btn")
car_extra += vbox("nut_housing", _p(0, 14, 0), _p(30, 14, 40), "alu_anodised")
x += box_link("z_carriage", 0.10, 0.10, 0.16, 1.0, "rail_silver", car_extra)
x += f'''
  <joint name="z_axis_joint" type="prismatic">
    <parent link="nozzle_rail"/><child link="z_carriage"/>
    <origin xyz="0 {-33 / 1000.0 / S:.5f} 0.15"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.20" upper="0.10" effort="60" velocity="0.15"/>
    <dynamics damping="0.5" friction="0.0"/>
  </joint>
'''
# ---- realistic dispensing head, drawn in REAL millimetres (scale-proof) ------------------
def mm(v):                       # real millimetres -> units of the pre-scale drawing
    return f"{v / 1000.0 / S:.5f}"
TIP_Z = 0.17                     # needle tip stays 0.17 below this link's origin (pre-scale): Z_WORK unchanged
def nz(h):                       # height above the needle tip (mm) -> z in this link's frame
    return f"{-TIP_Z + h / 1000.0 / S:.5f}"
def ncyl(name, h0, h1, r, mat, rpy="0 0 0", x=0.0):
    return vcyl(name, f"{x / 1000.0 / S:.5f} 0 {nz((h0 + h1) / 2.0)}", mm(r), mm(h1 - h0), mat, rpy)
noz = ""
noz += ncyl("needle_22G", 0, 18, 0.36, "rail_silver")           # 22 G dispensing needle, OD 0.72 mm
noz += ncyl("tip_taper", 18, 22, 1.8, "tip_blue")               # tapered tip (22 G = blue)
noz += ncyl("tip_taper2", 22, 27, 3.2, "tip_blue")
noz += ncyl("luer_hub", 27, 37, 4.5, "tip_blue")                # luer hub
noz += ncyl("luer_lock_collar", 37, 45, 6.5, "luer_white")      # luer-lock collar
noz += ncyl("glue_fill", 45, 112, 7.2, "glue_amber")             # glue inside the barrel
noz += ncyl("piston_stopper", 112, 118, 7.2, "rubber_black")    # rubber piston on top of the glue
noz += ncyl("barrel_10cc", 45, 135, 8.2, "barrel_clear")        # 10 cc clear syringe barrel
noz += ncyl("barrel_flange", 135, 137, 11.5, "motor_dark")
noz += ncyl("clamp_collar", 123, 135, 13.0, "rail_silver")       # clamp that grips the barrel
noz += ncyl("piston_cap", 137, 159, 9.5, "motor_dark")          # air-pressure adapter cap
noz += ncyl("air_fitting", 159, 167, 3.0, "rail_silver")
noz += ncyl("air_hose", 167, 175, 2.0, "hose_blue")
noz += vbox("holder_plate", f"0 {20 / 1000.0 / S:.5f} {nz(118.5)}", f"{36 / 1000.0 / S:.5f} {8 / 1000.0 / S:.5f} {113 / 1000.0 / S:.5f}", "rail_silver")
noz += vbox("clamp_block_upper", f"0 {1.5 / 1000.0 / S:.5f} {nz(129)}", f"{28 / 1000.0 / S:.5f} {29 / 1000.0 / S:.5f} {12 / 1000.0 / S:.5f}", "motor_dark")
noz += vbox("clamp_block_lower", f"0 {1.5 / 1000.0 / S:.5f} {nz(76)}", f"{28 / 1000.0 / S:.5f} {29 / 1000.0 / S:.5f} {12 / 1000.0 / S:.5f}", "motor_dark")
noz += ncyl("clamp_collar_lower", 70, 82, 13.0, "rail_silver")
noz += vcyl("air_hose_out", f"{-36 / 1000.0 / S:.5f} 0 {nz(175)}", mm(2.0), mm(72), "hose_blue", "0 1.5708 0")
noz += vcyl("air_hose_to_valve", f"{-72 / 1000.0 / S:.5f} {23 / 1000.0 / S:.5f} {nz(175)}", mm(2.0), mm(46), "hose_blue", "1.5708 0 0")
x += f'''
  <link name="nozzle_body">
    <inertial><mass value="0.5"/>{inertia(0.5, 0.07, 0.07, 0.24)}</inertial>
{noz}  </link>''' + fixed("nozzle_mount", "z_carriage", "nozzle_body", f"0 {-55 / 1000.0 / S:.5f} -0.18")

# tilt pivot pillar with tilt motor
pil = vcyl("tilt_motor", "0 0.09 0.235", 0.05, 0.12, "motor_dark", "1.5708 0 0")
pil += vcyl("tilt_motor_cap", "0 0.16 0.235", 0.052, 0.02, "rail_silver", "1.5708 0 0")
x += box_link("pivot_pillar", 0.08, 0.06, 0.53, 2.0, "frame_gray", pil) + fixed("pillar_mount", "base_link", "pivot_pillar", "0 0.40 0.565")

# tilt cradle (with side motors, size-adjust slides, locking knobs)
tl = ""
tl += vbox("plate_left", "-0.40 0.10 0", "0.03 0.40 0.30", "rail_silver")
tl += vbox("plate_right", "0.40 0.10 0", "0.03 0.40 0.30", "rail_silver")
tl += vbox("rear_bar", "0 0.28 0", "0.83 0.04 0.04", "rail_silver")
tl += vcyl("pivot_pin", "0 0.34 0", 0.025, 0.12, "rail_silver", "1.5708 0 0")
for sgn, side in ((-1, "l"), (1, "r")):
    tl += vcyl(f"motor_{side}", f"{sgn*0.50} 0 0", 0.06, 0.14, "motor_dark", "0 1.5708 0")
    tl += vcyl(f"motor_cap_{side}", f"{sgn*0.585} 0 0", 0.062, 0.02, "rail_silver", "0 1.5708 0")
    tl += vcyl(f"coupling_{side}", f"{sgn*0.43} 0 0", 0.03, 0.03, "rail_silver", "0 1.5708 0")
    tl += vbox(f"slide_top_{side}", f"{sgn*0.42} 0.10 0.11", "0.01 0.30 0.02", "motor_dark")
    tl += vbox(f"slide_bot_{side}", f"{sgn*0.42} 0.10 -0.11", "0.01 0.30 0.02", "motor_dark")
    tl += vcyl(f"knob_{side}", f"{sgn*0.43} 0.22 0.0", 0.02, 0.03, "red_btn", "0 1.5708 0")
x += f'''
  <link name="tilt_link">
    <inertial><mass value="6.0"/><inertia ixx="0.3" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.3"/></inertial>
{tl}  </link>
  <joint name="tilt_joint" type="revolute">
    <parent link="base_link"/><child link="tilt_link"/>
    <origin xyz="0 0 0.80"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.9" upper="0.9" effort="200" velocity="1.0"/>
    <dynamics damping="2.0" friction="0.1"/>
  </joint>'''

CUP_D = R + 0.03                         # cup centre distance from the ball centre
SHAFT = 0.43 - CUP_D - 0.07                # shaft grows when the cups slide inward for a smaller ball

def football_mount():
    if not RECLAMP:
        return fixed("football_mount", "left_cup", "football", f"{CUP_D:.4f} 0 0")
    ax = " ".join(f"{t:.6f}" for t in CLAMP_AXIS)
    return f'''
  <joint name="reclamp_joint" type="revolute">
    <parent link="left_cup"/><child link="football"/><origin xyz="{CUP_D:.4f} 0 0"/>
    <axis xyz="{ax}"/>
    <limit lower="0" upper="{CLAMP_ANGLE + 0.02:.4f}" effort="50" velocity="0.8"/>
    <dynamics damping="1.0" friction="0.05"/>
  </joint>'''

def cup(side, sign):
    return f'''
  <link name="{side}_cup">
    <inertial><mass value="0.8"/><inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.003"/></inertial>
    <visual name="pad"><origin rpy="0 1.5708 0"/><geometry><cylinder radius="0.05" length="0.08"/></geometry><material name="cup_red"/></visual>
    <visual name="rim"><origin xyz="{-sign*0.035:.3f} 0 0" rpy="0 1.5708 0"/><geometry><cylinder radius="0.056" length="0.012"/></geometry><material name="rubber_black"/></visual>
    <visual name="shaft"><origin xyz="{sign*(0.04 + SHAFT / 2):.4f} 0 0" rpy="0 1.5708 0"/><geometry><cylinder radius="0.02" length="{SHAFT:.4f}"/></geometry><material name="rail_silver"/></visual>
    <visual name="marker"><origin xyz="0 0.05 0"/><geometry><box size="0.09 0.02 0.02"/></geometry><material name="football_white"/></visual>
  </link>
  <joint name="{side}_cup_joint" type="revolute">
    <parent link="tilt_link"/><child link="{side}_cup"/>
    <origin xyz="{sign*CUP_D:.4f} 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-12.6" upper="12.6" effort="50" velocity="3.0"/>
    <dynamics damping="1.0" friction="0.05"/>
  </joint>'''
x += cup("left", -1) + cup("right", 1)

# ---------- black pentagon patches (mesh) ----------
import struct
NO_MESH = "--no-mesh" in sys.argv
def pentagon_patch_triangles(radius):
    """12 spherical pentagons whose corners are the ball's own corners (so they line up with the seams)."""
    cents = []
    for sa in (1, -1):
        for sb in (1, -1):
            v = (0.0, sa * 1.0, sb * phi)
            for p in (v, (v[1], v[2], v[0]), (v[2], v[0], v[1])):
                cents.append(matvec(R0, norm(p)))
    assert len(cents) == 12
    tris = []
    RINGS, ARC = 5, 8
    for c in cents:
        ang = sorted(((math.acos(max(-1.0, min(1.0, dot3(c, u)))), i) for i, u in enumerate(units)))[:5]
        assert abs(ang[0][0] - ang[4][0]) < 1e-3 and ang[4][0] < 0.40, "pentagon corners not found"
        ids = [i for _, i in ang]
        # order the five corners around the centre
        e1 = norm(tuple(units[ids[0]][k] - dot3(units[ids[0]], c) * c[k] for k in range(3)))
        e2 = cross(c, e1)
        ids.sort(key=lambda i: math.atan2(dot3(units[i], e2), dot3(units[i], e1)))
        for n in range(5):
            a, b = units[ids[n]], units[ids[(n + 1) % 5]]
            ring_pts = []
            for rr in range(RINGS + 1):
                f = rr / RINGS
                row = []
                for k in range(ARC + 1):
                    q = slerp(a, b, k / ARC)
                    row.append(norm(tuple(c[m] + (q[m] - c[m]) * f for m in range(3))) if f > 0 else c)
                ring_pts.append(row)
            for rr in range(RINGS):
                for k in range(ARC):
                    p00, p01 = ring_pts[rr][k], ring_pts[rr][k + 1]
                    p10, p11 = ring_pts[rr + 1][k], ring_pts[rr + 1][k + 1]
                    quad = [(p00, p10, p11)] if rr == 0 else [(p00, p10, p11), (p00, p11, p01)]
                    for t in quad:
                        t = tuple(tuple(radius * m for m in v) for v in t)
                        nrm = cross(tuple(t[1][m] - t[0][m] for m in range(3)), tuple(t[2][m] - t[0][m] for m in range(3)))
                        cen = tuple(sum(t[j][m] for j in range(3)) for m in range(3))
                        if dot3(nrm, cen) < 0:
                            t = (t[0], t[2], t[1])
                        tris.append(t)
    return tris

def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"football pentagon patches".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = cross(tuple(t[1][m] - t[0][m] for m in range(3)), tuple(t[2][m] - t[0][m] for m in range(3)))
            ln = math.sqrt(sum(v * v for v in n)) or 1.0
            f.write(struct.pack("<3f", *(v / ln for v in n)))
            for v in t:
                f.write(struct.pack("<3f", *v))
            f.write(b"\0\0")

patch_vis = ""
if not NO_MESH:
    meshdir = urdf.parent.parent / "meshes"
    meshdir.mkdir(parents=True, exist_ok=True)
    stl = meshdir / "ball_pentagons.stl"
    tris = pentagon_patch_triangles(BALL_R + 0.0004)      # real metres, 0.4 mm above the leather
    write_stl(stl, tris)
    print(f"pentagon patches: {len(tris)} triangles -> {stl}")
    patch_vis = ('    <visual name="pentagon_patches"><geometry><mesh filename="file://$(find football_gumming_sim)'
                 '/meshes/ball_pentagons.stl" scale="1 1 1"/></geometry><material name="patch_black"/></visual>\n')

x += f'''
  <link name="football">
    <inertial><mass value="0.43"/><inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.008"/></inertial>
    <visual name="ball"><geometry><sphere radius="{R:.5f}"/></geometry><material name="football_white"/></visual>
    <collision><geometry><sphere radius="{R:.5f}"/></geometry></collision>
{patch_vis}{"".join(seam_vis)}{"".join(guide)}  </link>''' + football_mount()

x += '\n  <ros2_control name="GazeboSimSystem" type="system">\n    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>\n'
for j in ["z_axis_joint", "tilt_joint", "left_cup_joint", "right_cup_joint"] + (["reclamp_joint"] if RECLAMP else []):
    x += (f'    <joint name="{j}"><command_interface name="position"/>'
          '<state_interface name="position"/><state_interface name="velocity"/></joint>\n')
x += '''  </ros2_control>

  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find football_gumming_sim)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
'''
x = x.replace("</parameters>\n    </plugin>", f"</parameters>\n      <position_proportional_gain>{GAIN}</position_proportional_gain>\n    </plugin>")
assert "position_proportional_gain" in x
GRAV = ""
for _ln in ("z_carriage", "nozzle_body"):
    GRAV += f'  <gazebo reference="{_ln}"><gravity>false</gravity></gazebo>\n'
_head, _sep, _tail = x.rpartition("</robot>")
assert _sep, "no </robot> found"
x = _head + GRAV + _sep + _tail
urdf.write_text(scale_urdf(x, S))

WORLD = '''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="football_gumming_robot_world">

    <physics name="physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>

    <scene>
      <ambient>0.55 0.55 0.56 1</ambient>
      <background>0.80 0.87 0.94 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- key light (casts the shadows) -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <direction>-0.5 0.3 -1</direction>
      <diffuse>0.85 0.85 0.82 1</diffuse>
      <specular>0.35 0.35 0.35 1</specular>
    </light>
    <!-- soft fill from the front-left so the operator side is not dark -->
    <light name="fill" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <direction>0.6 0.8 -0.5</direction>
      <diffuse>0.35 0.37 0.40 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>10 10</size></plane>
          </geometry>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>10 10</size></plane>
          </geometry>
          <material>
            <ambient>0.70 0.71 0.72 1</ambient>
            <diffuse>0.70 0.71 0.72 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
'''
if "--no-world" not in sys.argv:
    wpath = urdf.parent.parent / "worlds" / "football_gumming_robot.world"
    if wpath.parent.exists():
        wbak = Path(str(wpath) + ".v10.bak")
        if wpath.exists() and not wbak.exists():
            shutil.copy(wpath, wbak)
        wpath.write_text(WORLD)
        print("world written:", wpath)

# ---------- seam route planner: every one of the 90 seam edges, glued exactly once ----------
import json
from collections import deque

I3 = _I3

def plan_pass(rot, required, label):
    allowed = {e for e in edges if edge_ok(units[e[0]], units[e[1]], rot)}
    assert required <= allowed, f"{label}: {len(required - allowed)} required edges are not reachable"
    adj = {}
    for (i, j) in allowed:
        adj.setdefault(i, []).append(j)
        adj.setdefault(j, []).append(i)
    def bfs(src_v, want):
        prev = {src_v: None}
        dq = deque([src_v])
        while dq:
            c = dq.popleft()
            if want(c):
                path = []
                while c is not None:
                    path.append(c)
                    c = prev[c]
                return path[::-1]
            for n in sorted(adj.get(c, [])):
                if n not in prev:
                    prev[n] = c
                    dq.append(n)
        return None
    remaining = set(required)
    start = min({v for e in required for v in e}, key=lambda v: (abs(matvec(rot, units[v])[0]), v))
    cur, route, gum = start, [start], []
    def has_req(v):
        return any((min(v, n), max(v, n)) in remaining for n in adj[v])
    while remaining:
        path = bfs(cur, has_req)
        for a, b in zip(path[:-1], path[1:]):
            route.append(b); gum.append(False)          # travel with the valve closed
        cur = path[-1]
        opts = sorted(n for n in adj[cur] if (min(cur, n), max(cur, n)) in remaining)
        # prefer the neighbour that still has the most work left (keeps the nozzle gluing, not travelling)
        nxt = max(opts, key=lambda n: sum((min(n, m), max(n, m)) in remaining for m in adj[n]))
        remaining.discard((min(cur, nxt), max(cur, nxt)))
        route.append(nxt); gum.append(True); cur = nxt
    return route, gum

def spin_range(route, rot):
    prev, lo, hi = 0.0, 0.0, 0.0
    for v in route:
        p = matvec(rot, units[v])
        s = math.pi / 2 - math.atan2(p[2], p[1])
        while s - prev > math.pi: s -= 2 * math.pi
        while s - prev < -math.pi: s += 2 * math.pi
        prev = s; lo, hi = min(lo, s), max(hi, s)
    return lo, hi

all_edges = {(min(i, j), max(i, j)) for i, j in edges}
reach1 = {e for e in edges if edge_ok(units[e[0]], units[e[1]], I3)}
passes_def = [("Clamp 1: cups on pentagon pair A", I3, reach1, 0.0)]
if RECLAMP:
    passes_def.append(("Clamp 2: re-clamp onto hexagon pair", CLAMP_ROT, all_edges - reach1, CLAMP_ANGLE))

out_passes, covered = [], {}
for label, rot, req, ang in passes_def:
    route, gum = plan_pass(rot, set(req), label)
    lo, hi = spin_range(route, rot)
    glued = [(min(a, b), max(a, b)) for a, b, g in zip(route[:-1], route[1:], gum) if g]
    for e in glued:
        covered[e] = covered.get(e, 0) + 1
    dead = len(gum) - len(glued)
    print(f"{label}: {len(glued)} glued edges + {dead} travel edges | cup spin {lo:.1f}..{hi:.1f} rad (limit +-12.6)")
    assert max(abs(lo), abs(hi)) < 12.4, "cup spin would exceed its joint limit"
    out_passes.append({"name": label, "reclamp_angle": ang,
                       "vertices": [list(units[v]) for v in route],
                       "cup_vertices": [list(matvec(rot, units[v])) for v in route],
                       "gum": gum})
print(f"COVERAGE: {len(covered)}/90 seam edges glued, max times any edge glued: {max(covered.values())}")
assert all(c == 1 for c in covered.values())
if not RECLAMP:
    print("NOTE: single clamp only -> the 20 edges around the two cups stay unglued. Run with --reclamp for all 90.")

jpath = Path(ARGS[1]) if len(ARGS) > 1 else Path.home() / "FYP_ball_gum/seam_path.json"
p1 = out_passes[0]
jpath.write_text(json.dumps({
    "radius": BALL_R, "machine_scale": S, "ball_size": BALL_SIZE, "ball_type": BALL_TYPE, "ball_label": BALL_CATALOG[BALL_TYPE]["label"], "tilt_limit": TILT_LIMIT, "reclamp": RECLAMP, "closed": False,
    "vertices": p1["vertices"], "gum": p1["gum"], "passes": out_passes}))
print("route written:", jpath)
print("URDF written:", urdf, "| ball size:", BALL_SIZE, "| ball radius:", BALL_R, "| scale:", round(S, 4), "| reclamp joint:", RECLAMP, "| controller gain:", GAIN)
