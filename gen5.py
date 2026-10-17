import math, shutil, sys
from pathlib import Path

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
RECLAMP = "--reclamp" in sys.argv
BALL_R = 0.11
for a in sys.argv[1:]:
    if a.startswith("--ball-radius="):
        BALL_R = float(a.split("=", 1)[1])
S = BALL_R / 0.22      # whole machine is drawn at 0.22 m ball size, then scaled by S at the end
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
    text = re.sub(r'(xyz|size)="([^"]*)"', nums, text)
    text = re.sub(r'(radius|length)="([^"]*)"', nums, text)
    z = ('<limit lower="-0.20" upper="0.10" effort="60" velocity="0.15"/>',
         f'<limit lower="{-0.20 * k:.4f}" upper="{0.10 * k:.4f}" effort="60" velocity="{0.15 * k:.4f}"/>')
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

R = 0.22
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
for n, c in [("frame_gray", "0.22 0.26 0.30 1"), ("rail_silver", "0.65 0.68 0.72 1"),
             ("nozzle_orange", "0.95 0.35 0.06 1"), ("football_white", "0.92 0.92 0.88 1"),
             ("rubber_black", "0.06 0.06 0.06 1"), ("cup_red", "0.80 0.15 0.10 1"),
             ("motor_dark", "0.10 0.11 0.14 1"), ("glue_amber", "0.95 0.75 0.10 1"),
             ("seam_black", "0.03 0.03 0.03 1"), ("green_led", "0.10 0.85 0.25 1"),
             ("red_btn", "0.85 0.10 0.10 1"), ("cabinet_blue", "0.12 0.28 0.50 1")]:
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
base_extra += vbox("cabinet", "0.30 -0.25 0.45", "0.20 0.16 0.30", "cabinet_blue")
base_extra += vbox("cabinet_display", "0.30 -0.331 0.50", "0.10 0.004 0.06", "rubber_black")
base_extra += vsph("cabinet_led", "0.25 -0.335 0.40", 0.012, "green_led")
base_extra += vcyl("cabinet_estop", "0.35 -0.34 0.40", 0.02, 0.02, "red_btn", "1.5708 0 0")
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
    x += box_link(nm, 0.07, 0.07, 1.4, 3.0, "frame_gray")
    x += fixed(nm + "_mount", "base_link", nm, f"{px} {py} 0.95")
x += box_link("top_front_beam", 1.1, 0.07, 0.07, 3.0, "frame_gray") + fixed("front_beam_mount", "base_link", "top_front_beam", "0 -0.37 1.65")
x += box_link("top_back_beam", 1.1, 0.07, 0.07, 3.0, "frame_gray") + fixed("back_beam_mount", "base_link", "top_back_beam", "0 0.37 1.65")
x += box_link("top_cross_beam", 0.07, 0.74, 0.07, 3.0, "frame_gray") + fixed("cross_beam_mount", "base_link", "top_cross_beam", "0 0 1.65")
# lower frame rails
x += box_link("low_rail_front", 1.0, 0.05, 0.05, 2.0, "rail_silver") + fixed("low_rail_front_mount", "base_link", "low_rail_front", "0 -0.37 0.40")
x += box_link("low_rail_back", 1.0, 0.05, 0.05, 2.0, "rail_silver") + fixed("low_rail_back_mount", "base_link", "low_rail_back", "0 0.37 0.40")
x += box_link("low_rail_left", 0.05, 0.69, 0.05, 2.0, "rail_silver") + fixed("low_rail_left_mount", "base_link", "low_rail_left", "-0.52 0 0.40")
x += box_link("low_rail_right", 0.05, 0.69, 0.05, 2.0, "rail_silver") + fixed("low_rail_right_mount", "base_link", "low_rail_right", "0.52 0 0.40")

# nozzle vertical rail + Z motor
rail_extra = vcyl("z_motor", "0 0 0.38", 0.04, 0.09, "motor_dark")
rail_extra += vcyl("z_coupling", "0 0 0.32", 0.015, 0.05, "rail_silver")
rail_extra += vbox("guide_a", "0.0 -0.03 0", "0.01 0.01 0.60", "rail_silver")
rail_extra += vbox("guide_b", "0.0 0.03 0", "0.01 0.01 0.60", "rail_silver")
x += box_link("nozzle_rail", 0.04, 0.04, 0.60, 1.0, "rail_silver", rail_extra) + fixed("rail_mount", "base_link", "nozzle_rail", "0 0 1.35")
def _mmv(v):
    return v / 1000.0 / S
car_extra = vbox("front_plate", f"0 {-_mmv(28):.5f} 0", f"{_mmv(56):.5f} {_mmv(6):.5f} {_mmv(70):.5f}", "motor_dark")
for _bx in (-22, 22):
    for _bz in (-28, 28):
        car_extra += vcyl(f"plate_bolt_{_bx}_{_bz}", f"{_mmv(_bx):.5f} {-_mmv(31.5):.5f} {_mmv(_bz):.5f}",
                          f"{_mmv(2.5):.5f}", f"{_mmv(3):.5f}", "rail_silver", "1.5708 0 0")
x += box_link("z_carriage", 0.10, 0.10, 0.16, 1.0, "rail_silver", car_extra)
x += '''
  <joint name="z_axis_joint" type="prismatic">
    <parent link="nozzle_rail"/><child link="z_carriage"/>
    <origin xyz="0 0 0.15"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.20" upper="0.10" effort="60" velocity="0.15"/>
    <dynamics damping="4.0" friction="0.2"/>
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
noz += ncyl("needle_16G", 0, 18, 0.83, "rail_silver")           # 16 G dispensing needle, OD 1.65 mm
noz += ncyl("tip_taper", 18, 22, 1.8, "nozzle_orange")          # tapered tip
noz += ncyl("tip_taper2", 22, 27, 3.2, "nozzle_orange")
noz += ncyl("luer_hub", 27, 37, 4.5, "nozzle_orange")           # luer hub
noz += ncyl("luer_lock_collar", 37, 45, 6.5, "rail_silver")     # luer-lock collar
noz += ncyl("barrel_10cc", 45, 135, 8.2, "glue_amber")          # 10 cc syringe barrel (glue visible)
noz += ncyl("barrel_flange", 135, 137, 11.5, "motor_dark")
noz += ncyl("clamp_collar", 123, 135, 13.0, "motor_dark")       # clamp that grips the barrel
noz += ncyl("piston_cap", 137, 159, 9.5, "motor_dark")          # air-pressure adapter cap
noz += ncyl("air_fitting", 159, 167, 3.0, "rail_silver")
noz += ncyl("air_hose", 167, 175, 2.0, "rubber_black")
noz += vcyl("air_hose_out", f"{40 / 1000.0 / S:.5f} 0 {nz(175)}", mm(2.0), mm(80), "rubber_black", "0 1.5708 0")
x += f'''
  <link name="nozzle_body">
    <inertial><mass value="0.5"/>{inertia(0.5, 0.07, 0.07, 0.24)}</inertial>
{noz}  </link>''' + fixed("nozzle_mount", "z_carriage", "nozzle_body", "0 0 -0.18")

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
    <limit lower="-0.9" upper="0.9" effort="200" velocity="0.5"/>
    <dynamics damping="2.0" friction="0.1"/>
  </joint>'''

def football_mount():
    if not RECLAMP:
        return fixed("football_mount", "left_cup", "football", "0.25 0 0")
    ax = " ".join(f"{t:.6f}" for t in CLAMP_AXIS)
    return f'''
  <joint name="reclamp_joint" type="revolute">
    <parent link="left_cup"/><child link="football"/><origin xyz="0.25 0 0"/>
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
    <visual name="shaft"><origin xyz="{sign*0.09:.2f} 0 0" rpy="0 1.5708 0"/><geometry><cylinder radius="0.02" length="0.10"/></geometry><material name="rail_silver"/></visual>
    <visual name="marker"><origin xyz="0 0.05 0"/><geometry><box size="0.09 0.02 0.02"/></geometry><material name="football_white"/></visual>
  </link>
  <joint name="{side}_cup_joint" type="revolute">
    <parent link="tilt_link"/><child link="{side}_cup"/>
    <origin xyz="{sign*0.25:.2f} 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-12.6" upper="12.6" effort="50" velocity="1.5"/>
    <dynamics damping="1.0" friction="0.05"/>
  </joint>'''
x += cup("left", -1) + cup("right", 1)

x += f'''
  <link name="football">
    <inertial><mass value="0.43"/><inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.008"/></inertial>
    <visual name="ball"><geometry><sphere radius="0.22"/></geometry><material name="football_white"/></visual>
    <collision><geometry><sphere radius="0.22"/></geometry></collision>
{"".join(seam_vis)}{"".join(guide)}  </link>''' + football_mount()

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
urdf.write_text(scale_urdf(x, S))

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
    "radius": BALL_R, "tilt_limit": TILT_LIMIT, "reclamp": RECLAMP, "closed": False,
    "vertices": p1["vertices"], "gum": p1["gum"], "passes": out_passes}))
print("route written:", jpath)
print("URDF written:", urdf, "| ball radius:", BALL_R, "| scale:", round(S, 4), "| reclamp joint:", RECLAMP)
