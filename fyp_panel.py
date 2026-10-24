#!/usr/bin/env python3
# FYP touchscreen HMI (7-inch layout, 800x480).  Runs in Ubuntu and shows through WSLg next to Gazebo.
# It only starts demo.sh and reads what the run prints, so it needs nothing but python3-tk.
#   RUN      1 pick ball + size, 2 APPLY, 3 START SIMULATION, 4 RUN  |  status, progress, job card, live seam map
#   SETUP    short demo, clear gum, backup to Windows
#   RECORDS  one line per finished run (records/runs.csv), export to Excel
# E-STOP is on every screen.  (Everything demo.sh prints is still saved to panel_last.log for debugging.)
import csv
import json
import math
import os
import queue
import re
import shutil
import subprocess
import threading
import time
import tkinter as tk
from tkinter import scrolledtext, ttk

DEMO = os.environ.get("FYP_DEMO", os.path.join(os.path.expanduser("~"), "FYP_ball_gum", "demo.sh"))
WS = os.path.dirname(DEMO)
LOGFILE = os.path.join(WS, "panel_last.log")
GEN = os.path.join(WS, "gen_machine.py")
ROUTE = os.path.join(WS, "seam_path.json")
RECORDS = os.path.join(WS, "records", "runs.csv")
RECORD_COLUMNS = ["time", "run", "ball", "size", "result", "edges_glued", "edges_total", "passes", "cycle_s",
                  "wall_s", "glue_markers", "z_lift_pct", "time_scale", "glue_style"]


# ---------------------------------------------------------------- data helpers (unchanged behaviour)
def read_records():
    """All saved runs, newest first. The sequence script writes one row per finished run."""
    try:
        with open(RECORDS, newline="") as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return []
    return rows[::-1]


def add_record(row):
    """Used by the panel for runs that did not finish (stopped or failed)."""
    os.makedirs(os.path.dirname(RECORDS), exist_ok=True)
    new = not os.path.exists(RECORDS)
    with open(RECORDS, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(RECORD_COLUMNS)
        w.writerow([row.get(c, "") for c in RECORD_COLUMNS])


def windows_documents():
    """Path of the Windows Documents folder as seen from Ubuntu (None if it cannot be found)."""
    try:
        u = subprocess.run(["cmd.exe", "/c", "echo %USERNAME%"], capture_output=True, text=True, timeout=10,
                           cwd="/mnt/c").stdout.strip().replace("\r", "")
        p = f"/mnt/c/Users/{u}/Documents"
        return p if u and os.path.isdir(p) else None
    except Exception:
        return None


TITLE = "GUM APPLICATION ON SPORTS BALL SEAMS"
FOOTER = "National University of Technology, Islamabad"
BEAD_DIAMETER_MM = 1.5            # assumed glue bead width, used only for the glue-used ESTIMATE
TOTALS = {"football": 90, "volleyball": 84, "basketball": 164}      # seam edges per ball (used before the model is applied)

DEFAULT_CATALOG = {"football": {"label": "Football", "enabled": True, "sizes": [3, 4, 5]},
                   "volleyball": {"label": "Volleyball", "enabled": False, "sizes": []},
                   "basketball": {"label": "Basketball", "enabled": False, "sizes": []}}


def load_catalog():
    """Which balls and sizes exist: asked from the generator, so new balls appear here without editing the panel."""
    try:
        out = subprocess.run(["python3", GEN, "--list-balls"], capture_output=True, text=True, timeout=15).stdout
        cat = json.loads(out.strip().splitlines()[-1])
        if cat:
            return cat
    except Exception:
        pass
    return DEFAULT_CATALOG


def current_model(catalog):
    """The ball the installed model was built for (read from the last generated route file)."""
    try:
        with open(ROUTE) as f:
            j = json.load(f)
        ball, size = j.get("ball_type", "football"), int(j.get("ball_size", 5))
        if ball in catalog and size in catalog[ball]["sizes"]:
            return ball, size
    except Exception:
        pass
    ball = next((k for k, v in catalog.items() if v["enabled"]), "football")
    return ball, (catalog[ball]["sizes"] or [5])[-1]


def load_route():
    """Glue edges of the current route in gluing order, plus totals, seam length and the lengths run so far (for the
    seam map, the counters and the glue estimate)."""
    empty = {"edges": [], "total": 90, "passes": 2, "cum_m": [], "length_m": 0.0, "ball": "football", "size": 5}
    try:
        with open(ROUTE) as f:
            j = json.load(f)
    except Exception:
        return empty
    R = float(j.get("radius", 0.11))
    passes = j.get("passes") or [{"vertices": j.get("vertices", []), "gum": j.get("gum")}]
    edges, cum, run = [], [], 0.0
    for ps in passes:
        V = ps.get("vertices") or []
        G = ps.get("gum") or [True] * max(0, len(V) - 1)
        for e, g in enumerate(G):
            if g and e + 1 < len(V):
                a, b = tuple(V[e]), tuple(V[e + 1])
                d = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
                run += math.acos(d) * R
                edges.append((a, b))
                cum.append(run)
    return {"edges": edges, "total": int(j.get("total_edges") or len(edges) or 90), "passes": max(1, len(passes)),
            "cum_m": cum, "length_m": run, "ball": j.get("ball_type", "football"), "size": j.get("ball_size", 5)}


def glue_ml(length_m):
    """Estimated glue volume for a bead of BEAD_DIAMETER_MM along length_m of seam (millilitres)."""
    r = BEAD_DIAMETER_MM / 2000.0
    return length_m * math.pi * r * r * 1e6


def last_cycle(ball, size):
    """Cycle time (s) of the last completed full run for this ball and size, or None."""
    for r in read_records():
        if r.get("result") == "COMPLETED" and r.get("run") == "full" and r.get("ball") == ball and str(r.get("size")) == str(size):
            try:
                return float(r["cycle_s"])
            except (KeyError, ValueError):
                pass
    return None


# ---------------------------------------------------------------- look
BG, CARD, CARD2, EDGE = "#0e141a", "#18222c", "#202c38", "#2b3947"
TEXT, MUTED, DIM = "#eaf0f5", "#8a98a8", "#566474"
GREEN, BLUE, AMBER, RED, GREY, TEAL = "#1f9d55", "#2b6cb0", "#d99a0b", "#c62828", "#35424f", "#18a999"
GLUE, PENDING = "#f5b50a", "#33424f"
FONT = "DejaVu Sans"

STATES = {  # state -> (chip colour, headline, hint)
    "IDLE": (GREY, "Simulation is off", "Pick the ball and size, APPLY if you changed them, then START SIMULATION."),
    "STARTING": (AMBER, "Starting the simulation", "Gazebo and the controllers are loading (up to a minute)."),
    "READY": (GREEN, "Ready", "Press RUN FULL CYCLE."),
    "WORKING": (BLUE, "Preparing the run", "Moving to the first seam."),
    "RUNNING": (BLUE, "Gluing", ""),
    "RE-CLAMPING": (AMBER, "Re-clamping the ball", "The ball is released, turned and gripped on two other panels."),
    "DONE": (GREEN, "Cycle complete", ""),
    "FAULT": (RED, "Fault", "Something failed. The details are in panel_last.log (in the project folder)."),
    "STOPPED": (GREY, "Stopped", "Press START SIMULATION, then run again."),
}


class Hmi:
    def __init__(self, root):
        self.root = root
        self.proc = None
        self.q = queue.Queue()
        self.busy = False
        self.t0 = None
        self.stop_pressed = False
        self.state = "IDLE"
        self.passes_seen = 0
        self.run_key = "full"
        self.catalog = load_catalog()
        self.installed = current_model(self.catalog)      # what the machine model is built for
        self.sel_ball, self.sel_size = self.installed      # what the operator has picked (not applied yet)
        self.route = load_route()
        self.edges_now = (0, self.route["total"])
        self.ball_buttons, self.size_buttons, self.cells = {}, {}, {}
        self.run_buttons = []
        self.pct = 0.0
        self.map_ids = []
        self.map_cv = []

        root.title("Gum Application on Sports Ball Seams")
        root.configure(bg=BG)
        root.geometry("800x480")
        root.minsize(800, 480)

        # ---- header
        bar = tk.Frame(root, bg=CARD, height=44)
        bar.pack(fill="x", side="top")
        bar.pack_propagate(False)
        tk.Label(bar, text=TITLE, bg=CARD, fg=TEXT, font=(FONT, 13, "bold")).pack(side="left", padx=(14, 8))
        self.chip = tk.Label(bar, text="IDLE", bg=GREY, fg="white", font=(FONT, 11, "bold"), padx=14, pady=3)
        self.chip.pack(side="right", padx=12)
        self.clock = tk.Label(bar, text="", bg=CARD, fg=MUTED, font=(FONT, 10))
        self.clock.pack(side="right")

        # ---- footer, then navigation + E-STOP (on every screen)
        foot = tk.Frame(root, bg=BG, height=18)
        foot.pack(fill="x", side="bottom")
        foot.pack_propagate(False)
        tk.Label(foot, text=FOOTER, bg=BG, fg=DIM, font=(FONT, 8)).pack(expand=True)
        nav = tk.Frame(root, bg=CARD, height=52)
        nav.pack(fill="x", side="bottom")
        nav.pack_propagate(False)
        self.nav_btns = {}
        for name in ("RUN", "SETUP", "RECORDS"):
            b = tk.Button(nav, text=name, relief="flat", bd=0, font=(FONT, 12, "bold"), width=10, cursor="hand2",
                          command=(lambda n=name: self.show(n)))
            b.pack(side="left", fill="y", padx=(0, 2))
            self.nav_btns[name] = b
        self.estop = tk.Button(nav, text="E-STOP", bg=RED, fg="white", activebackground="#e53935", activeforeground="white",
                               relief="flat", bd=0, font=(FONT, 14, "bold"), width=11, cursor="hand2",
                               highlightbackground="#f2c230", highlightthickness=3, command=lambda: self.run("stop"))
        self.estop.pack(side="right", fill="y", padx=(2, 0))

        body = tk.Frame(root, bg=BG)
        body.pack(fill="both", expand=True)
        self.pages = {n: tk.Frame(body, bg=BG) for n in ("RUN", "SETUP", "RECORDS")}
        self.build_run(self.pages["RUN"])
        self.build_setup(self.pages["SETUP"])
        self.build_records(self.pages["RECORDS"])

        self.refresh_selector()
        self.reset_tiles()
        self.set_state("IDLE")
        self.show("RUN")
        self.say("Panel started. Press START SIMULATION, wait for READY, then press RUN FULL CYCLE.\n")
        root.after(100, self.poll)
        root.after(500, self.tick)

    # ------------------------------------------------------------ small widgets
    def card(self, parent, title=None):
        f = tk.Frame(parent, bg=CARD, highlightbackground=EDGE, highlightthickness=1)
        if title:
            tk.Label(f, text=title, bg=CARD, fg=MUTED, font=(FONT, 8, "bold")).pack(anchor="w", padx=10, pady=(6, 0))
        return f

    def button(self, parent, text, color, cmd, size=12, pady=10, **kw):
        return tk.Button(parent, text=text, bg=color, fg="white", activebackground=color, activeforeground="white",
                         disabledforeground="#7d8a98", relief="flat", bd=0, font=(FONT, size, "bold"), pady=pady,
                         cursor="hand2", command=cmd, **kw)

    # ------------------------------------------------------------ RUN screen
    def build_run(self, page):
        page.columnconfigure(0, weight=3)
        page.columnconfigure(1, weight=2, minsize=290)
        page.rowconfigure(0, weight=1)
        left = tk.Frame(page, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(10, 5), pady=(8, 4))
        right = tk.Frame(page, bg=BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(5, 10), pady=(8, 4))

        # 1-2: pick ball and size, APPLY
        sel = self.card(left)
        sel.pack(fill="x")
        row1 = tk.Frame(sel, bg=CARD)
        row1.pack(fill="x", padx=8, pady=(7, 3))
        tk.Label(row1, text="1  BALL", bg=CARD, fg=MUTED, font=(FONT, 8, "bold"), width=8, anchor="w").pack(side="left")
        for key, info in self.catalog.items():
            b = tk.Button(row1, text=info["label"].upper() + ("" if info["enabled"] else " (soon)"), relief="flat", bd=0,
                          font=(FONT, 10, "bold"), pady=6, cursor="hand2", command=(lambda k=key: self.pick_ball(k)))
            b.pack(side="left", fill="x", expand=True, padx=2)
            self.ball_buttons[key] = b
        row2 = tk.Frame(sel, bg=CARD)
        row2.pack(fill="x", padx=8, pady=(0, 7))
        tk.Label(row2, text="2  SIZE", bg=CARD, fg=MUTED, font=(FONT, 8, "bold"), width=8, anchor="w").pack(side="left")
        self.row2 = tk.Frame(row2, bg=CARD)
        self.row2.pack(side="left", fill="x", expand=True)
        self.apply_btn = self.button(row2, "APPLY", BLUE, self.apply_selection, 10, 6)
        self.apply_btn.pack(side="right", padx=(6, 2))

        # status + progress
        st = self.card(left)
        st.pack(fill="x", pady=(6, 0))
        top = tk.Frame(st, bg=CARD)
        top.pack(fill="x", padx=12, pady=(6, 0))
        self.head = tk.Label(top, text="", bg=CARD, fg=TEXT, font=(FONT, 16, "bold"), anchor="w")
        self.head.pack(side="left")
        self.count = tk.Label(top, text="", bg=CARD, fg=TEXT, font=(FONT, 16, "bold"))
        self.count.pack(side="right")
        mid = tk.Frame(st, bg=CARD)
        mid.pack(fill="x", padx=12)
        self.hint = tk.Label(mid, text="", bg=CARD, fg=MUTED, font=(FONT, 9), anchor="w", justify="left", wraplength=330)
        self.hint.pack(side="left", fill="x", expand=True)
        self.eta = tk.Label(mid, text="", bg=CARD, fg=MUTED, font=(FONT, 9))
        self.eta.pack(side="right")
        self.bar = tk.Canvas(st, height=18, bg="#0b1015", highlightthickness=0)
        self.bar.pack(fill="x", padx=12, pady=(3, 8))
        self.bar.bind("<Configure>", lambda e: self.draw_bar())

        # job card
        job = self.card(left)
        job.pack(fill="x", pady=(6, 0))
        names = [("job", "JOB"), ("seam", "SEAM"), ("glue", "GLUE USED (ESTIMATE)"),
                 ("time", "ELAPSED"), ("pass", "PASS"), ("last", "LAST CYCLE")]
        for i, (key, title) in enumerate(names):
            job.columnconfigure(i % 3, weight=1, uniform="j")
            c = tk.Frame(job, bg=CARD)
            c.grid(row=i // 3, column=i % 3, sticky="w", padx=(10, 4), pady=(4 if i < 3 else 0, 4))
            tk.Label(c, text=title, bg=CARD, fg=MUTED, font=(FONT, 7, "bold")).pack(anchor="w")
            v = tk.Label(c, text="-", bg=CARD, fg=TEXT, font=(FONT, 10, "bold"))
            v.pack(anchor="w")
            self.cells[key] = v

        # 3-4: START and RUN
        act = tk.Frame(left, bg=BG)
        act.pack(fill="x", side="bottom")
        act.columnconfigure(0, weight=1, uniform="a")
        act.columnconfigure(1, weight=1, uniform="a")
        self.btn_start = self.button(act, "3  START SIMULATION", GREEN, lambda: self.run("start"), 11, 11)
        self.btn_start.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.btn_run = self.button(act, "4  RUN FULL CYCLE", BLUE, lambda: self.run("full"), 11, 11)
        self.btn_run.grid(row=0, column=1, sticky="ew", padx=(4, 0))
        self.run_buttons += [self.btn_start, self.btn_run]

        # seam map
        mp = self.card(right, "LIVE SEAM MAP  (two sides of the ball)")
        mp.pack(fill="both", expand=True)
        views = tk.Frame(mp, bg=CARD)
        views.pack(pady=(6, 0))
        for _ in range(2):
            cv = tk.Canvas(views, width=136, height=140, bg=CARD, highlightthickness=0)
            cv.pack(side="left", padx=2)
            self.map_cv.append(cv)
        leg = tk.Frame(mp, bg=CARD)
        leg.pack(fill="x", padx=12, pady=(4, 2))
        for col, txt in ((GLUE, "glued"), (PENDING, "still to do")):
            tk.Label(leg, text="\u25cf", bg=CARD, fg=col, font=(FONT, 11)).pack(side="left")
            tk.Label(leg, text=txt, bg=CARD, fg=MUTED, font=(FONT, 9)).pack(side="left", padx=(2, 12))
        self.model_lbl = tk.Label(mp, text="", bg=CARD, fg=MUTED, font=(FONT, 9), anchor="w", justify="left", wraplength=270)
        self.model_lbl.pack(fill="x", padx=12, pady=(2, 8), side="bottom")
        self.build_map()

    def build_map(self):
        """Project every glue edge onto two discs (front and back of the ball) once; colouring is changed later."""
        for cv in self.map_cv:
            cv.delete("all")
        edges = self.route["edges"]
        self.map_ids = [[] for _ in edges]
        v = self.unit((0.55, 0.45, 0.70))
        for k, cv in enumerate(self.map_cv):
            d = v if k == 0 else tuple(-c for c in v)
            right = self.unit(self.cross((0.0, 0.0, 1.0), d))
            up = self.cross(d, right)
            cx, cy, r = 68, 66, 62
            cv.create_oval(cx - r, cy - r, cx + r, cy + r, fill="#0d1318", outline=EDGE)
            cv.create_text(cx, 134, text="FRONT" if k == 0 else "BACK", fill=DIM, font=(FONT, 7, "bold"))
            for i, (a, b) in enumerate(edges):
                pts = []
                for t in range(5):
                    p = self.unit(tuple(a[m] * (1 - t / 4.0) + b[m] * (t / 4.0) for m in range(3)))
                    pts.append((self.dot(p, d) > 0.02, cx + r * self.dot(p, right), cy - r * self.dot(p, up)))
                for s0, s1 in zip(pts[:-1], pts[1:]):
                    if s0[0] and s1[0]:
                        self.map_ids[i].append((cv, cv.create_line(s0[1], s0[2], s1[1], s1[2], fill=PENDING, width=2,
                                                                   capstyle="round")))
        self._painted = None
        self.paint_map(0)

    @staticmethod
    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @staticmethod
    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    @staticmethod
    def unit(a):
        n = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2) or 1.0
        return (a[0] / n, a[1] / n, a[2] / n)

    def paint_map(self, n_glued):
        n_glued = min(max(0, int(n_glued)), len(self.map_ids))
        if getattr(self, "_painted", None) == n_glued:
            return
        self._painted = n_glued
        for i, items in enumerate(self.map_ids):
            for cv, it in items:
                cv.itemconfigure(it, fill=GLUE if i < n_glued else PENDING)

    # ------------------------------------------------------------ SETUP screen (tools)
    def build_setup(self, page):
        page.columnconfigure(0, weight=1)
        page.columnconfigure(1, weight=1)
        tools = self.card(page, "TOOLS")
        tools.grid(row=0, column=0, sticky="nsew", padx=(10, 5), pady=10)
        self.btn_short = self.button(tools, "RUN SHORT DEMO  (12 seams)", BLUE, lambda: self.run("short"), 11, 12)
        self.btn_clear = self.button(tools, "CLEAR GUM FROM THE BALL", AMBER, lambda: self.run("clear"), 11, 12)
        self.btn_backup = self.button(tools, "BACKUP PROJECT TO WINDOWS", GREY, lambda: self.run("backup"), 11, 12)
        for b in (self.btn_short, self.btn_clear, self.btn_backup):
            b.pack(fill="x", padx=12, pady=(10, 0))
        self.run_buttons += [self.btn_short, self.btn_clear, self.btn_backup]
        about = self.card(page, "MACHINE")
        about.grid(row=0, column=1, sticky="nsew", padx=(5, 10), pady=10)
        self.about_lbl = tk.Label(about, text="", bg=CARD, fg=TEXT, font=(FONT, 10), anchor="nw", justify="left", wraplength=330)
        self.about_lbl.pack(fill="both", expand=True, padx=12, pady=10)

    # ------------------------------------------------------------ ball + size selection (RUN screen)
    def pick_ball(self, key):
        if self.busy or not self.catalog[key]["enabled"]:
            return
        self.sel_ball = key
        sizes = self.catalog[key]["sizes"]
        if self.sel_size not in sizes and sizes:
            self.sel_size = sizes[-1]
        self.selection_changed()

    def pick_size(self, n):
        if self.busy:
            return
        self.sel_size = n
        self.selection_changed()

    def sel_total(self):
        """Seam edges of the SELECTED ball (known as soon as it is picked, before it is applied)."""
        if self.sel_ball == self.installed[0]:
            return self.route["total"]
        info = self.catalog.get(self.sel_ball, {})
        return int(info.get("edges") or TOTALS.get(self.sel_ball, self.route["total"]))

    def selection_changed(self):
        self.refresh_selector()
        if self.state in ("IDLE", "READY", "DONE", "STOPPED", "FAULT"):
            self.reset_tiles()
            self.set_state(self.state if self.state != "DONE" else "READY")

    def refresh_selector(self):
        for key, b in self.ball_buttons.items():
            en = self.catalog[key]["enabled"]
            on = key == self.sel_ball
            b.configure(bg=BLUE if on else (GREY if en else "#1d2630"), fg="white" if en else "#667584",
                        activebackground=BLUE if on else GREY, activeforeground="white", disabledforeground="#667584",
                        state="disabled" if (self.busy or not en) else "normal")
        for w in self.row2.winfo_children():
            w.destroy()
        self.size_buttons = {}
        sizes = self.catalog[self.sel_ball]["sizes"]
        for i, n in enumerate(sizes):
            on = n == self.sel_size
            self.row2.columnconfigure(i, weight=1, uniform="s")
            text = f"SIZE {n}" + (" (STANDARD)" if len(sizes) == 1 else "")
            b = tk.Button(self.row2, text=text, bg=BLUE if on else GREY, fg="white",
                          activebackground=BLUE if on else GREY, activeforeground="white", disabledforeground="#9aa7b5",
                          relief="flat", bd=0, font=(FONT, 10, "bold"), pady=6, cursor="hand2",
                          state="disabled" if self.busy else "normal", command=(lambda m=n: self.pick_size(m)))
            b.grid(row=0, column=i, sticky="ew", padx=2)
            self.size_buttons[n] = b
        inst_ball, inst_size = self.installed
        same = (inst_ball, inst_size) == (self.sel_ball, self.sel_size)
        self.apply_btn.configure(bg=GREY if same else AMBER, activebackground=GREY if same else AMBER,
                                 state="disabled" if (self.busy or same) else "normal")
        self.update_run_buttons()
        label = self.catalog[inst_ball]["label"]
        if same:
            self.model_lbl.configure(text=f"{label} | size {inst_size} | {self.route['total']} seam edges", fg=MUTED)
        else:
            self.model_lbl.configure(text=f"The map shows the {label.lower()} that is installed. Press APPLY to switch to "
                                          f"{self.catalog[self.sel_ball]['label'].lower()} size {self.sel_size}.", fg=AMBER)
        self.about_lbl.configure(text=(f"Ball: {label}, size {inst_size}\nSeam edges: {self.route['total']}\n"
                                       f"Clamps per ball: {self.route['passes']}\nSeam length: {self.route['length_m']:.2f} m\n\n"
                                       "Simulation only. The glue amount is an estimate from the seam length "
                                       f"and an assumed {BEAD_DIAMETER_MM} mm bead."))

    def apply_selection(self):
        if self.busy or (self.sel_ball, self.sel_size) == self.installed:
            return
        self.run("select", self.sel_ball, str(self.sel_size))
        self.show("RUN")

    # ------------------------------------------------------------ RECORDS screen
    def build_records(self, page):
        cards = tk.Frame(page, bg=BG)
        cards.pack(fill="x", padx=10, pady=(10, 4))
        self.rec_cards = {}
        for i, (key, title) in enumerate([("today", "RUNS TODAY"), ("ok", "COMPLETED"), ("cycle", "AVG CYCLE"), ("edges", "EDGES GLUED")]):
            cards.columnconfigure(i, weight=1, uniform="r")
            f = self.card(cards, title)
            f.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 3, 0))
            v = tk.Label(f, text="-", bg=CARD, fg=TEXT, font=(FONT, 16, "bold"))
            v.pack(anchor="w", padx=10, pady=(0, 7))
            self.rec_cards[key] = v
        st = ttk.Style(self.root)
        st.theme_use("clam")
        st.configure("Rec.Treeview", background="#0f151b", fieldbackground="#0f151b", foreground=TEXT, rowheight=26,
                     font=(FONT, 9), borderwidth=0)
        st.configure("Rec.Treeview.Heading", background=CARD, foreground=MUTED, font=(FONT, 8, "bold"), relief="flat")
        st.map("Rec.Treeview", background=[("selected", BLUE)])
        cols = [("time", "TIME", 130), ("ball", "BALL", 80), ("size", "SIZE", 42), ("run", "RUN", 50), ("result", "RESULT", 90),
                ("edges", "EDGES", 70), ("cycle", "CYCLE", 60), ("zlift", "Z LIFT", 55), ("glue", "GLUE", 55)]
        foot = tk.Frame(page, bg=BG)
        foot.pack(fill="x", side="bottom", padx=10, pady=(0, 8))
        for text, color, cmd in [("EXPORT TO WINDOWS (Excel)", BLUE, self.export_records), ("REFRESH", GREY, self.fill_records)]:
            self.button(foot, text, color, cmd, 10, 7).pack(side="left", padx=(0, 6))
        self.rec_msg = tk.Label(foot, text="", bg=BG, fg=MUTED, font=(FONT, 9))
        self.rec_msg.pack(side="left", padx=6)
        box = tk.Frame(page, bg=BG)
        box.pack(fill="both", expand=True, padx=10, pady=4)
        self.tree = ttk.Treeview(box, columns=[c[0] for c in cols], show="headings", style="Rec.Treeview")
        for key, title, width in cols:
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, anchor="w")
        sb = ttk.Scrollbar(box, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.tree.tag_configure("bad", foreground="#ff8a80")

    def fill_records(self):
        rows = read_records()
        self.tree.delete(*self.tree.get_children())
        for r in rows[:200]:
            result = r.get("result", "")
            cycle = r.get("cycle_s") or ""
            try:
                cyc_txt = f"{int(float(cycle)) // 60}:{int(float(cycle)) % 60:02d}" if cycle else "-"
            except ValueError:
                cyc_txt = "-"
            self.tree.insert("", "end", tags=(() if result == "COMPLETED" else ("bad",)), values=(
                r.get("time", ""), (r.get("ball", "") or "").capitalize(), r.get("size", ""), r.get("run", ""), result,
                f"{r.get('edges_glued', '')}/{r.get('edges_total', '')}", cyc_txt,
                (f"{r['z_lift_pct']}%" if r.get("z_lift_pct") else "-"), r.get("glue_style", "") or "-"))
        today = time.strftime("%Y-%m-%d")
        done = [r for r in rows if r.get("result") == "COMPLETED"]
        cyc = []
        for r in done:
            try:
                if r.get("cycle_s"):
                    cyc.append(float(r["cycle_s"]))
            except ValueError:
                pass
        edges = 0
        for r in done:
            try:
                edges += int(r.get("edges_glued") or 0)
            except ValueError:
                pass
        self.rec_cards["today"].configure(text=str(sum(1 for r in rows if (r.get("time") or "").startswith(today))))
        self.rec_cards["ok"].configure(text=(f"{len(done)} of {len(rows)}" if rows else "-"),
                                       fg=GREEN if rows and len(done) == len(rows) else TEXT)
        self.rec_cards["cycle"].configure(text=(f"{sum(cyc) / len(cyc) / 60:.1f} min" if cyc else "-"))
        self.rec_cards["edges"].configure(text=str(edges) if rows else "-")
        self.rec_msg.configure(text="" if rows else "No runs recorded yet. Finish a run and it appears here.", fg=MUTED)

    def export_records(self):
        if not os.path.exists(RECORDS):
            self.rec_msg.configure(text="Nothing to export yet.", fg=AMBER)
            return
        docs = windows_documents()
        if not docs:
            self.rec_msg.configure(text=f"Windows Documents not found. The file is: {RECORDS}", fg=AMBER)
            return
        dest = os.path.join(docs, "FYP_records_" + time.strftime("%Y%m%d_%H%M") + ".csv")
        shutil.copyfile(RECORDS, dest)
        self.rec_msg.configure(text="Saved to Documents\\" + os.path.basename(dest), fg=GREEN)

    # ------------------------------------------------------------ navigation + state
    def show(self, name):
        for n, p in self.pages.items():
            p.pack_forget()
        self.pages[name].pack(fill="both", expand=True)
        for n, b in self.nav_btns.items():
            on = n == name
            b.configure(bg=BLUE if on else CARD2, fg="white" if on else MUTED, activebackground=BLUE if on else CARD2,
                        activeforeground="white")
        if name == "RECORDS":
            self.fill_records()

    def set_state(self, name):
        self.state = name
        colour, head, hint = STATES.get(name, (GREY, name, ""))
        self.chip.configure(text=name, bg=colour)
        self.head.configure(text=head, fg=TEXT if name != "FAULT" else "#ff8a80")
        same = (self.sel_ball, self.sel_size) == self.installed
        if name == "RUNNING":
            hint = f"Pass {max(1, self.passes_seen)} of {self.run_passes()}: the nozzle follows the seams and lifts over glued ones."
        elif name == "DONE":
            a, b = self.edges_now
            hint = f"{a} of {b} seam edges glued."
        elif not same and name in ("IDLE", "READY", "STOPPED", "FAULT"):
            hint = (f"You picked {self.catalog[self.sel_ball]['label']} size {self.sel_size}: press APPLY "
                    "(the model is rebuilt and the simulation restarts, about a minute).")
        self.hint.configure(text=hint, fg=MUTED)
        self.update_run_buttons()

    def update_run_buttons(self):
        """The next step is the coloured button: APPLY (if the pick changed), START (if the simulation is off), RUN."""
        up = self.state in ("READY", "DONE", "RUNNING", "RE-CLAMPING", "WORKING")
        same = (self.sel_ball, self.sel_size) == self.installed
        self.btn_start.configure(bg=GREEN if (not up and same) else GREY, activebackground=GREEN if (not up and same) else GREY)
        self.btn_run.configure(bg=BLUE if (up and same) else GREY, activebackground=BLUE if (up and same) else GREY)

    def run_passes(self):
        return 1 if self.run_key == "short" else self.route["passes"]

    def cell(self, key, text, color=TEXT):
        self.cells[key].configure(text=text, fg=color)

    def reset_tiles(self):
        r = self.route
        total = self.sel_total()
        inst_same = self.sel_ball == self.installed[0]
        label = self.catalog[self.sel_ball]["label"]
        self.cell("job", f"{label} | size {self.sel_size}")
        if inst_same:
            self.cell("seam", f"{r['total']} edges | {r['length_m']:.1f} m")
            self.cell("glue", f"0.0 / {glue_ml(r['length_m']):.1f} mL")
        else:
            self.cell("seam", f"{total} edges", MUTED)
            self.cell("glue", "after APPLY", MUTED)
        self.cell("time", "00:00")
        self.cell("pass", "-")
        lc = last_cycle(self.sel_ball, self.sel_size)
        self.cell("last", f"{int(lc) // 60}:{int(lc) % 60:02d}" if lc else "-")
        self.eta.configure(text="")
        self.set_progress(0, total)

    def set_progress(self, a, b):
        b = max(1, b)
        self.edges_now = (a, b)
        self.pct = min(1.0, a / b)
        self.count.configure(text=f"{a} / {b}")
        self.draw_bar()
        self.paint_map(a)
        r = self.route
        if r["cum_m"] and self.sel_ball == self.installed[0] and b == r["total"]:     # full run: use the real seam lengths
            used = r["cum_m"][min(a, len(r["cum_m"])) - 1] if a > 0 else 0.0
            self.cell("glue", f"{glue_ml(used):.1f} / {glue_ml(r['length_m']):.1f} mL")
        elif r["cum_m"] and a > 0 and self.sel_ball == self.installed[0]:               # short demo: first edges only
            used = r["cum_m"][min(a, len(r["cum_m"])) - 1]
            self.cell("glue", f"{glue_ml(used):.1f} mL")

    def draw_bar(self):
        self.bar.delete("all")
        w = max(self.bar.winfo_width(), 10)
        self.bar.create_rectangle(0, 0, w * self.pct, 18, fill=TEAL, width=0)
        self.bar.create_text(w / 2, 9, text=f"{self.pct * 100:.0f} %", fill="white", font=(FONT, 9, "bold"))

    def set_busy(self, busy):
        self.busy = busy
        for b in self.run_buttons:
            b.configure(state="disabled" if busy else "normal")
        self.refresh_selector()

    # ------------------------------------------------------------ log file (kept for debugging, no screen)
    def say(self, text):
        try:
            with open(LOGFILE, "a") as f:
                f.write(text)
        except OSError:
            pass

    # ------------------------------------------------------------ running demo.sh
    def run(self, key, *args):
        if key == "stop":
            self.stop_pressed = True
            if self.proc and self.proc.poll() is None:
                self.proc.terminate()
        elif self.busy:
            return
        if key in ("short", "full") and (self.sel_ball, self.sel_size) != self.installed:
            self.set_state(self.state)            # the hint tells the operator to press APPLY first
            self.hint.configure(text="You changed the ball or size: press APPLY first, then run.", fg=AMBER)
            return
        if key in ("short", "full"):
            try:
                open(LOGFILE, "w").close()
            except OSError:
                pass
        self.say(f"\n--- {key} ---\n")
        if key in ("short", "full"):
            self.run_key = key
            self.route = load_route()
            self.build_map()
            self.reset_tiles()
            self.set_progress(0, self.route["total"] if key == "full" else 12)
            self.passes_seen = 0
            self.stop_pressed = False
            self.run_t0 = time.time()
            self.t0 = time.monotonic()
            self.show("RUN")
            self.set_state("WORKING")
        elif key in ("start", "select"):
            self.set_state("STARTING")
        elif key == "stop":
            self.set_state("STOPPED")
            self.t0 = None
        self.set_busy(key != "stop")
        threading.Thread(target=self.worker, args=(key, args), daemon=True).start()

    def worker(self, key, args=()):
        try:
            self.proc = subprocess.Popen(["bash", DEMO, key, *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         text=True, bufsize=1)
            for line in self.proc.stdout:
                self.q.put(("line", line))
            code = self.proc.wait()
        except Exception as exc:
            self.q.put(("line", f"Could not run {DEMO}: {exc}\n"))
            code = 1
        self.q.put(("done", (key, code)))

    def read_line(self, line):
        s = line.strip()
        if s.startswith("READY"):
            self.set_state("READY")
            self.set_busy(False)          # unlock the buttons as soon as the simulation says READY
        elif s.startswith("FAILED"):
            self.set_state("FAULT")
        elif s.startswith("Settings:"):
            self.set_state("RUNNING")
        elif "Tracing seams" in s:
            self.passes_seen += 1
            self.cell("pass", f"{self.passes_seen} of {self.run_passes()}")
            self.set_state("RUNNING")
        elif s.startswith("RE-CLAMP"):
            self.set_state("RE-CLAMPING")
        elif s.startswith("PROGRESS"):
            m = re.search(r"glued=(\d+)/(\d+)", s)
            if m:
                self.set_progress(int(m.group(1)), int(m.group(2)))
        elif s.startswith("Done"):
            self.set_state("DONE")

    def poll(self):
        try:
            while True:
                kind, val = self.q.get_nowait()
                if kind == "line":
                    self.say(val)
                    self.read_line(val)
                else:
                    self.finish(*val)
        except queue.Empty:
            pass
        self.root.after(100, self.poll)

    def finish(self, key, code):
        self.set_busy(False)
        if key in ("short", "full"):
            if code != 0:               # the sequence only records runs that finished, so record this one here
                ball, size = self.installed
                try:
                    add_record({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "run": key, "ball": ball, "size": size,
                                "result": "STOPPED" if self.stop_pressed else "FAILED",
                                "edges_glued": self.edges_now[0], "edges_total": self.edges_now[1],
                                "wall_s": int(time.time() - getattr(self, "run_t0", time.time()))})
                except OSError:
                    pass
            elapsed = int(time.monotonic() - self.t0) if self.t0 else 0
            self.t0 = None
            self.set_state("DONE" if code == 0 else "STOPPED")
            if code == 0:
                self.hint.configure(text=f"{self.edges_now[0]} of {self.edges_now[1]} seam edges glued in "
                                         f"{elapsed // 60}:{elapsed % 60:02d}.  Saved under RECORDS.")
                lc = last_cycle(*self.installed)
                self.cell("last", f"{int(lc) // 60}:{int(lc) % 60:02d}" if lc else "-")
            self.eta.configure(text="")
        elif key in ("start", "select"):
            if key == "select":      # always trust the files on disk, even if the restart failed
                self.installed = current_model(self.catalog)
                self.sel_ball, self.sel_size = self.installed
                self.route = load_route()
                self.build_map()
                self.refresh_selector()
                self.reset_tiles()
            self.set_state("READY" if code == 0 else "FAULT")
        elif key == "stop":
            self.set_state("STOPPED")
        elif key in ("clear", "backup"):
            self.set_state("READY" if code == 0 else "FAULT")

    def tick(self):
        self.clock.configure(text=time.strftime("%H:%M  "))
        if self.t0 is not None:
            e = int(time.monotonic() - self.t0)
            self.cell("time", f"{e // 60:02d}:{e % 60:02d}")
            if self.pct >= 0.03 and self.state in ("RUNNING", "RE-CLAMPING"):
                rest = int(e * (1 - self.pct) / self.pct)
                self.eta.configure(text=f"about {rest // 60}:{rest % 60:02d} left")
        self.root.after(500, self.tick)


if __name__ == "__main__":
    root = tk.Tk()
    Hmi(root)
    root.mainloop()
