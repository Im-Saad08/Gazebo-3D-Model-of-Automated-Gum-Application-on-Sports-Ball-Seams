#!/usr/bin/env python3
# FYP touchscreen-style HMI (runs in Ubuntu, shows through WSLg next to Gazebo).
# It only starts demo.sh and reads what the run prints, so it needs nothing but python3-tk.
import csv
import json
import os
import queue
import re
import subprocess
import threading
import time
import shutil
import tkinter as tk
from tkinter import scrolledtext, ttk

DEMO = os.environ.get("FYP_DEMO", os.path.join(os.path.expanduser("~"), "FYP_ball_gum", "demo.sh"))

LOGFILE = os.path.join(os.path.dirname(DEMO), "panel_last.log")
WS = os.path.dirname(DEMO)
GEN = os.path.join(WS, "gen_machine.py")
ROUTE = os.path.join(WS, "seam_path.json")
RECORDS = os.path.join(WS, "records", "runs.csv")
RECORD_COLUMNS = ["time", "run", "ball", "size", "result", "edges_glued", "edges_total", "passes", "cycle_s",
                  "wall_s", "glue_markers", "z_lift_pct", "time_scale", "glue_style"]


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


BG, CARD, EDGE = "#10161c", "#1a232d", "#2b3947"
TEXT, MUTED = "#e9eef3", "#8a98a8"
GREEN, BLUE, AMBER, RED, GREY, TEAL = "#1f9d55", "#2b6cb0", "#d99a0b", "#c62828", "#3a4757", "#18a999"
FONT = "DejaVu Sans"

STATES = {  # state name -> chip colour
    "IDLE": GREY, "STARTING": AMBER, "READY": GREEN, "RUNNING": BLUE, "RE-CLAMPING": AMBER,
    "DONE": GREEN, "FAULT": RED, "STOPPED": GREY, "WORKING": BLUE,
}


class Hmi:
    def __init__(self, root):
        self.root = root
        self.proc = None
        self.q = queue.Queue()
        self.busy = False
        self.t0 = None
        self.passes_seen = 0
        self.edges_now = (0, 90)
        self.stop_pressed = False
        self.rec_win = None
        self.catalog = load_catalog()
        self.installed = current_model(self.catalog)      # what the running machine model is built for
        self.sel_ball, self.sel_size = self.installed       # what the operator has picked (not applied yet)
        self.ball_buttons, self.size_buttons = {}, {}
        root.title("FYP HMI - Vision-Guided Gum Application (simulation)")
        root.configure(bg=BG)
        root.geometry("900x700")
        root.minsize(800, 620)

        # ---- header bar
        bar = tk.Frame(root, bg=CARD, height=54)
        bar.pack(fill="x")
        bar.pack_propagate(False)
        tk.Label(bar, text="GUM APPLICATOR", bg=CARD, fg=TEXT, font=(FONT, 16, "bold")).pack(side="left", padx=(18, 6))
        self.sub = tk.Label(bar, text="", bg=CARD, fg=MUTED, font=(FONT, 11))
        self.sub.pack(side="left")
        self.chip = tk.Label(bar, text="IDLE", bg=GREY, fg="white", font=(FONT, 13, "bold"), padx=18, pady=4)
        self.chip.pack(side="right", padx=18)
        self.clock = tk.Label(bar, text="", bg=CARD, fg=MUTED, font=(FONT, 11))
        self.clock.pack(side="right")

        main = tk.Frame(root, bg=BG)
        main.pack(fill="both", expand=True, padx=16, pady=12)
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=1, minsize=230)
        main.rowconfigure(0, weight=1)

        # ---- left: status tiles + progress + log
        left = tk.Frame(main, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        # ---- operator setup: ball type, size, apply
        sel = tk.Frame(left, bg=CARD, highlightbackground=EDGE, highlightthickness=1)
        sel.pack(fill="x", padx=4, pady=(0, 6))
        tk.Label(sel, text="OPERATOR SETUP  -  ball and size", bg=CARD, fg=MUTED, font=(FONT, 9, "bold")).pack(anchor="w", padx=12, pady=(8, 4))
        row1 = tk.Frame(sel, bg=CARD)
        row1.pack(fill="x", padx=12, pady=2)
        tk.Label(row1, text="BALL", bg=CARD, fg=MUTED, font=(FONT, 9, "bold"), width=6, anchor="w").pack(side="left")
        for key, info in self.catalog.items():
            b = tk.Button(row1, text=info["label"].upper() + ("" if info["enabled"] else "\n(not available yet)"), relief="flat", bd=0,
                          font=(FONT, 10, "bold"), padx=10, pady=6, cursor="hand2",
                          command=(lambda k=key: self.pick_ball(k)))
            b.pack(side="left", padx=3)
            self.ball_buttons[key] = b
        self.row2 = tk.Frame(sel, bg=CARD)
        self.row2.pack(fill="x", padx=12, pady=2)
        row3 = tk.Frame(sel, bg=CARD)
        row3.pack(fill="x", padx=12, pady=(4, 10))
        self.apply_btn = tk.Button(row3, text="APPLY AND RESTART SIMULATION", bg=BLUE, fg="white", activebackground=BLUE,
                                   activeforeground="white", relief="flat", bd=0, font=(FONT, 11, "bold"), padx=14, pady=7,
                                   cursor="hand2", command=self.apply_selection)
        self.apply_btn.pack(side="left")
        self.model_lbl = tk.Label(row3, text="", bg=CARD, fg=MUTED, font=(FONT, 10))
        self.model_lbl.pack(side="left", padx=12)
        self.refresh_selector()

        grid = tk.Frame(left, bg=BG)
        grid.pack(fill="x")
        for c in range(3):
            grid.columnconfigure(c, weight=1, uniform="t")
        self.tiles = {}
        for i, (key, title) in enumerate([("pass", "PASS"), ("edges", "EDGES GLUED"), ("time", "ELAPSED"),
                                          ("nozzle", "NOZZLE"), ("valve", "GLUE VALVE"), ("ztrack", "Z LIFT CHECK")]):
            f = tk.Frame(grid, bg=CARD, highlightbackground=EDGE, highlightthickness=1)
            f.grid(row=i // 3, column=i % 3, sticky="nsew", padx=4, pady=4)
            tk.Label(f, text=title, bg=CARD, fg=MUTED, font=(FONT, 9, "bold")).pack(anchor="w", padx=12, pady=(8, 0))
            v = tk.Label(f, text="-", bg=CARD, fg=TEXT, font=(FONT, 20, "bold"))
            v.pack(anchor="w", padx=12, pady=(0, 10))
            self.tiles[key] = v

        pf = tk.Frame(left, bg=CARD, highlightbackground=EDGE, highlightthickness=1)
        pf.pack(fill="x", padx=4, pady=(6, 4))
        tk.Label(pf, text="SEAM COVERAGE", bg=CARD, fg=MUTED, font=(FONT, 9, "bold")).pack(anchor="w", padx=12, pady=(8, 2))
        self.bar = tk.Canvas(pf, height=26, bg="#0d1217", highlightthickness=0)
        self.bar.pack(fill="x", padx=12, pady=(0, 12))
        self.bar.bind("<Configure>", lambda e: self.draw_bar())
        self.pct = 0.0

        self.log = scrolledtext.ScrolledText(left, bg="#0b0f13", fg="#b9c6d3", insertbackground="white",
                                             font=("DejaVu Sans Mono", 9), relief="flat", height=8)
        self.log.pack(fill="both", expand=True, padx=4, pady=(4, 0))
        self.say("Press START, wait for READY, then run the demo.\n")

        # ---- right: touch buttons
        right = tk.Frame(main, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")
        self.buttons = {}
        for key, label, color in [("start", "START SIMULATION", GREEN), ("short", "RUN SHORT DEMO", BLUE),
                                  ("full", "RUN FULL SEAM LOOP", BLUE), ("clear", "CLEAR GUM", AMBER),
                                  ("records", "RECORDS", TEAL), ("backup", "BACKUP TO WINDOWS", GREY), ("copy", "COPY LOG", GREY)]:
            b = tk.Button(right, text=label, bg=color, fg="white", activebackground=color, activeforeground="white",
                          relief="flat", bd=0, font=(FONT, 12, "bold"), pady=11, cursor="hand2",
                          command=(self.copy_log if key == "copy" else self.open_records if key == "records" else (lambda k=key: self.run(k))))
            b.pack(fill="x", pady=3)
            self.buttons[key] = b
        self.estop = tk.Button(right, text="E-STOP\nstop everything", bg=RED, fg="white", activebackground="#e53935",
                               activeforeground="white", relief="flat", bd=0, font=(FONT, 14, "bold"), pady=10,
                               cursor="hand2", highlightbackground="#f2c230", highlightthickness=3,
                               command=lambda: self.run("stop"))
        self.estop.pack(fill="x", side="bottom", pady=(10, 0))

        self.reset_tiles()
        self.set_state("IDLE")
        root.after(100, self.poll)
        root.after(500, self.tick)

    # ------------------------------------------------------------ operator setup
    def pick_ball(self, key):
        if self.busy or not self.catalog[key]["enabled"]:
            return
        self.sel_ball = key
        sizes = self.catalog[key]["sizes"]
        if self.sel_size not in sizes and sizes:
            self.sel_size = sizes[-1]
        self.refresh_selector()

    def pick_size(self, n):
        if self.busy:
            return
        self.sel_size = n
        self.refresh_selector()

    def refresh_selector(self):
        for key, b in self.ball_buttons.items():
            en = self.catalog[key]["enabled"]
            on = key == self.sel_ball
            b.configure(bg=BLUE if on else (GREY if en else "#222b35"), fg="white" if en else "#667584",
                        activebackground=BLUE if on else GREY, activeforeground="white", disabledforeground="#667584",
                        state="disabled" if (self.busy or not en) else "normal")
        for w in self.row2.winfo_children():
            w.destroy()
        self.size_buttons = {}
        tk.Label(self.row2, text="SIZE", bg=CARD, fg=MUTED, font=(FONT, 9, "bold"), width=6, anchor="w").pack(side="left")
        for n in self.catalog[self.sel_ball]["sizes"]:
            on = n == self.sel_size
            b = tk.Button(self.row2, text=f"SIZE {n}", bg=BLUE if on else GREY, fg="white", activebackground=BLUE if on else GREY,
                          activeforeground="white", relief="flat", bd=0, font=(FONT, 11, "bold"), padx=16, pady=6,
                          cursor="hand2", state="disabled" if self.busy else "normal", command=(lambda m=n: self.pick_size(m)))
            b.pack(side="left", padx=3)
            self.size_buttons[n] = b
        inst_ball, inst_size = self.installed
        same = (inst_ball, inst_size) == (self.sel_ball, self.sel_size)
        self.model_lbl.configure(text=f"machine is set up for: {self.catalog[inst_ball]['label']} size {inst_size}"
                                      + ("" if same else "   ->  press APPLY to change"), fg=TEXT if same else AMBER)
        self.apply_btn.configure(state="disabled" if (self.busy or same) else "normal")
        self.sub.configure(text=f"{self.catalog[inst_ball]['label'].lower()} seam gluing  |  size {inst_size}  |  simulation")

    def apply_selection(self):
        if self.busy or (self.sel_ball, self.sel_size) == self.installed:
            return
        self.run("select", self.sel_ball, str(self.sel_size))

    # ------------------------------------------------------------ records screen
    def open_records(self):
        if self.rec_win is not None and self.rec_win.winfo_exists():
            self.rec_win.lift()
            self.fill_records()
            return
        w = tk.Toplevel(self.root)
        self.rec_win = w
        w.title("FYP HMI - Run records")
        w.configure(bg=BG)
        w.geometry("1000x620")
        bar = tk.Frame(w, bg=CARD, height=48)
        bar.pack(fill="x")
        bar.pack_propagate(False)
        tk.Label(bar, text="RUN RECORDS", bg=CARD, fg=TEXT, font=(FONT, 15, "bold")).pack(side="left", padx=18)
        tk.Label(bar, text="every finished or stopped run is saved automatically", bg=CARD, fg=MUTED, font=(FONT, 10)).pack(side="left")
        cards = tk.Frame(w, bg=BG)
        cards.pack(fill="x", padx=14, pady=(12, 4))
        self.rec_cards = {}
        for i, (key, title) in enumerate([("today", "RUNS TODAY"), ("ok", "COMPLETED"), ("cycle", "AVG CYCLE TIME"), ("edges", "EDGES GLUED (ALL RUNS)")]):
            cards.columnconfigure(i, weight=1, uniform="r")
            f = tk.Frame(cards, bg=CARD, highlightbackground=EDGE, highlightthickness=1)
            f.grid(row=0, column=i, sticky="nsew", padx=4)
            tk.Label(f, text=title, bg=CARD, fg=MUTED, font=(FONT, 9, "bold")).pack(anchor="w", padx=12, pady=(8, 0))
            v = tk.Label(f, text="-", bg=CARD, fg=TEXT, font=(FONT, 20, "bold"))
            v.pack(anchor="w", padx=12, pady=(0, 10))
            self.rec_cards[key] = v
        st = ttk.Style(w)
        st.theme_use("clam")
        st.configure("Rec.Treeview", background="#0f151b", fieldbackground="#0f151b", foreground=TEXT, rowheight=30,
                     font=(FONT, 10), borderwidth=0)
        st.configure("Rec.Treeview.Heading", background=CARD, foreground=MUTED, font=(FONT, 9, "bold"), relief="flat")
        st.map("Rec.Treeview", background=[("selected", BLUE)])
        cols = [("time", "TIME", 150), ("ball", "BALL", 90), ("size", "SIZE", 55), ("run", "RUN", 60), ("result", "RESULT", 100),
                ("edges", "EDGES", 80), ("cycle", "CYCLE", 80), ("zlift", "Z LIFT", 70), ("glue", "GLUE", 80)]
        box = tk.Frame(w, bg=BG)
        box.pack(fill="both", expand=True, padx=14, pady=8)
        self.tree = ttk.Treeview(box, columns=[c[0] for c in cols], show="headings", style="Rec.Treeview")
        for key, title, width in cols:
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, anchor="w")
        sb = ttk.Scrollbar(box, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.tree.tag_configure("bad", foreground="#ff8a80")
        foot = tk.Frame(w, bg=BG)
        foot.pack(fill="x", padx=14, pady=(0, 12))
        for text, color, cmd in [("EXPORT TO WINDOWS (Excel)", BLUE, self.export_records), ("REFRESH", GREY, self.fill_records),
                                 ("CLOSE", GREY, w.destroy)]:
            tk.Button(foot, text=text, bg=color, fg="white", activebackground=color, activeforeground="white", relief="flat", bd=0,
                      font=(FONT, 11, "bold"), padx=16, pady=8, cursor="hand2", command=cmd).pack(side="left", padx=(0, 8))
        self.rec_msg = tk.Label(foot, text="", bg=BG, fg=MUTED, font=(FONT, 10))
        self.rec_msg.pack(side="left", padx=8)
        self.fill_records()

    def fill_records(self):
        rows = read_records()
        self.tree.delete(*self.tree.get_children())
        for r in rows[:200]:
            result = r.get("result", "")
            cycle = r.get("cycle_s") or ""
            self.tree.insert("", "end", tags=(() if result == "COMPLETED" else ("bad",)), values=(
                r.get("time", ""), (r.get("ball", "") or "").capitalize(), r.get("size", ""), r.get("run", ""), result,
                f"{r.get('edges_glued', '')}/{r.get('edges_total', '')}",
                (f"{int(float(cycle)) // 60}:{int(float(cycle)) % 60:02d}" if cycle else "-"),
                (f"{r['z_lift_pct']}%" if r.get("z_lift_pct") else "-"), r.get("glue_style", "") or "-"))
        today = time.strftime("%Y-%m-%d")
        done = [r for r in rows if r.get("result") == "COMPLETED"]
        cyc = [float(r["cycle_s"]) for r in done if r.get("cycle_s")]
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
        self.rec_msg.configure(text="" if rows else "No runs recorded yet. Finish a run and it appears here.")

    def export_records(self):
        if not os.path.exists(RECORDS):
            self.rec_msg.configure(text="Nothing to export yet.", fg=AMBER)
            return
        docs = windows_documents()
        if not docs:
            self.rec_msg.configure(text=f"Could not find Windows Documents. The file is: {RECORDS}", fg=AMBER)
            return
        dest = os.path.join(docs, "FYP_records_" + time.strftime("%Y%m%d_%H%M") + ".csv")
        shutil.copyfile(RECORDS, dest)
        self.rec_msg.configure(text="Saved to Documents\\" + os.path.basename(dest), fg=GREEN)

    # ------------------------------------------------------------ small helpers
    def say(self, text):
        self.log.insert("end", text)
        self.log.see("end")
        try:
            with open(LOGFILE, "a") as f:
                f.write(text)
        except OSError:
            pass

    def copy_log(self):
        txt = self.log.get("1.0", "end")
        self.root.clipboard_clear()
        self.root.clipboard_append(txt)
        self.root.update()
        self.say(f"\n[log copied; also saved in {LOGFILE}]\n")

    def set_state(self, name):
        self.chip.configure(text=name, bg=STATES.get(name, GREY))

    def tile(self, key, text, color=TEXT):
        self.tiles[key].configure(text=text, fg=color)

    def reset_tiles(self):
        self.tile("pass", "-")
        self.tile("edges", "0 / 90")
        self.tile("time", "00:00")
        self.tile("nozzle", "-")
        self.tile("valve", "-")
        self.tile("ztrack", "-")
        self.pct = 0.0
        self.draw_bar()

    def draw_bar(self):
        self.bar.delete("all")
        w = max(self.bar.winfo_width(), 10)
        self.bar.create_rectangle(0, 0, w * self.pct, 26, fill=TEAL, width=0)
        self.bar.create_text(w / 2, 13, text=f"{self.pct * 100:.0f} %", fill="white", font=(FONT, 10, "bold"))

    def set_busy(self, busy):
        self.busy = busy
        for k, b in self.buttons.items():
            if k in ("copy", "records"):
                continue
            b.configure(state="disabled" if busy else "normal")
        self.refresh_selector()

    # ------------------------------------------------------------ running demo.sh
    def run(self, key, *args):
        if key == "stop":
            self.stop_pressed = True
            if self.proc and self.proc.poll() is None:
                self.proc.terminate()
        elif self.busy:
            return
        if key in ("short", "full"):
            try:
                open(LOGFILE, "w").close()
            except OSError:
                pass
        self.say(f"\n--- {key} ---\n")
        if key in ("short", "full"):
            self.reset_tiles()
            self.passes_seen = 0
            self.edges_now = (0, 90)
            self.stop_pressed = False
            self.run_t0 = time.time()
            self.t0 = time.monotonic()
            self.set_state("WORKING")
        elif key in ("start", "select"):
            self.set_state("STARTING")
            if key == "select":
                self.reset_tiles()
                self.pending = (args[0], int(args[1]))
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
        low = s.lower()
        if s.startswith("SELECTED:"):
            self.say("   (model rebuilt, restarting the simulation...)\n")
        elif s.startswith("READY"):
            self.set_state("READY")
            self.set_busy(False)          # unlock the buttons as soon as the simulation says READY
        elif s.startswith("FAILED"):
            self.set_state("FAULT")
        elif s.startswith("Settings:"):
            self.set_state("RUNNING")
        elif "Tracing seams" in s:
            self.passes_seen += 1
            self.tile("pass", f"{self.passes_seen} of 2")
            self.set_state("RUNNING")
        elif s.startswith("RE-CLAMP"):
            self.set_state("RE-CLAMPING")
            self.tile("nozzle", "UP", AMBER)
            self.tile("valve", "CLOSED", MUTED)
        elif "nozzle down" in low:
            self.tile("nozzle", "DOWN", GREEN)
            self.tile("valve", "OPEN", GREEN)
        elif "nozzle up" in low:
            self.tile("nozzle", "UP", AMBER)
            self.tile("valve", "CLOSED", MUTED)
        elif s.startswith("Raise nozzle") or s.startswith("Return home") or s.startswith("Home"):
            self.tile("nozzle", "UP", AMBER)
            self.tile("valve", "CLOSED", MUTED)
        elif s.startswith("PROGRESS"):
            m = re.search(r"glued=(\d+)/(\d+)", s)
            if m:
                a, b = int(m.group(1)), max(1, int(m.group(2)))
                self.edges_now = (a, b)
                self.tile("edges", f"{a} / {b}")
                self.pct = a / b
                self.draw_bar()
        elif s.startswith("Z TRACKING"):
            m = re.search(r"\((\d+)%\)", s)
            if m:
                p = int(m.group(1))
                self.tile("ztrack", f"{p} %", GREEN if p >= 70 else RED)
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
                    key, code = val
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
                            if self.rec_win is not None and self.rec_win.winfo_exists():
                                self.fill_records()
                        elif self.rec_win is not None and self.rec_win.winfo_exists():
                            self.fill_records()
                        self.set_state("DONE" if code == 0 else "STOPPED")
                        self.t0 = None
                        self.tile("nozzle", "-")
                        self.tile("valve", "-")
                    elif key in ("start", "select"):
                        self.set_state("READY" if code == 0 else "FAULT")
                        if key == "select":      # always trust the files on disk, even if the restart failed
                            self.installed = current_model(self.catalog)
                            self.sel_ball, self.sel_size = self.installed
                            self.refresh_selector()
                    elif key == "stop":
                        self.set_state("STOPPED")
                    elif key in ("clear", "backup"):
                        self.set_state("READY" if code == 0 else "FAULT")
        except queue.Empty:
            pass
        self.root.after(100, self.poll)

    def tick(self):
        self.clock.configure(text=time.strftime("%H:%M  "))
        if self.t0 is not None:
            e = int(time.monotonic() - self.t0)
            self.tile("time", f"{e // 60:02d}:{e % 60:02d}")
        self.root.after(500, self.tick)


if __name__ == "__main__":
    root = tk.Tk()
    Hmi(root)
    root.mainloop()
