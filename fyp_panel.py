#!/usr/bin/env python3
# FYP touchscreen-style HMI (runs in Ubuntu, shows through WSLg next to Gazebo).
# It only starts demo.sh and reads what the run prints, so it needs nothing but python3-tk.
import os
import queue
import re
import subprocess
import threading
import time
import tkinter as tk
from tkinter import scrolledtext

DEMO = os.environ.get("FYP_DEMO", os.path.join(os.path.expanduser("~"), "FYP_ball_gum", "demo.sh"))

LOGFILE = os.path.join(os.path.dirname(DEMO), "panel_last.log")

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
        root.title("FYP HMI - Vision-Guided Gum Application (simulation)")
        root.configure(bg=BG)
        root.geometry("860x560")
        root.minsize(760, 520)

        # ---- header bar
        bar = tk.Frame(root, bg=CARD, height=54)
        bar.pack(fill="x")
        bar.pack_propagate(False)
        tk.Label(bar, text="GUM APPLICATOR", bg=CARD, fg=TEXT, font=(FONT, 16, "bold")).pack(side="left", padx=(18, 6))
        tk.Label(bar, text="football seam gluing  |  simulation", bg=CARD, fg=MUTED, font=(FONT, 11)).pack(side="left")
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
                                  ("backup", "BACKUP TO WINDOWS", GREY), ("copy", "COPY LOG", GREY)]:
            b = tk.Button(right, text=label, bg=color, fg="white", activebackground=color, activeforeground="white",
                          relief="flat", bd=0, font=(FONT, 12, "bold"), pady=11, cursor="hand2",
                          command=(self.copy_log if key == "copy" else (lambda k=key: self.run(k))))
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
            if k == "copy":
                continue
            b.configure(state="disabled" if busy else "normal")

    # ------------------------------------------------------------ running demo.sh
    def run(self, key):
        if key == "stop":
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
            self.t0 = time.monotonic()
            self.set_state("WORKING")
        elif key == "start":
            self.set_state("STARTING")
        elif key == "stop":
            self.set_state("STOPPED")
            self.t0 = None
        self.set_busy(key != "stop")
        threading.Thread(target=self.worker, args=(key,), daemon=True).start()

    def worker(self, key):
        try:
            self.proc = subprocess.Popen(["bash", DEMO, key], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
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
        if s.startswith("READY"):
            self.set_state("READY")
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
                        self.set_state("DONE" if code == 0 else "STOPPED")
                        self.t0 = None
                        self.tile("nozzle", "-")
                        self.tile("valve", "-")
                    elif key == "start":
                        self.set_state("READY" if code == 0 else "FAULT")
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
