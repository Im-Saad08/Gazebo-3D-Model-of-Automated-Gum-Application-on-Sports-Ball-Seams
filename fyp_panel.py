#!/usr/bin/env python3
# FYP control panel: big buttons for the supervisor demo (runs in Ubuntu, shows through WSLg).
import os
import queue
import subprocess
import threading
import tkinter as tk
from tkinter import scrolledtext

DEMO = os.environ.get("FYP_DEMO", os.path.join(os.path.expanduser("~"), "FYP_ball_gum", "demo.sh"))

BG = "#14181d"
PANEL = "#1d232b"
TEXT = "#e8edf2"
MUTED = "#8b97a5"
GREEN = "#1f9d55"
BLUE = "#2b6cb0"
AMBER = "#c98a0b"
RED = "#b83232"
GREY = "#3a4450"


class Panel:
    def __init__(self, root):
        self.root = root
        self.proc = None
        self.q = queue.Queue()
        self.busy = False
        root.title("FYP - Vision-Guided Automated Gum Application (Simulation)")
        root.configure(bg=BG)
        root.geometry("760x640")
        root.minsize(640, 560)

        tk.Label(root, text="Automated Gum Application on Sport's Ball Seams", bg=BG, fg=TEXT,
                 font=("DejaVu Sans", 17, "bold")).pack(pady=(16, 0))
        tk.Label(root, text="ROS 2 + Gazebo simulation", bg=BG, fg=MUTED,
                 font=("DejaVu Sans", 11)).pack(pady=(0, 10))

        self.status = tk.Label(root, text="Status: not started", bg=PANEL, fg=TEXT, anchor="w",
                               font=("DejaVu Sans", 12, "bold"), padx=14, pady=8)
        self.status.pack(fill="x", padx=18)

        grid = tk.Frame(root, bg=BG)
        grid.pack(fill="x", padx=18, pady=12)
        for c in range(2):
            grid.columnconfigure(c, weight=1, uniform="b")

        self.buttons = {}
        spec = [
            ("start", "1   Start Simulation", GREEN, 0, 0, 2),
            ("short", "2   Run Gum Demo  (short)", BLUE, 1, 0, 1),
            ("full", "3   Run Full Seam Loop", BLUE, 1, 1, 1),
            ("clear", "Clear Gum", AMBER, 2, 0, 1),
            ("backup", "Backup to Windows", GREY, 2, 1, 1),
            ("stop", "Stop Everything", RED, 3, 0, 2),
        ]
        for key, label, color, r, c, span in spec:
            b = tk.Button(grid, text=label, bg=color, fg="white", activebackground=color,
                          activeforeground="white", relief="flat", bd=0,
                          font=("DejaVu Sans", 13, "bold"), pady=14, cursor="hand2",
                          command=lambda k=key: self.run(k))
            b.grid(row=r, column=c, columnspan=span, sticky="ew", padx=5, pady=5)
            self.buttons[key] = b

        self.log = scrolledtext.ScrolledText(root, bg="#0d1014", fg="#c9d4df", insertbackground="white",
                                             font=("DejaVu Sans Mono", 10), relief="flat", height=12)
        self.log.pack(fill="both", expand=True, padx=18, pady=(0, 16))
        self.say("Press 1 to start. Gazebo opens in its own window; wait for READY, then press 2 or 3.\n")
        self.root.after(100, self.poll)

    def say(self, text):
        self.log.insert("end", text)
        self.log.see("end")

    def set_busy(self, busy, status=None):
        self.busy = busy
        for k, b in self.buttons.items():
            if k != "stop":
                b.configure(state="disabled" if busy else "normal")
        if status:
            self.status.configure(text="Status: " + status)

    def run(self, key):
        if key == "stop":
            if self.proc and self.proc.poll() is None:
                self.proc.terminate()
        elif self.busy:
            return
        labels = {"start": "starting simulation...", "short": "running short gum demo...",
                  "full": "running full seam loop...", "clear": "clearing gum...",
                  "backup": "backing up...", "stop": "stopping..."}
        self.say(f"\n--- {key} ---\n")
        self.set_busy(key != "stop", labels[key])
        threading.Thread(target=self.worker, args=(key,), daemon=True).start()

    def worker(self, key):
        try:
            self.proc = subprocess.Popen(["bash", DEMO, key], stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True, bufsize=1)
            for line in self.proc.stdout:
                self.q.put(("line", line))
            code = self.proc.wait()
        except Exception as exc:
            self.q.put(("line", f"Could not run {DEMO}: {exc}\n"))
            code = 1
        self.q.put(("done", (key, code)))

    def poll(self):
        try:
            while True:
                kind, val = self.q.get_nowait()
                if kind == "line":
                    self.say(val)
                else:
                    key, code = val
                    ok = code == 0
                    msg = {"start": "SIMULATION RUNNING - ready" if ok else "start failed (see log)",
                           "short": "demo finished" if ok else "demo stopped",
                           "full": "full loop finished" if ok else "run stopped",
                           "clear": "gum cleared" if ok else "could not clear gum",
                           "backup": "backup saved" if ok else "backup failed",
                           "stop": "stopped - press 1 to start again"}[key]
                    self.set_busy(False, msg)
        except queue.Empty:
            pass
        self.root.after(100, self.poll)


if __name__ == "__main__":
    root = tk.Tk()
    Panel(root)
    root.mainloop()
