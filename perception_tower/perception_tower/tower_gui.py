"""Perception Tower GUI — tkinter control panel.

Provides buttons for CMD_INIT / CMD_SCAN, real-time status display,
and a scrollable log area.

Usage:
    python -m perception_tower.tower_gui
    # or
    tower_gui
"""

from __future__ import annotations

import queue
import tkinter as tk
from tkinter import scrolledtext, messagebox
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from perception_tower_interfaces.msg import TowerStatus
from perception_tower_interfaces.srv import TowerCommand

STATE_NAMES = {
    0: "IDLE",
    1: "INITING",
    2: "READY",
    3: "SCANNING",
    4: "PROCESSING",
    5: "ERROR",
}

STATE_COLORS = {
    0: "#888888",
    1: "#e6a817",
    2: "#2ecc71",
    3: "#3498db",
    4: "#9b59b6",
    5: "#e74c3c",
}


class TowerGUI:
    def __init__(self, node: Node):
        self._node = node
        self._pending: dict[int, object] = {}
        self._log_queue: queue.Queue = queue.Queue()

        # --- ROS setup ---
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            TowerStatus, "/perception_tower/status", self._on_status, status_qos
        )
        self._cmd_cli = node.create_client(TowerCommand, "/perception_tower/command")

        # --- tkinter ---
        self._root = tk.Tk()
        self._root.title("Perception Tower Control")
        self._root.geometry("620x520")
        self._root.minsize(500, 400)
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()
        self._ros_tick()
        self._drain_log_queue()

    # ── UI construction ──────────────────────────────────────────────

    def _build_ui(self):
        # ── Status bar ──
        status_frame = tk.LabelFrame(self._root, text="Status", padx=8, pady=6)
        status_frame.pack(fill=tk.X, padx=8, pady=(8, 4))

        self._state_label = tk.Label(
            status_frame, text="IDLE", font=("Helvetica", 18, "bold"),
            fg=STATE_COLORS[0], anchor=tk.W,
        )
        self._state_label.pack(side=tk.LEFT)

        self._progress_label = tk.Label(
            status_frame, text="0%", font=("Helvetica", 14), anchor=tk.E
        )
        self._progress_label.pack(side=tk.RIGHT)

        self._msg_label = tk.Label(
            status_frame, text="initialized", font=("Helvetica", 11),
            fg="#555", anchor=tk.W, wraplength=400,
        )
        self._msg_label.pack(side=tk.LEFT, padx=(12, 0), fill=tk.X, expand=True)

        # ── Progress bar ──
        bar_frame = tk.Frame(self._root)
        bar_frame.pack(fill=tk.X, padx=8, pady=(0, 4))
        self._canvas = tk.Canvas(bar_frame, height=18, bg="#e0e0e0", highlightthickness=0)
        self._canvas.pack(fill=tk.X)
        self._bar_rect = None

        # ── Buttons ──
        btn_frame = tk.Frame(self._root)
        btn_frame.pack(fill=tk.X, padx=8, pady=4)

        self._btn_init = tk.Button(
            btn_frame, text="CMD_INIT (Reset & Home)", width=26, height=2,
            bg="#e6a817", activebackground="#cc9610",
            font=("Helvetica", 11, "bold"),
            command=lambda: self._send_cmd(TowerCommand.Request.CMD_INIT),
        )
        self._btn_init.pack(side=tk.LEFT, padx=(0, 8))

        self._btn_scan = tk.Button(
            btn_frame, text="CMD_SCAN (Scan)", width=26, height=2,
            bg="#3498db", activebackground="#2980b9", fg="white",
            font=("Helvetica", 11, "bold"),
            command=lambda: self._send_cmd(TowerCommand.Request.CMD_SCAN),
        )
        self._btn_scan.pack(side=tk.LEFT)

        # ── Log area ──
        log_frame = tk.LabelFrame(self._root, text="Log", padx=4, pady=4)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=(4, 8))

        self._log_text = scrolledtext.ScrolledText(
            log_frame, height=14, font=("Courier", 10),
            state=tk.DISABLED, wrap=tk.WORD, bg="#1e1e1e", fg="#d4d4d4",
            insertbackground="white",
        )
        self._log_text.pack(fill=tk.BOTH, expand=True)

        # tag for timestamp coloring
        self._log_text.tag_configure("ts", foreground="#6a9955")
        self._log_text.tag_configure("info", foreground="#d4d4d4")
        self._log_text.tag_configure("warn", foreground="#e6a817")
        self._log_text.tag_configure("err", foreground="#e74c3c")

    # ── ROS callbacks ────────────────────────────────────────────────

    def _on_status(self, msg: TowerStatus):
        state_name = STATE_NAMES.get(msg.state, f"UNKNOWN({msg.state})")
        color = STATE_COLORS.get(msg.state, "#888888")

        self._log_queue.put(("info", f"status: {state_name} {msg.progress_pct}% — {msg.message}"))

        self._state_label.config(text=state_name, fg=color)
        self._progress_label.config(text=f"{msg.progress_pct}%")
        self._msg_label.config(text=msg.message)
        self._update_bar(msg.progress_pct)

        busy = msg.state in (1, 3, 4)  # INITING, SCANNING, PROCESSING
        self._btn_init.config(state=tk.DISABLED if busy else tk.NORMAL)
        self._btn_scan.config(state=tk.DISABLED if busy else tk.NORMAL)

    # ── Command sending (non-blocking) ──────────────────────────────

    def _send_cmd(self, command: int):
        name = "INIT" if command == TowerCommand.Request.CMD_INIT else "SCAN"
        if not self._cmd_cli.wait_for_service(timeout_sec=1.0):
            self._log_queue.put(("err", f"service /perception_tower/command not available"))
            messagebox.showerror("Error", "Command service not available.\nIs tower_node running?")
            return

        req = TowerCommand.Request()
        req.command = command
        self._log_queue.put(("info", f">>> sending CMD_{name}"))
        fut = self._cmd_cli.call_async(req)
        self._pending[command] = fut

    def _check_pending(self):
        done = [k for k, f in self._pending.items() if f.done()]
        for cmd in done:
            fut = self._pending.pop(cmd)
            name = "INIT" if cmd == TowerCommand.Request.CMD_INIT else "SCAN"
            try:
                result = fut.result()
                if result.accepted:
                    self._log_queue.put(("info", f"CMD_{name} accepted: {result.message}"))
                else:
                    self._log_queue.put(("warn", f"CMD_{name} rejected: {result.message}"))
            except Exception as exc:
                self._log_queue.put(("err", f"CMD_{name} failed: {exc}"))

    # ── Progress bar ─────────────────────────────────────────────────

    def _update_bar(self, pct: int):
        self._canvas.delete("all")
        w = self._canvas.winfo_width()
        h = self._canvas.winfo_height()
        if w <= 1:
            return
        fill_w = max(1, int(w * pct / 100))
        self._canvas.create_rectangle(0, 0, fill_w, h, fill="#3498db", outline="")
        self._canvas.create_text(
            w // 2, h // 2, text=f"{pct}%", fill="white",
            font=("Helvetica", 9, "bold"),
        )

    # ── Log drain (thread-safe) ─────────────────────────────────────

    def _drain_log_queue(self):
        while not self._log_queue.empty():
            level, text = self._log_queue.get_nowait()
            ts = datetime.now().strftime("%H:%M:%S")
            self._log_text.config(state=tk.NORMAL)
            self._log_text.insert(tk.END, f"[{ts}] ", "ts")
            self._log_text.insert(tk.END, f"{text}\n", level)
            self._log_text.see(tk.END)
            self._log_text.config(state=tk.DISABLED)
        self._root.after(100, self._drain_log_queue)

    # ── ROS spin tick ───────────────────────────────────────────────

    def _ros_tick(self):
        rclpy.spin_once(self._node, timeout_sec=0)
        self._check_pending()
        self._root.after(20, self._ros_tick)

    # ── Lifecycle ────────────────────────────────────────────────────

    def _on_close(self):
        self._root.destroy()

    def run(self):
        self._root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("tower_gui")
    try:
        gui = TowerGUI(node)
        gui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
