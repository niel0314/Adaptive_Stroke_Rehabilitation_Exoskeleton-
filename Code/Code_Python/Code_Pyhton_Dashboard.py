"""
============================================================================
 HapticElbow  ·  Rehab Control Panel  ·  Dashboard
 Niel Boon & Senne Peeters  ·  KU Leuven  ·  2026
============================================================================

 Single-file dashboard for the adaptive elbow exoskeleton (ODrive S1 +
 2x MPU-6050 + DRV2605L LRA). Talks to the Arduino firmware over a simple
 line-based serial protocol (see Code_Arduino.ino).

 ─── ARCHITECTURE ─────────────────────────────────────────────────────────
   SerialClient          - non-blocking link to the Arduino firmware
   TelemetryBuffer       - sliding windows of each sensor channel
   CompensationMonitor   - stable shoulder-compensation detector
   GameState / ROMState  - state for the gamified exercise modes
   RehabApp(ctk.CTk)     - main GUI; one _build_*-method per tab
 ──────────────────────────────────────────────────────────────────────────

 The shoulder-compensation detector is a reach-based, ratio-modulated,
 zone-aware monitor — see the CompensationMonitor docstring. Grounded in
 Schwarz 2020, Levin's RPSS and Cirstea & Levin 2000 (see README §3.6.1).
"""

from __future__ import annotations

import csv
import math
import random
import socket
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import customtkinter as ctk
import tkinter as tk
from tkinter import filedialog, messagebox

import serial
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)


# ===========================================================================
#  1.  CONFIGURATION  (ports, theme, fonts)
# ===========================================================================

COM_PORT  = "COM6"             # CHANGE THIS to your serial port
BAUDRATE  = 115200

UDP_IP    = "127.0.0.1"        # destination for 3D visualisation (Unity)
UDP_PORT  = 5005

# Design tokens — centrally managed so every component looks consistent.
THEME = {
    "bg_app":          "#0e1117",
    "bg_sidebar":      "#161b22",
    "bg_card":         "#1c2128",
    "bg_card_alt":     "#22272e",
    "bg_elevated":     "#262d36",
    "bg_canvas":       "#13171d",
    "border":          "#30363d",
    "border_subtle":   "#21262d",
    "accent":          "#2f9bff",
    "accent_hover":    "#1f7fd6",
    "accent_dim":      "#1a4f80",
    "success":         "#3fb950",
    "success_hover":   "#2ea043",
    "warning":         "#e3b341",
    "warning_hover":   "#bb8409",
    "danger":          "#f85149",
    "danger_hover":    "#da3633",
    "text_primary":    "#e6edf3",
    "text_secondary":  "#8b949e",
    "text_muted":      "#484f58",
    "chart_pos":       "#58a6ff",
    "chart_vel":       "#3fb950",
    "chart_acc":       "#e3b341",
    "chart_trq":       "#f85149",
    "chart_target":    "#8b949e",
    "chart_assist":    "#bc8cff",
}

FONT_FAMILY = "Segoe UI"
FONTS = {
    "display":      (FONT_FAMILY, 52, "bold"),
    "h1":           (FONT_FAMILY, 26, "bold"),
    "h2":           (FONT_FAMILY, 18, "bold"),
    "h3":           (FONT_FAMILY, 15, "bold"),
    "body":         (FONT_FAMILY, 13),
    "body_bold":    (FONT_FAMILY, 13, "bold"),
    "caption":      (FONT_FAMILY, 11),
    "caption_bold": (FONT_FAMILY, 11, "bold"),
    "small":        (FONT_FAMILY, 10),
}

# ===========================================================================
#  2.  HELPERS  (small, reusable, stateless)
# ===========================================================================

def style_axes(ax):
    """Consistent styling for every matplotlib axes."""
    ax.set_facecolor(THEME["bg_card"])
    ax.tick_params(colors=THEME["text_secondary"], labelsize=9)
    ax.grid(True, color=THEME["border"], linewidth=0.5, alpha=0.5)
    for spine in ax.spines.values():
        spine.set_color(THEME["border"])
        spine.set_linewidth(0.8)


def blend_color(bg_hex: str, fg_hex: str, alpha: float) -> str:
    """Linear interpolation between two #RRGGBB colours.
    alpha=0 → bg, alpha=1 → fg. Used for soft fade effects."""
    alpha = max(0.0, min(1.0, alpha))
    bg = tuple(int(bg_hex[i:i + 2], 16) for i in (1, 3, 5))
    fg = tuple(int(fg_hex[i:i + 2], 16) for i in (1, 3, 5))
    out = tuple(int(b + (f - b) * alpha) for b, f in zip(bg, fg))
    return f"#{out[0]:02x}{out[1]:02x}{out[2]:02x}"


def make_card(parent, **kwargs) -> ctk.CTkFrame:
    """Standard card frame with consistent styling."""
    return ctk.CTkFrame(parent,
                        fg_color=THEME["bg_card"],
                        corner_radius=12,
                        border_width=1,
                        border_color=THEME["border_subtle"],
                        **kwargs)


def make_section_title(parent, text: str, sub: Optional[str] = None):
    """Section heading with optional subtitle."""
    wrap = ctk.CTkFrame(parent, fg_color="transparent")
    wrap.pack(fill="x", padx=4, pady=(6, 12))
    ctk.CTkLabel(wrap, text=text, font=FONTS["h2"],
                 text_color=THEME["text_primary"]).pack(anchor="w")
    if sub:
        ctk.CTkLabel(wrap, text=sub, font=FONTS["caption"],
                     text_color=THEME["text_secondary"]).pack(anchor="w", pady=(2, 0))
    return wrap


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


# ---------------------------------------------------------------------------
#  Tooltip — small hover popup, works on both tk and ctk widgets.
# ---------------------------------------------------------------------------

class Tooltip:
    """Show a short explanation when the mouse hovers over a widget.

    Usage:
        Tooltip(my_label, "This number tells you ...")

    Works on CTk and tk widgets by simply binding <Enter>/<Leave>/<Motion>
    events. No extra dependencies."""

    _OPEN: Optional["Tooltip"] = None   # only one tooltip open at a time

    def __init__(self, widget, text: str, wraplength: int = 300):
        self.widget = widget
        self.text   = text
        self.wraplength = wraplength
        self._tip: Optional[tk.Toplevel] = None
        widget.bind("<Enter>",  self._show, add="+")
        widget.bind("<Leave>",  self._hide, add="+")
        widget.bind("<Button>", self._hide, add="+")

    def _show(self, _event=None):
        if self._tip is not None:
            return
        # Close any other open tooltip first
        if Tooltip._OPEN is not None and Tooltip._OPEN is not self:
            Tooltip._OPEN._hide()
        Tooltip._OPEN = self

        x = self.widget.winfo_rootx() + 18
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 6

        tip = tk.Toplevel(self.widget)
        tip.wm_overrideredirect(True)
        tip.wm_geometry(f"+{x}+{y}")
        tip.configure(bg=THEME["border"])
        # One border frame for the drawn edge
        inner = tk.Frame(tip, bg=THEME["bg_elevated"], padx=10, pady=8)
        inner.pack(padx=1, pady=1)
        tk.Label(inner, text=self.text,
                 bg=THEME["bg_elevated"],
                 fg=THEME["text_primary"],
                 font=(FONT_FAMILY, 10),
                 wraplength=self.wraplength,
                 justify="left").pack()
        self._tip = tip

    def _hide(self, _event=None):
        if self._tip is not None:
            try:
                self._tip.destroy()
            except Exception:
                pass
            self._tip = None
        if Tooltip._OPEN is self:
            Tooltip._OPEN = None


# ===========================================================================
#  3.  SERIAL CLIENT  (non-blocking link to Arduino)
# ===========================================================================

class SerialClient:
    """Wrapper around `pyserial` with:
      - Reconnect-resilient open
      - Fast outbound `send()`
      - Line-based inbound parse via callback (`on_line`)
      - 'is_alive' flag via watchdog timestamp

    The callback receives every complete (stripped) line and is allowed to
    raise — exceptions are swallowed here so the UI does not crash.
    """

    def __init__(self, port: str, baud: int):
        self.port_name = port
        self.baud = baud
        self.serial: Optional[serial.Serial] = None
        self.last_rx_time: float = 0.0
        self._buf = bytearray()
        self._open()

    def _open(self):
        try:
            self.serial = serial.Serial(self.port_name, self.baud, timeout=0.1)
            time.sleep(2.0)  # give the Arduino time to boot
            print(f"[SerialClient] connected on {self.port_name}")
        except Exception as e:
            print(f"[SerialClient] could not open {self.port_name}: {e}")
            self.serial = None

    @property
    def is_open(self) -> bool:
        return self.serial is not None and self.serial.is_open

    @property
    def alive(self) -> bool:
        """True if a message was received within 1.5 s."""
        return self.is_open and (time.time() - self.last_rx_time < 1.5)

    def send(self, cmd: str):
        if not self.is_open:
            return
        try:
            self.serial.write((cmd + "\n").encode())
        except Exception as e:
            print(f"[SerialClient] send error: {e}")

    def pump(self, on_line: Callable[[str], None]):
        """Read every available byte and call `on_line` for each complete
        line. Safe to call from a tk.after() loop."""
        if not self.is_open:
            return
        try:
            n = self.serial.in_waiting
            if n:
                self._buf.extend(self.serial.read(n))
                while b"\n" in self._buf:
                    line, self._buf = self._buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="ignore").strip()
                    if text:
                        self.last_rx_time = time.time()
                        try:
                            on_line(text)
                        except Exception as e:
                            print(f"[SerialClient] callback error: {e}")
        except Exception:
            pass


# ===========================================================================
#  4.  TELEMETRY BUFFER  (sliding windows per channel)
# ===========================================================================

class TelemetryBuffer:
    """Keeps the last N samples per channel in deques.
    Shared by the graphs and the analyses."""

    def __init__(self, size_chart: int = 200, size_compensation: int = 200):
        self.t        = deque(maxlen=size_chart)
        self.pos      = deque(maxlen=size_chart)
        self.vel      = deque(maxlen=size_chart)
        self.acc      = deque(maxlen=size_chart)
        self.trq      = deque(maxlen=size_chart)
        self.target   = deque(maxlen=size_chart)
        self.imu1     = deque(maxlen=size_compensation)
        self.imu2     = deque(maxlen=size_compensation)
        self._prev_vel = 0.0

    def push_pos(self, p: float, target: float, t: float):
        self.t.append(t)
        self.pos.append(p)
        self.target.append(target)

    def push_vel(self, v: float):
        self.vel.append(v)
        acc = (v - self._prev_vel) / 0.1     # dt=100ms (Arduino telemetry)
        self.acc.append(acc)
        self._prev_vel = v

    def push_trq(self, t: float):
        self.trq.append(t)

    def push_imu1(self, v: float):
        self.imu1.append(v)

    def push_imu2(self, v: float):
        self.imu2.append(v)


# ===========================================================================
#  5.  COMPENSATION MONITOR  (continuous sliding-window detector)
# ===========================================================================

class CompensationMonitor:
    """Shoulder-compensation detector - continuous, window-based.

    ── Why the design changed ──────────────────────────────────────────
    The previous version split the motion into discrete reaches and
    only finalised a score at the end of each reach. Two problems
    came out of that in practice:
      1. Inside one long continuous motion, an early stretch of good
         elbow use would give "credit" that hid a later shoulder
         compensation within the same reach.
      2. A quick bad motion followed immediately by a return to rest
         could end with no visible alarm, because the per-reach
         finalisation zeroed it via the direction check.

    ── How the new design works ────────────────────────────────────────
      • A sliding window of WINDOW_S seconds (default 1.5 s) holds the
        most recent samples of the upper-arm IMU and the motor angle.
      • Shoulder rise is the MAX of two things:
          - the absolute elevation above the session baseline (catches
            a sustained raised position even after the window slides
            past the upward motion),
          - the rise from the window-minimum to now (catches a recent
            upward motion).
      • Elbow excursion is the range of motor angle in the window
        ONLY. Old elbow motion from earlier in the session does not
        give a free pass anymore.
      • The same zone-aware ratio + 3° wobble subtraction is applied.
      • A direction check kills the live score whenever the upper arm
        is net descending across the window (returning to rest is not
        a compensation).
      • The displayed score has a peak-hold (PEAK_HOLD_S, default
        1.5 s) followed by a linear decay (DECAY_S, default 1.0 s).
        So a brief compensation followed by an immediate return still
        stays visible long enough for the therapist to see it.

    ── The chart ───────────────────────────────────────────────────────
    The chart at the bottom of the tab shows one bar per burst of
    activity. A burst is a stretch of continuous motion (combined
    velocity above BURST_START_OMEGA_DEG). When a burst ends, the
    max displayed score during that burst becomes the bar height.

    The COMPENSATIONS counter increments on every rising edge into
    the COMPENSATION level, so multiple events within a single burst
    are counted separately.
    """

    # ── Sliding window for the live score ────────────────────────────
    WINDOW_S = 1.5

    # ── Score peak hold + decay ──────────────────────────────────────
    PEAK_HOLD_S = 1.5     # how long the peak stays at full value
    DECAY_S     = 1.0     # then linear decay over this many seconds

    # ── IMU filtering before window stats ─────────────────────────────
    IMU_LPF_ALPHA = 0.25

    # ── Dynamic-motion gating of the IMU1 filter ─────────────────────
    # When the motor is rotating fast, the forearm exerts a tangential
    # reaction force on the upper arm. The MPU-6050 accelerometer
    # cannot tell that force apart from gravity, so the IMU1 reading
    # briefly swings even though the upper arm did not actually rotate.
    # That used to cause a false compensation alarm whenever the patient
    # made a quick forearm extension.
    #
    # The fix: when the motor is moving fast, we "freeze" the IMU1
    # low-pass filter so it holds the last stable value. Once the
    # motion stops, the filter catches up to the true reading.
    # A real shoulder lift persists after the motion stops, so it is
    # still detected (with a small delay). A motion artifact disappears
    # when the motion stops, so no false alarm is raised.
    DYNAMIC_GATE_LOW_DEG_S   = 10.0    # below this: full responsiveness
    DYNAMIC_GATE_HIGH_DEG_S  = 40.0    # above this: filter mostly frozen
    IMU_LPF_ALPHA_FROZEN     = 0.01    # alpha while frozen

    # ── Direction-aware threshold (degrees) ──────────────────────────
    DESCENDING_THRESHOLD = -5.0

    # ── Scoring constants ────────────────────────────────────────────
    ELBOW_WOBBLE             = 3.0
    ELBOW_MIN_FLOOR          = 5.0
    SHOULDER_RISE_FREE       = 5.0
    SHOULDER_RISE_TO_MAX     = 40.0

    # ── Elbow position credit ────────────────────────────────────────
    # An elbow that is currently bent represents active elbow use, even
    # if it is not moving right now. A patient who bends the elbow
    # first and then lifts the shoulder (e.g. hand-to-head motion) is
    # NOT compensating: they are using the elbow to position the
    # forearm. The window-based motion credit alone would miss this,
    # so we also give credit for the current elbow flexion above this
    # threshold. The threshold ignores small natural flexion that is
    # not really "using" the elbow.
    ELBOW_POSITION_THRESHOLD = 20.0   # deg of flexion before credit starts

    # ── Workspace zones (Levin RPSS) ──────────────────────────────────
    ZONE_LOW_CUTOFF  = 30.0
    ZONE_HIGH_CUTOFF = 90.0
    ZONE_RATIO_LOW   = 0.40
    ZONE_RATIO_MID   = 0.30
    ZONE_RATIO_HIGH  = 0.50

    # ── Level thresholds (in %) ──────────────────────────────────────
    LEVEL_WARN_DEFAULT  = 25.0
    LEVEL_ALARM_DEFAULT = 40.0

    # ── Motion burst detector (chart bars only) ───────────────────────
    BURST_START_OMEGA_DEG = 6.0
    BURST_END_OMEGA_DEG   = 3.0
    BURST_END_DEBOUNCE_S  = 0.40
    BURST_MIN_DUR_S       = 0.40

    LEVELS = ("INACTIVE", "REST", "GOOD", "WARNING", "COMPENSATION")

    def __init__(self, level_warn: float = LEVEL_WARN_DEFAULT,
                       level_alarm: float = LEVEL_ALARM_DEFAULT):
        self.level_warn  = level_warn
        self.level_alarm = level_alarm

        # Filter / time state
        self._imu1_filtered: Optional[float] = None
        self._prev_t: Optional[float] = None

        # Sliding window: each entry is (t, imu1_filtered, motor_deg)
        self._window: deque = deque()

        # Session baseline (set on start, refreshed on tare)
        self._session_baseline_imu1 = 0.0

        # Displayed score (with peak-hold + decay)
        self.score          = 0.0
        self._live_score    = 0.0     # raw window-based score
        self._score_peak_t  = 0.0

        # Mini-metrics for the UI
        self.shoulder_delta  = 0.0    # effective rise (max of abs + window)
        self.elbow_delta_max = 0.0    # window elbow range
        self.motor_speed_now = 0.0    # current motor speed [deg/s]
        self._current_motor_deg = 0.0 # current elbow angle (for position credit)

        # Level + event counter (rising-edge into COMPENSATION)
        self.level             = "INACTIVE"
        self.event_count       = 0
        self._was_in_alarm     = False

        # Burst tracking (for chart only)
        self._burst_active        = False
        self._burst_start_t       = 0.0
        self._burst_end_pending_t = 0.0
        self._burst_max_score     = 0.0

        # History (one bar per completed burst)
        self.last_reach_score = 0.0
        self.history: deque = deque(maxlen=30)
        self.history_levels: deque = deque(maxlen=30)
        self.reach_count = 0
        self.reach_active = False
        self.reach_state = "IDLE"

        # Session lifecycle
        self.session_t0 = 0.0
        self._active    = False
        self._log: list = []

    # ── Lifecycle ─────────────────────────────────────────────────────

    def start(self, imu_upper: float, imu_lower: float, motor_deg: float):
        """Begin a session: store the current pose as the baseline and clear all state."""
        self._imu1_filtered = imu_upper
        self._prev_t = time.time()
        self._session_baseline_imu1 = imu_upper
        self._window.clear()

        self.score = 0.0
        self._live_score = 0.0
        self._score_peak_t = self._prev_t

        self.level = "GOOD"
        self.last_reach_score = 0.0
        self.shoulder_delta = 0.0
        self.elbow_delta_max = 0.0
        self.motor_speed_now = 0.0

        self.history.clear()
        self.history_levels.clear()
        self._log.clear()

        self.event_count = 0
        self._was_in_alarm = False
        self.reach_count = 0
        self.reach_active = False
        self.reach_state = "IDLE"

        self._burst_active = False
        self._burst_end_pending_t = 0.0
        self._burst_max_score = 0.0

        self.session_t0 = time.time()
        self._active = True

    def stop(self):
        """End the session; the detector stops scoring until started again."""
        self._active = False
        self.level = "INACTIVE"

    def reset_baseline(self, imu_upper: float, imu_lower: float, motor_deg: float):
        """After a tare: refresh filter, baseline and clear the window."""
        self._imu1_filtered = imu_upper
        self._session_baseline_imu1 = imu_upper
        self._window.clear()
        self.score = 0.0
        self._live_score = 0.0
        self._burst_active = False
        self._burst_end_pending_t = 0.0
        self.reach_active = False
        self.reach_state = "IDLE"

    # ── Properties (UI compat) ────────────────────────────────────────

    @property
    def active(self) -> bool:
        return self._active

    @property
    def baseline(self) -> float:
        return 0.0

    @property
    def session_seconds(self) -> float:
        return time.time() - self.session_t0 if self._active else 0.0

    @property
    def has_log(self) -> bool:
        return len(self._log) > 0

    @property
    def avg_recent_score(self) -> float:
        if not self.history:
            return 0.0
        return sum(self.history) / len(self.history)

    @property
    def time_distribution(self):
        # No longer used; kept for compatibility.
        return (0.0, 0.0, 0.0)

    # ── Tick (called from the dashboard loop) ─────────────────────────

    def tick(self, imu_upper: float, imu_lower: float,
                   motor_deg: float, motor_vel: float):
        if not self._active:
            return

        now = time.time()
        if self._prev_t is None:
            self._prev_t = now
            self._imu1_filtered = imu_upper
            return

        dt = now - self._prev_t
        self._prev_t = now
        if dt < 0.001 or dt > 1.0:
            return

        # 1. Smooth IMU1, with the filter gated by motor activity.
        # When the motor is rotating fast, tangential acceleration
        # corrupts the upper-arm accelerometer. We fade alpha down so
        # the filter holds its last stable value during that time.
        motor_omega = abs(motor_vel) * 360.0   # rev/s -> deg/s
        if motor_omega <= self.DYNAMIC_GATE_LOW_DEG_S:
            alpha = self.IMU_LPF_ALPHA
        elif motor_omega >= self.DYNAMIC_GATE_HIGH_DEG_S:
            alpha = self.IMU_LPF_ALPHA_FROZEN
        else:
            frac = ((motor_omega - self.DYNAMIC_GATE_LOW_DEG_S) /
                    (self.DYNAMIC_GATE_HIGH_DEG_S - self.DYNAMIC_GATE_LOW_DEG_S))
            alpha = (self.IMU_LPF_ALPHA * (1.0 - frac)
                     + self.IMU_LPF_ALPHA_FROZEN * frac)

        if self._imu1_filtered is None:
            self._imu1_filtered = imu_upper
        else:
            self._imu1_filtered = (
                self._imu1_filtered * (1.0 - alpha)
                + imu_upper * alpha)

        # 2. Push the latest sample, drop old ones beyond WINDOW_S
        self._window.append((now, self._imu1_filtered, motor_deg))
        cutoff = now - self.WINDOW_S
        while self._window and self._window[0][0] < cutoff:
            self._window.popleft()

        # 3. Window-based metrics
        self.motor_speed_now = abs(motor_vel) * 360.0
        self._current_motor_deg = motor_deg

        if len(self._window) < 5:
            self._live_score = 0.0
            self.shoulder_delta = 0.0
            self.elbow_delta_max = 0.0
        else:
            imu1_vals  = [s[1] for s in self._window]
            motor_vals = [s[2] for s in self._window]

            imu1_now             = imu1_vals[-1]
            imu1_min_in_window   = min(imu1_vals)
            imu1_start_of_window = imu1_vals[0]

            motor_min = min(motor_vals)
            motor_max = max(motor_vals)

            # Recent shoulder rise (window-based) — current minus window-low
            window_rise = max(0.0, imu1_now - imu1_min_in_window)

            # Absolute elevation above session baseline — catches sustained
            # raised positions even after the window forgets the rise
            abs_elev = max(0.0, imu1_now - self._session_baseline_imu1)

            # Use the bigger of the two
            effective_rise = max(window_rise, abs_elev)

            # Recent elbow range — only motion inside the window counts
            window_elbow_range = motor_max - motor_min

            # Net direction across the window (return-to-rest check)
            net_direction = imu1_now - imu1_start_of_window

            if net_direction < self.DESCENDING_THRESHOLD:
                # Arm is net descending - treat as return motion, no alarm
                self._live_score = 0.0
            else:
                self._live_score = self._compute_score(
                    shoulder_rise   = effective_rise,
                    elbow_excursion = window_elbow_range)

            self.shoulder_delta  = effective_rise
            self.elbow_delta_max = window_elbow_range

        # 4. Peak hold + decay on the displayed score
        if self._live_score >= self.score:
            self.score = self._live_score
            self._score_peak_t = now
        else:
            hold_remaining = self.PEAK_HOLD_S - (now - self._score_peak_t)
            if hold_remaining <= 0:
                decay_amount = (100.0 / self.DECAY_S) * dt
                self.score = max(self._live_score, self.score - decay_amount)
            # else: still in hold window, keep score unchanged

        # 5. Classify level
        if self.reach_count == 0 and self.score < 1.0:
            new_level = "REST"
        else:
            new_level = self._classify(self.score)
        self.level = new_level

        # 6. Rising-edge event counter (compensation events)
        is_alarm = (new_level == "COMPENSATION")
        if is_alarm and not self._was_in_alarm:
            self.event_count += 1
        self._was_in_alarm = is_alarm

        # 7. Burst detection (for the history chart only)
        imu1_omega = self._imu1_angular_speed()
        combined_omega = self.motor_speed_now + imu1_omega

        if not self._burst_active:
            if combined_omega > self.BURST_START_OMEGA_DEG:
                self._burst_active = True
                self._burst_start_t = now
                self._burst_max_score = self.score
                self._burst_end_pending_t = 0.0
                self.reach_active = True
                self.reach_state = "ACTIVE"
        else:
            # Track max displayed score during the burst
            if self.score > self._burst_max_score:
                self._burst_max_score = self.score

            # End detection
            if combined_omega < self.BURST_END_OMEGA_DEG:
                if self._burst_end_pending_t == 0.0:
                    self._burst_end_pending_t = now
                elif now - self._burst_end_pending_t > self.BURST_END_DEBOUNCE_S:
                    burst_dur = now - self._burst_start_t
                    if burst_dur >= self.BURST_MIN_DUR_S:
                        self._finalize_burst()
                    self._burst_active = False
                    self._burst_end_pending_t = 0.0
                    self.reach_active = False
                    self.reach_state = "IDLE"
            else:
                self._burst_end_pending_t = 0.0

        # 8. CSV log
        self._log.append((now - self.session_t0, imu_upper, imu_lower,
                          motor_deg, self.score, self.level))

    def _imu1_angular_speed(self) -> float:
        """Approximate IMU1 angular speed across the window, in deg/s."""
        if len(self._window) < 2:
            return 0.0
        t0, v0, _ = self._window[0]
        t1, v1, _ = self._window[-1]
        dt = t1 - t0
        if dt < 0.05:
            return 0.0
        return abs(v1 - v0) / dt

    def _finalize_burst(self):
        """Log the burst's max displayed score as one bar in the history."""
        final = self._burst_max_score

        if final >= self.level_alarm:
            level = "COMPENSATION"
        elif final >= self.level_warn:
            level = "WARNING"
        else:
            level = "GOOD"

        self.last_reach_score = final
        self.history.append(final)
        self.history_levels.append(level)
        self.reach_count += 1

    def _classify(self, score_pct: float) -> str:
        if score_pct >= self.level_alarm:
            return "COMPENSATION"
        if score_pct >= self.level_warn:
            return "WARNING"
        return "GOOD"

    def _expected_elbow_ratio(self) -> float:
        """Levin RPSS zone-aware elbow/shoulder ratio."""
        imu1 = self._imu1_filtered if self._imu1_filtered is not None else 0.0
        if imu1 < self.ZONE_LOW_CUTOFF:
            return self.ZONE_RATIO_LOW
        if imu1 < self.ZONE_HIGH_CUTOFF:
            return self.ZONE_RATIO_MID
        return self.ZONE_RATIO_HIGH

    def _compute_score(self, shoulder_rise: float, elbow_excursion: float) -> float:
        """Core scoring rule:
          - how much the shoulder has risen (minus the free margin),
          - reduced by how much the elbow is contributing, weighted by
            the workspace zone.

        Elbow contribution is the bigger of two signals:
          - motion credit  - how much the elbow moved recently (range
            of motion in the window, with wobble subtracted),
          - position credit - how flexed the elbow currently is (above
            the position threshold).
        Either one counts as "the elbow is in use", so a patient who
        bent the elbow first and then lifts the shoulder for a
        hand-to-head motion does not trigger an alarm.
        """
        if shoulder_rise > self.SHOULDER_RISE_FREE:
            excess = shoulder_rise - self.SHOULDER_RISE_FREE
            shoulder_score = min(100.0,
                                  excess * 100.0 / self.SHOULDER_RISE_TO_MAX)
        else:
            shoulder_score = 0.0

        ratio    = self._expected_elbow_ratio()
        required = max(self.ELBOW_MIN_FLOOR, shoulder_rise * ratio)

        motion_credit   = max(0.0, elbow_excursion - self.ELBOW_WOBBLE)
        position_credit = max(0.0,
                              self._current_motor_deg - self.ELBOW_POSITION_THRESHOLD)
        effective_elbow = max(motion_credit, position_credit)

        elbow_credit = min(1.0, effective_elbow / required) if required > 0 else 1.0

        return shoulder_score * (1.0 - elbow_credit)

    # ── Export ────────────────────────────────────────────────────────

    def export_report(self, path: Path):
        """Write a multi-panel PNG report of the session for clinical review.

        Three labelled panels:
          - Top (full width):    Per-reach compensation score (coloured
                                 bar chart). The main clinical view -
                                 which reaches went well, which did not.
          - Bottom-left:         Shoulder elevation (upper-arm IMU) over
                                 time.
          - Bottom-right:        Elbow flexion (motor angle) over time.

        A two-line header at the top shows "HapticElbow - Session
        Report" with the session statistics underneath.

        The dashboard runs in a dark theme, but the report is rendered
        on a light background so it prints cleanly. The
        ``plt.rc_context`` block overrides the global dark style for
        this one figure only.
        """
        import matplotlib.pyplot as plt
        from matplotlib.figure import Figure

        if not self._log:
            return

        # Pull the timeline data out of the log
        times = [row[0] for row in self._log]
        imu1  = [row[1] for row in self._log]
        motor = [row[3] for row in self._log]

        duration = times[-1] if times else 0.0
        reach_levels = list(self.history_levels)
        n_good = sum(1 for l in reach_levels if l == "GOOD")
        n_warn = sum(1 for l in reach_levels if l == "WARNING")
        n_comp = sum(1 for l in reach_levels if l == "COMPENSATION")

        # Colour palette - readable on a white printed page
        C_GOOD = "#2ea043"
        C_WARN = "#bb8409"
        C_COMP = "#cf222e"
        C_SHLD = "#8250df"
        C_ELBW = "#1a7f37"
        C_TEXT = "#1f2328"
        C_GRID = "#d0d7de"
        C_AXIS = "#57606a"

        # Override the global dark-theme rcParams just for this report
        light_theme = {
            "figure.facecolor":   "white",
            "savefig.facecolor":  "white",
            "axes.facecolor":     "white",
            "axes.edgecolor":     C_AXIS,
            "axes.labelcolor":    C_TEXT,
            "axes.titlecolor":    C_TEXT,
            "text.color":         C_TEXT,
            "xtick.color":        C_AXIS,
            "ytick.color":        C_AXIS,
            "grid.color":         C_GRID,
            "legend.facecolor":   "white",
            "legend.edgecolor":   C_GRID,
            "legend.labelcolor":  C_TEXT,
        }

        with plt.rc_context(light_theme):
            fig = Figure(figsize=(14, 9), dpi=120, constrained_layout=True)
            fig.patch.set_facecolor("white")

            # Bar chart on the top row (full width), angle plots below.
            # Top row is 1.4x taller because the bar chart is the
            # headline view a clinician will look at first.
            gs = fig.add_gridspec(2, 2, height_ratios=[1.4, 1.0])

            # ── Top : per-reach bar chart (full width) ───────────────────
            ax1 = fig.add_subplot(gs[0, :])
            reach_scores = list(self.history)
            if reach_scores:
                x = list(range(1, len(reach_scores) + 1))
                bar_colors = [C_COMP if s >= self.level_alarm else
                              C_WARN if s >= self.level_warn else
                              C_GOOD for s in reach_scores]
                ax1.bar(x, reach_scores, color=bar_colors, width=0.7,
                        edgecolor=C_TEXT, linewidth=0.7)
                for xi, s in zip(x, reach_scores):
                    label_y = s + 2.0 if s < 92 else s - 6
                    ax1.text(xi, label_y, f"{s:.0f}%", ha="center",
                             va="bottom" if label_y > s else "top",
                             fontsize=10, fontweight="bold", color=C_TEXT)
                ax1.set_xticks(x if len(x) <= 20 else
                                list(range(1, len(x) + 1, max(1, len(x) // 15))))
            ax1.axhline(self.level_warn,  color=C_WARN, ls="--", lw=1.2,
                        alpha=0.8,
                        label=f"Warning ({self.level_warn:.0f}%)")
            ax1.axhline(self.level_alarm, color=C_COMP, ls="--", lw=1.2,
                        alpha=0.8,
                        label=f"Compensation ({self.level_alarm:.0f}%)")
            ax1.set_ylim(0, 110)
            ax1.set_xlabel("Reach number")
            ax1.set_ylabel("Compensation score (%)")
            ax1.set_title("Per-reach compensation score",
                          fontweight="bold", fontsize=13,
                          loc="left", pad=6)
            ax1.legend(loc="upper right", fontsize=9)
            ax1.grid(True, axis="y", alpha=0.5)

            # ── Bottom-left : shoulder elevation (IMU1) ──────────────────
            ax3 = fig.add_subplot(gs[1, 0])
            ax3.plot(times, imu1, color=C_SHLD, linewidth=1.0)
            ax3.fill_between(times, imu1, color=C_SHLD, alpha=0.15)
            ax3.set_xlabel("Time (s)")
            ax3.set_ylabel("Upper-arm angle (deg)")
            ax3.set_title("Shoulder elevation (upper-arm IMU)",
                          fontweight="bold", fontsize=12,
                          loc="left", pad=6)
            ax3.grid(True, alpha=0.5)

            # ── Bottom-right : elbow flexion (motor) ─────────────────────
            ax4 = fig.add_subplot(gs[1, 1])
            ax4.plot(times, motor, color=C_ELBW, linewidth=1.0)
            ax4.fill_between(times, motor, color=C_ELBW, alpha=0.15)
            ax4.set_xlabel("Time (s)")
            ax4.set_ylabel("Elbow flexion (deg)")
            ax4.set_title("Elbow angle (motor encoder)",
                          fontweight="bold", fontsize=12,
                          loc="left", pad=6)
            ax4.grid(True, alpha=0.5)

            # ── Header : title on line 1, stats on line 2 ────────────────
            # No explicit y= so constrained_layout reserves the right
            # amount of space and the panel titles below stay clear.
            m, s = divmod(int(duration), 60)
            stats = (f"Duration {m:d}:{s:02d}      "
                     f"{self.reach_count} reaches      "
                     f"Good {n_good}  /  Warning {n_warn}  /  "
                     f"Compensation {n_comp}")
            fig.suptitle(f"HapticElbow  -  Session Report\n{stats}",
                         fontsize=13, fontweight="bold", color=C_TEXT)

            fig.savefig(str(path), dpi=120, facecolor="white")

# ===========================================================================
#  5b.  SMOOTHNESS MONITOR  (SPARC — Spectral Arc Length)
#       Kept as a no-op metric for possible future use; the UI no longer
#       displays it (see README §3.9).
# ===========================================================================

class SmoothnessMonitor:
    """SPARC (Spectral Arc Length) — Balasubramanian et al. 2012.

    Calculated on the magnitude spectrum of the velocity:
    a smooth motion has a compact, fast-decaying spectrum (short arc
    length); a jerky motion has lots of high-frequency content
    (long arc length).

    Higher (less negative) = smoother = better motor control.
    """

    SAMPLE_RATE_HZ  = 10.0    # matches Arduino telemetry (POS @ 10 Hz)
    FC_HZ           = 5.0     # cutoff for the spectrum segment
    WINDOW_SECONDS  = 15.0    # rolling window — longer = more stable
    UPDATE_MIN_VEL  = 0.03    # below this (rev/s) there is no motion

    def __init__(self):
        self._buf: deque = deque(
            maxlen=int(self.SAMPLE_RATE_HZ * self.WINDOW_SECONDS))
        self.current = 0.0
        self.history: deque = deque(maxlen=180)

    def push(self, vel: float):
        self._buf.append(vel)

    AMP_TH = 0.05   # adaptive cutoff: trim spectrum where magnitude < 5%

    def recompute(self):
        """Compute SPARC on the current buffer contents.
        Call on a fixed interval (e.g. 500 ms)."""
        if len(self._buf) < 16:
            return
        try:
            import numpy as np
            v = np.array(self._buf, dtype=float)
            if float(np.max(np.abs(v))) < self.UPDATE_MIN_VEL:
                return

            N = len(v)
            nfft = int(2 ** (math.ceil(math.log2(max(N, 2))) + 4))

            Mf = np.abs(np.fft.rfft(v, n=nfft))
            mx = float(Mf.max())
            if mx <= 0:
                return
            Mf = Mf / mx
            freqs = np.fft.rfftfreq(nfft, d=1.0 / self.SAMPLE_RATE_HZ)

            mask = freqs <= self.FC_HZ
            Mf = Mf[mask]; freqs = freqs[mask]
            if len(Mf) < 4:
                return

            above = Mf >= self.AMP_TH
            if not above.any():
                return
            last_idx = int(np.where(above)[0].max())
            if last_idx < 2:
                return
            Mf_sel = Mf[:last_idx + 1]
            f_sel  = freqs[:last_idx + 1]

            span = float(f_sel[-1] - f_sel[0])
            if span <= 0:
                return
            df_norm = np.diff(f_sel) / span
            dMf     = np.diff(Mf_sel)
            arc = float(np.sum(np.sqrt(df_norm * df_norm + dMf * dMf)))
            self.current = -arc
            self.history.append(self.current)
        except Exception:
            pass

    @property
    def session_average(self) -> float:
        if not self.history:
            return 0.0
        return sum(self.history) / len(self.history)


# ===========================================================================
#  6.  GAME / ROM STATE  (lightweight dataclasses)
# ===========================================================================

@dataclass
class GameState:
    mode: str = "Catch Blocks"
    score: int = 0
    ghost_score: int = 0
    target_y: int = 150
    hold_frames: int = 0


@dataclass
class ROMState:
    active: bool = False
    rom_min: float = 100.0
    rom_max: float = 0.0


# ===========================================================================
#  7.  MAIN APP
# ===========================================================================

class RehabApp(ctk.CTk):
    """Main window + every tab. One `_build_*`-method per tab."""

    # ── Construction ─────────────────────────────────────────────────

    def __init__(self):
        super().__init__()
        self.title("HapticElbow  ·  Rehab Control Panel")
        self.geometry("1440x960")
        self.configure(fg_color=THEME["bg_app"])

        # ── State ─────────────────────────────────────────────────
        self.sock_visual = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.serial      = SerialClient(COM_PORT, BAUDRATE)
        self.buf         = TelemetryBuffer()
        self.comp        = CompensationMonitor()
        self.smooth      = SmoothnessMonitor()
        self.game        = GameState()
        self.rom         = ROMState()
        self.start_time  = time.time()

        # IMU offsets (tare buttons) + most recent values
        self.imu1_offset = 0.0
        self.imu2_offset = 0.0
        self.raw_imu1    = 0.0
        self.raw_imu2    = 0.0
        self.current_imu1 = 0.0
        self.current_imu2 = 0.0
        self.current_motor_angle = 0.0
        self.last_motor_vel = 0.0    # rev/s — used by compensation detector
        self.current_target = 0.0

        # ── Build UI ──────────────────────────────────────────────
        plt.style.use("dark_background")
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        self._build_sidebar()
        self._build_main_frame()

        # ── Start background loops ────────────────────────────────
        self.serial.last_rx_time = time.time()   # avoid an immediate "OFFLINE"
        self.after(30,  self._loop_serial)
        self.after(1000, self._loop_check_connection)
        self.after(500,  self._loop_graphs)
        self.after(50,   self._loop_3d)
        self.after(150,  self._loop_compensation)

    # ===================================================================
    #  7a.  SIDEBAR
    # ===================================================================

    def _build_sidebar(self):
        """Build the left sidebar: branding, the connection/status cards, and the
        main action buttons (Calibrate, Start, Stop, Reset)."""
        sb = ctk.CTkFrame(self, width=260, corner_radius=0,
                          fg_color=THEME["bg_sidebar"])
        sb.pack(side="left", fill="y")
        sb.pack_propagate(False)

        # Branding
        logo = ctk.CTkFrame(sb, fg_color="transparent")
        logo.pack(pady=(28, 4), padx=24, fill="x")
        ctk.CTkLabel(logo, text="HapticElbow", font=FONTS["h1"],
                     text_color=THEME["accent"]).pack(anchor="w")
        ctk.CTkLabel(logo, text="Rehab Interface", font=FONTS["caption"],
                     text_color=THEME["text_secondary"]).pack(anchor="w")

        ctk.CTkFrame(sb, height=1, fg_color=THEME["border_subtle"]).pack(
            fill="x", padx=20, pady=(20, 18))

        # Status card
        sc = ctk.CTkFrame(sb, fg_color=THEME["bg_card"],
                          corner_radius=10, border_width=1,
                          border_color=THEME["border_subtle"])
        sc.pack(fill="x", padx=16, pady=(0, 14))
        ctk.CTkLabel(sc, text="SYSTEM STATUS", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(
            anchor="w", padx=14, pady=(12, 2))
        sr = ctk.CTkFrame(sc, fg_color="transparent")
        sr.pack(fill="x", padx=14, pady=(0, 14))
        self.status_dot = ctk.CTkLabel(sr, text="●",
                                       font=(FONT_FAMILY, 18, "bold"),
                                       text_color=THEME["text_muted"])
        self.status_dot.pack(side="left", padx=(0, 8))
        self.lbl_status = ctk.CTkLabel(sr, text="Connecting...",
                                       font=FONTS["body_bold"],
                                       text_color=THEME["text_secondary"],
                                       wraplength=170, anchor="w",
                                       justify="left")
        self.lbl_status.pack(side="left", fill="x", expand=True)

        # Session-time card (extra live KPI)
        tk_card = ctk.CTkFrame(sb, fg_color=THEME["bg_card"],
                               corner_radius=10, border_width=1,
                               border_color=THEME["border_subtle"])
        tk_card.pack(fill="x", padx=16, pady=(0, 22))
        ctk.CTkLabel(tk_card, text="CONNECTED", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(
            anchor="w", padx=14, pady=(12, 2))
        self.lbl_uptime = ctk.CTkLabel(tk_card, text="00:00",
                                       font=FONTS["h2"],
                                       text_color=THEME["accent"])
        self.lbl_uptime.pack(anchor="w", padx=14, pady=(0, 12))

        # Actions
        ctk.CTkLabel(sb, text="ACTIONS", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(
            anchor="w", padx=20, pady=(0, 6))

        ctk.CTkButton(sb, text="1.  Calibrate  (to 0°)",
                      height=42, font=FONTS["body_bold"],
                      fg_color=THEME["bg_card"], hover_color=THEME["bg_card_alt"],
                      border_width=1, border_color=THEME["border"],
                      text_color=THEME["text_primary"], anchor="w",
                      command=lambda: self.serial.send("STRAIGHT")
                      ).pack(pady=4, padx=16, fill="x")

        self.btn_start = ctk.CTkButton(
            sb, text="▶  START  THERAPY",
            fg_color=THEME["success"], hover_color=THEME["success_hover"],
            height=46, font=FONTS["h3"], text_color="#0e1117",
            command=lambda: self.serial.send("start"))
        self.btn_start.pack(pady=(10, 4), padx=16, fill="x")

        ctk.CTkButton(sb, text="⏹  STOP  /  EMERGENCY",
                      fg_color=THEME["danger"], hover_color=THEME["danger_hover"],
                      height=46, font=FONTS["h3"],
                      command=lambda: self.serial.send("stop")
                      ).pack(pady=4, padx=16, fill="x")

        ctk.CTkButton(sb, text="System  Reset",
                      fg_color="transparent", border_width=1,
                      border_color=THEME["danger"],
                      text_color=THEME["danger"],
                      hover_color=THEME["bg_card"],
                      height=34, font=FONTS["caption_bold"],
                      command=lambda: self.serial.send("RESET")
                      ).pack(pady=(0, 24), padx=16, fill="x", side="bottom")

    # ===================================================================
    #  7b.  MAIN FRAME  (header + tabs)
    # ===================================================================

    def _build_main_frame(self):
        """Build the right-hand area: the header bar on top, the tab strip below."""
        main = ctk.CTkFrame(self, fg_color="transparent")
        main.pack(side="right", fill="both", expand=True, padx=24, pady=24)

        self._build_header(main)
        self._build_tabs(main)

    def _build_header(self, parent):
        """Build the always-visible header: live angle, IMU read-outs, the central
        tare buttons, the torque/target KPIs and the small 2D arm sketch."""
        card = make_card(parent)
        card.pack(fill="x", pady=(0, 16), ipady=8)

        # Left side: position + IMUs
        col = ctk.CTkFrame(card, fg_color="transparent")
        col.pack(side="left", padx=32, pady=18)

        ctk.CTkLabel(col, text="CURRENT  POSITION", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(anchor="w")
        self.lbl_degrees = ctk.CTkLabel(col, text="0.0°", font=FONTS["display"],
                                        text_color=THEME["accent"])
        self.lbl_degrees.pack(anchor="w", pady=(2, 8))

        imu = ctk.CTkFrame(col, fg_color="transparent")
        imu.pack(anchor="w")
        self.lbl_imu1 = ctk.CTkLabel(imu, text="Upper Arm     --°",
                                     font=(FONT_FAMILY, 12),
                                     text_color=THEME["text_secondary"])
        self.lbl_imu1.pack(anchor="w")
        self.lbl_imu2 = ctk.CTkLabel(imu, text="Lower Arm     --°",
                                     font=(FONT_FAMILY, 12),
                                     text_color=THEME["text_secondary"])
        self.lbl_imu2.pack(anchor="w")

        # ── Central tare buttons ──
        # One place where the IMU orientation is fixed. Every subsystem
        # (compensation monitor, 3D arm, motion visualisations) uses the
        # same offsets — no duplicate calibration.
        tare_row = ctk.CTkFrame(col, fg_color="transparent")
        tare_row.pack(anchor="w", pady=(10, 0))
        btn_tv = ctk.CTkButton(
            tare_row, text="↓  Tare Vertical",
            width=150, height=30, font=FONTS["caption_bold"],
            fg_color=THEME["bg_elevated"], hover_color=THEME["bg_card_alt"],
            border_width=1, border_color=THEME["border"],
            text_color=THEME["text_primary"],
            command=lambda: self._tare_pose(0, 0))
        btn_tv.pack(side="left", padx=(0, 6))
        Tooltip(btn_tv,
                "Store the current pose as vertical (arm down). "
                "Upper arm = 0°, lower arm = 0°. Applies to the entire dashboard.")

        btn_th = ctk.CTkButton(
            tare_row, text="→  Tare Horizontal",
            width=160, height=30, font=FONTS["caption_bold"],
            fg_color=THEME["bg_elevated"], hover_color=THEME["bg_card_alt"],
            border_width=1, border_color=THEME["border"],
            text_color=THEME["text_primary"],
            command=lambda: self._tare_pose(90, 0))
        btn_th.pack(side="left")
        Tooltip(btn_th,
                "Store the current pose as horizontal forward. "
                "Upper arm = 90°, lower arm = 0°. Applies to the entire dashboard.")

        # Short feedback label, populated on tare confirmation
        self.lbl_tare_feedback = ctk.CTkLabel(
            col, text="", font=FONTS["small"],
            text_color=THEME["success"])
        self.lbl_tare_feedback.pack(anchor="w", pady=(4, 0))

        # Mid: KPIs (torque, target angle)
        kpi = ctk.CTkFrame(card, fg_color="transparent")
        kpi.pack(side="left", padx=32, pady=18, fill="y")
        self.lbl_kpi_trq    = self._kpi_block(kpi, "TORQUE",       "0.00 Nm",
                                              THEME["chart_trq"])
        self.lbl_kpi_target = self._kpi_block(kpi, "TARGET ANGLE", "--°",
                                              THEME["chart_target"])

        # Right side: 2D arm visual
        cv = ctk.CTkFrame(card, fg_color="transparent")
        cv.pack(side="right", padx=32, pady=18)
        self.canvas_arm = tk.Canvas(cv, width=260, height=130,
                                    bg=THEME["bg_card"], highlightthickness=0)
        self.canvas_arm.pack()
        self.canvas_arm.create_line(50, 100, 150, 100, width=10,
                                     fill=THEME["text_muted"], capstyle=tk.ROUND)
        self.canvas_arm.create_oval(140, 90, 160, 110,
                                     fill=THEME["text_primary"], outline="")
        self._update_arm_visual(0.0)

    def _kpi_block(self, parent, label: str, value: str,
                   color: str) -> ctk.CTkLabel:
        """Make one small labelled KPI read-out for the header."""
        col = ctk.CTkFrame(parent, fg_color="transparent")
        col.pack(side="left", padx=(0, 24))
        ctk.CTkLabel(col, text=label, font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(anchor="w")
        lbl = ctk.CTkLabel(col, text=value, font=FONTS["h2"], text_color=color)
        lbl.pack(anchor="w")
        return lbl

    def _build_tabs(self, parent):
        """Create the tab strip and build each of the nine tabs into it."""
        tv = ctk.CTkTabview(
            parent, corner_radius=12,
            fg_color=THEME["bg_card"],
            segmented_button_fg_color=THEME["bg_sidebar"],
            segmented_button_selected_color=THEME["accent"],
            segmented_button_selected_hover_color=THEME["accent_hover"],
            segmented_button_unselected_color=THEME["bg_sidebar"],
            segmented_button_unselected_hover_color=THEME["bg_card_alt"],
            text_color=THEME["text_primary"])
        tv.pack(fill="both", expand=True)
        self.tabview = tv

        self._build_tab_info        (tv.add("Project Info"))
        self._build_tab_settings    (tv.add("Settings"))
        self._build_tab_3d          (tv.add("3D Arm Simulation"))
        self._build_tab_position    (tv.add("Position Control"))
        self._build_tab_rom         (tv.add("ROM Test"))
        self._build_tab_compensation(tv.add("Anti-Compensation"))
        self._build_tab_game        (tv.add("Game Mode"))
        self._build_tab_graphs      (tv.add("Live Graphs"))
        self._build_tab_pid         (tv.add("ODrive Tuning"))

    # ===================================================================
    #  7c.  TAB  ·  PROJECT INFO
    # ===================================================================

    def _build_tab_info(self, tab):
        """Build the Project Info tab: a read-only summary of the project."""
        info = (
            "Adaptive Stroke Rehabilitation Exoskeleton            "
            "Niel Boon & Senne Peeters\n\n"
            "1. Medical Challenge: Assist-as-Needed (AAN) Training\n"
            "Stroke patients frequently suffer from hemiparesis and fluctuating "
            "muscle strength due to neurological fatigue. Static-resistance tools "
            "are either too strenuous or insufficient for effective recovery.\n\n"
            "This project focuses on Assist-as-Needed (AAN) rehabilitation. The "
            "interface is transparent when the patient moves correctly and provides "
            "active haptic support the moment a deviation from the desired "
            "trajectory is detected — maximising neuroplasticity by keeping the "
            "patient an active participant.\n\n"
            "2. Technical Approach\n"
            "A wearable elbow exoskeleton uses an impedance-control model. The "
            "ODrive S1 motor controller drives the mechanism with back-drivable, "
            "current-controlled actuation. Two MPU-6050 sensors on the upper and "
            "lower arm track joint kinematics in real time.\n\n"
            "Control modes:\n"
            "  • Assistive — calculated torque boost when the patient cannot "
            "reach or hold a target trajectory.\n"
            "  • Resistive — variable damping for strengthening therapy.\n"
            "  • Free-ride / Transparent — minimal mechanical impedance.\n\n"
            "3. Shoulder-Compensation Monitor\n"
            "The dashboard fuses both IMUs and the motor angle into a per-reach "
            "compensation score with zone-aware ratio, wobble subtraction and "
            "direction-aware finalisation (Schwarz 2020 / Levin RPSS / "
            "Cirstea & Levin 2000). See README §3.6.1."
        )

        wrap = ctk.CTkFrame(tab, fg_color="transparent")
        wrap.pack(fill="both", expand=True, padx=18, pady=18)
        tb = ctk.CTkTextbox(wrap, font=(FONT_FAMILY, 13), wrap="word",
                            fg_color=THEME["bg_card_alt"],
                            text_color=THEME["text_primary"],
                            border_width=1, border_color=THEME["border_subtle"],
                            corner_radius=10)
        tb.pack(fill="both", expand=True)
        tb.insert("1.0", info)
        tb.configure(state="disabled")

    # ===================================================================
    #  7d.  TAB  ·  SETTINGS
    # ===================================================================

    def _build_tab_settings(self, tab):
        """Build the Settings tab: mode switch plus every therapy parameter slider.
        Sliders are disabled until "Adjust Settings" is pressed, then sent on Save."""
        s = ctk.CTkScrollableFrame(
            tab, fg_color="transparent",
            scrollbar_button_color=THEME["border"],
            scrollbar_button_hover_color=THEME["text_muted"])
        s.pack(fill="both", expand=True, padx=8, pady=8)

        make_section_title(s, "Therapy Settings",
                           "Pick a mode and adjust the parameters as needed.")

        self.mode_switch = ctk.CTkSegmentedButton(
            s, values=["Assistive (Help)", "Resistive (Train)"],
            command=self._switch_mode, font=FONTS["body_bold"],
            selected_color=THEME["accent"],
            selected_hover_color=THEME["accent_hover"],
            unselected_color=THEME["bg_elevated"],
            unselected_hover_color=THEME["bg_card_alt"])
        self.mode_switch.set("Assistive (Help)")
        self.mode_switch.pack(pady=(4, 14), padx=8, fill="x")

        self.btn_change = ctk.CTkButton(
            s, text="Adjust Settings",
            fg_color=THEME["warning"], hover_color=THEME["warning_hover"],
            text_color="#1a1a1a", height=36, font=FONTS["body_bold"],
            command=self._activate_settings)
        self.btn_change.pack(pady=(0, 16), padx=8, fill="x")

        self.sliders: list = []

        # ── Range of motion ──
        frame_limits = self._settings_card(s, "Range of Motion  ·  Soft Limits")
        self.lbl_lim_min = ctk.CTkLabel(
            frame_limits, text="Minimum Angle (Extension)     0.00",
            font=FONTS["body"], text_color=THEME["text_primary"], anchor="w")
        self.lbl_lim_min.pack(fill="x", padx=14, pady=(8, 2))
        self.sl_lim_min = self._slider(frame_limits, 0, 100,
                                       self._check_lim_min)
        self.sl_lim_min.set(0)
        self.sl_lim_min.pack(fill="x", padx=14, pady=(0, 12))

        self.lbl_lim_max = ctk.CTkLabel(
            frame_limits, text="Maximum Angle (Flexion)     100.00",
            font=FONTS["body"], text_color=THEME["text_primary"], anchor="w")
        self.lbl_lim_max.pack(fill="x", padx=14, pady=(2, 2))
        self.sl_lim_max = self._slider(frame_limits, 0, 100,
                                       self._check_lim_max)
        self.sl_lim_max.set(100)
        self.sl_lim_max.pack(fill="x", padx=14, pady=(0, 14))

        # ── Assistive ──
        frame_a = self._settings_card(s, "Assistive Mode")
        self.sl_a_torque = self._labeled_slider(frame_a,
            "Assist Torque (Nm)",         0.0, 4.0, 0.60)
        self.sl_a_factor = self._labeled_slider(frame_a,
            "Assist Ramp Factor",         0.1, 2.0, 1.00)
        self.sl_a_damp   = self._labeled_slider(frame_a,
            "Extension Damping",          0.0, 4.0, 0.40)

        # ── Resistive ──
        frame_r = self._settings_card(s, "Resistive Mode")
        self.sl_t_resist = self._labeled_slider(frame_r,
            "Training Resistance (heavy)", 0.5, 4.0, 2.50)

        # ── Safety ──
        frame_v = self._settings_card(s, "General  &  Safety")
        self.sl_v_spring = self._labeled_slider(frame_v,
            "Soft-Limit Stiffness", 0.05, 0.50, 0.10)
        self.sl_v_torque = self._labeled_slider(frame_v,
            "Max Torque (Nm)",      0.50, 4.00, 1.50)
        self.sl_v_haptic = self._labeled_slider(frame_v,
            "Haptic Strength",      0.10, 1.00, 1.00)
        # Shock Reduction (high-frequency damping gain in the Arduino).
        # Default 1.5 = the current well-working value.
        # Higher = more stable / fewer shocks with a loose mount, but
        # feels slightly less responsive. Lower = more direct support,
        # but more vibration-prone with light coupling.
        self.sl_v_shock  = self._labeled_slider(frame_v,
            "Shock Reduction",      0.00, 3.00, 1.50)

        self.btn_save = ctk.CTkButton(
            s, text="Save  &  Ready",
            fg_color=THEME["accent"], hover_color=THEME["accent_hover"],
            height=42, font=FONTS["h3"], command=self._save_settings)
        self.btn_save.pack(pady=(16, 8), padx=8, fill="x")
        self.btn_save.configure(state="disabled")

    def _settings_card(self, parent, title: str) -> ctk.CTkFrame:
        """Make one titled card to group related sliders inside a settings tab."""
        card = make_card(parent)
        card.pack(fill="x", padx=8, pady=6)
        ctk.CTkLabel(card, text=title, font=FONTS["h3"],
                     text_color=THEME["accent"]).pack(anchor="w", padx=14,
                                                       pady=(12, 4))
        return card

    def _slider(self, parent, mn, mx, cmd) -> ctk.CTkSlider:
        """Make a plain slider (starts disabled) and register it for enable/disable."""
        sl = ctk.CTkSlider(parent, from_=mn, to=mx, command=cmd,
                           progress_color=THEME["accent"],
                           button_color=THEME["accent"],
                           button_hover_color=THEME["accent_hover"])
        sl.configure(state="disabled")
        self.sliders.append(sl)
        return sl

    def _labeled_slider(self, parent, text, mn, mx, start) -> ctk.CTkSlider:
        """Make a slider with a label that shows its live value next to its name."""
        lbl = ctk.CTkLabel(parent, text=f"{text}     {start:.2f}",
                           font=FONTS["body"],
                           text_color=THEME["text_primary"], anchor="w")
        lbl.pack(fill="x", padx=14, pady=(8, 2))
        sl = ctk.CTkSlider(
            parent, from_=mn, to=mx,
            command=lambda v: lbl.configure(text=f"{text}     {v:.2f}"),
            progress_color=THEME["accent"],
            button_color=THEME["accent"],
            button_hover_color=THEME["accent_hover"])
        sl.set(start)
        sl.pack(fill="x", padx=14, pady=(0, 12))
        sl.configure(state="disabled")
        self.sliders.append(sl)
        return sl

    # ===================================================================
    #  7e.  TAB  ·  3D ARM SIMULATION
    # ===================================================================

    def _build_tab_3d(self, tab):
        """Build the 3D arm tab: the live stick figure plus the Mirror Therapy toggle."""
        make_section_title(tab, "3D Arm Simulation",
                           "Use the tare buttons at the top of the screen to "
                           "calibrate the IMU orientation — they apply to "
                           "every visualisation.")

        ctrl = ctk.CTkFrame(tab, fg_color="transparent")
        ctrl.pack(pady=(0, 8), fill="x")

        # Mirror Therapy toggle — shows a mirrored arm on the other side
        # for visual bilateral therapy (Ramachandran 1996,
        # Thieme et al. 2018 Cochrane).
        self.mirror_active = False
        self.switch_mirror = ctk.CTkSwitch(
            ctrl, text="🪞  Mirror Therapy  (mirrored arm)",
            font=FONTS["body_bold"], command=self._toggle_mirror,
            progress_color=THEME["chart_assist"],
            button_color=THEME["chart_assist"],
            button_hover_color=THEME["chart_assist"])
        self.switch_mirror.pack(pady=(8, 4))
        Tooltip(self.switch_mirror,
                "Shows a mirrored copy of your movement on the other side "
                "of the body. Classic mirror therapy for neuroplasticity "
                "after stroke (Ramachandran 1996; Thieme et al. 2018 "
                "Cochrane). The patient visually sees a 'healthy' "
                "bilateral motion.")

        # Figure
        fig = Figure(figsize=(7, 7), dpi=100)
        fig.patch.set_facecolor(THEME["bg_card"])
        ax = fig.add_subplot(111, projection="3d")
        ax.set_facecolor(THEME["bg_card"])
        ax.set_xlim([-50, 50]); ax.set_ylim([-50, 50]); ax.set_zlim([-150, 30])
        ax.set_xlabel("X", color=THEME["text_secondary"])
        ax.set_ylabel("Y", color=THEME["text_secondary"])
        ax.set_zlabel("Z", color=THEME["text_secondary"])
        ax.tick_params(colors=THEME["text_muted"], labelsize=8)
        ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.set_edgecolor(THEME["border_subtle"])
            axis._axinfo['grid']['color']     = THEME["border_subtle"]
            axis._axinfo['grid']['linewidth'] = 0.4

        body, joint = THEME["text_muted"], THEME["text_secondary"]
        ax.plot([-12, 12], [0, 0], [-55, -55], '-', color=body, linewidth=8)
        ax.plot([0, 0],   [0, 0], [-55, 0],   '-', color=body, linewidth=8)
        ax.plot([-20, 20], [0, 0], [0, 0],   '-', color=body, linewidth=8)
        ax.plot([0, 0],   [0, 0], [0, 8],    '-', color=body, linewidth=6)
        ax.plot([0], [0], [15], 'o', color=joint, markersize=24)
        ax.plot([-20, -25, -22], [0, 0, 10], [0, -28, -50],
                'o-', color=body, linewidth=6, markersize=8)
        ax.plot([-10, -10, -10], [0, 0, 0], [-55, -100, -145],
                'o-', color=body, linewidth=7, markersize=8)
        ax.plot([10, 10, 10],   [0, 0, 0], [-55, -100, -145],
                'o-', color=body, linewidth=7, markersize=8)
        ax.plot([-20, 20], [0, 0], [0, 0], 'o', color=joint, markersize=11)
        ax.plot([-12, 12], [0, 0], [-55, -55], 'o', color=body, markersize=11)

        self.arm_line_3d, = ax.plot([], [], [], '-',
                                    color=THEME["accent"], linewidth=10)
        self.shoulder_3d, = ax.plot([], [], [], 'o',
                                    color=THEME["accent"], markersize=14)
        self.elbow_3d,    = ax.plot([], [], [], 'o',
                                    color=THEME["warning"], markersize=14)
        self.wrist_3d,    = ax.plot([], [], [], 'o',
                                    color=THEME["success"], markersize=12)

        # Mirror arm — hidden until the toggle is on. Separate objects so we
        # can show/hide them atomically.
        mirror_col = THEME["chart_assist"]
        self.mirror_line_3d,     = ax.plot([], [], [], '-',
                                            color=mirror_col, linewidth=10,
                                            alpha=0.85)
        self.mirror_shoulder_3d, = ax.plot([], [], [], 'o',
                                            color=mirror_col, markersize=14)
        self.mirror_elbow_3d,    = ax.plot([], [], [], 'o',
                                            color=THEME["warning"],
                                            markersize=14, alpha=0.85)
        self.mirror_wrist_3d,    = ax.plot([], [], [], 'o',
                                            color=THEME["success"],
                                            markersize=12, alpha=0.85)
        for obj in (self.mirror_line_3d, self.mirror_shoulder_3d,
                    self.mirror_elbow_3d, self.mirror_wrist_3d):
            obj.set_visible(False)

        self.canvas_3d = FigureCanvasTkAgg(fig, master=tab)
        self.canvas_3d.get_tk_widget().pack(fill="both", expand=True,
                                             padx=8, pady=(4, 8))

    # ===================================================================
    #  7f.  TAB  ·  POSITION CONTROL
    # ===================================================================

    def _build_tab_position(self, tab):
        """Build the Position Control tab: preset and slider-based target angles."""
        make_section_title(tab, "Manual Position Control",
                           "Make sure therapy is on 'Stop' before setting a "
                           "target angle.")

        pc = make_card(tab)
        pc.pack(pady=(4, 16), padx=18, fill="x")
        ctk.CTkLabel(pc, text="QUICK  PRESETS", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(
            anchor="w", padx=18, pady=(14, 8))
        row = ctk.CTkFrame(pc, fg_color="transparent")
        row.pack(pady=(0, 16), padx=12)
        for text, h in [("0°  ·  Straight", 0),
                        ("50°  ·  Middle",  50),
                        ("100°  ·  Flexed", 100)]:
            ctk.CTkButton(row, text=text, width=160, height=44,
                          font=FONTS["body_bold"],
                          fg_color=THEME["bg_elevated"],
                          hover_color=THEME["bg_card_alt"],
                          border_width=1, border_color=THEME["border"],
                          command=lambda hh=h: self._set_angle(hh)
                          ).pack(side="left", padx=6)

        sc = make_card(tab)
        sc.pack(pady=(0, 18), padx=18, fill="x", ipady=4)
        ctk.CTkLabel(sc, text="FINE  POSITIONING", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(
            anchor="w", padx=18, pady=(14, 8))
        sub = ctk.CTkFrame(sc, fg_color="transparent")
        sub.pack(fill="x", padx=18, pady=(0, 18))
        self.lbl_target_pos = ctk.CTkLabel(sub, text="Target  50.0°", width=110,
                                           font=FONTS["h3"],
                                           text_color=THEME["accent"])
        self.lbl_target_pos.pack(side="left", padx=(0, 16))
        self.slider_position = ctk.CTkSlider(
            sub, from_=0, to=100,
            command=lambda v: self.lbl_target_pos.configure(text=f"Target  {v:.1f}°"),
            progress_color=THEME["accent"],
            button_color=THEME["accent"],
            button_hover_color=THEME["accent_hover"])
        self.slider_position.set(50)
        self.slider_position.pack(side="left", fill="x", expand=True, padx=10)
        ctk.CTkButton(sub, text="Go to Angle", width=140, height=36,
                      font=FONTS["body_bold"],
                      fg_color=THEME["accent"], hover_color=THEME["accent_hover"],
                      command=lambda: self._set_angle(self.slider_position.get())
                      ).pack(side="right", padx=(16, 0))

    # ===================================================================
    #  7g.  TAB  ·  ROM TEST
    # ===================================================================

    def _build_tab_rom(self, tab):
        """Build the ROM test tab: start/stop buttons and a dial of the reached range."""
        make_section_title(tab, "Range of Motion  (ROM)",
                           "Press Start and move the arm freely through "
                           "its full range.")

        self.lbl_rom_instruction = ctk.CTkLabel(
            tab, text="Press Start and move freely.",
            font=FONTS["body"], text_color=THEME["text_secondary"])
        self.lbl_rom_instruction.pack(pady=(0, 4))

        row = ctk.CTkFrame(tab, fg_color="transparent")
        row.pack(pady=12)
        ctk.CTkButton(row, text="▶  START  ROM TEST",
                      width=180, height=40, font=FONTS["body_bold"],
                      fg_color=THEME["success"], hover_color=THEME["success_hover"],
                      text_color="#0e1117",
                      command=self._start_rom_test).pack(side="left", padx=8)
        ctk.CTkButton(row, text="⏹  STOP  &  SAVE",
                      width=180, height=40, font=FONTS["body_bold"],
                      fg_color=THEME["danger"], hover_color=THEME["danger_hover"],
                      command=self._stop_rom_test).pack(side="left", padx=8)

        self.lbl_rom_result = ctk.CTkLabel(
            tab, text="Extension    --°     ◆     Flexion    --°",
            font=FONTS["h2"], text_color=THEME["warning"])
        self.lbl_rom_result.pack(pady=(16, 8))

        self.canvas_rom = tk.Canvas(tab, width=300, height=300,
                                    bg=THEME["bg_card"], highlightthickness=0)
        self.canvas_rom.pack(pady=10)
        self.canvas_rom.create_oval(50, 50, 250, 250,
                                     outline=THEME["border"], width=1)
        self.canvas_rom.create_arc(50, 50, 250, 250, start=0, extent=100,
                                    fill=THEME["bg_elevated"], outline="")
        for deg, label in [(0, "0°"), (30, "30°"), (60, "60°"), (90, "90°")]:
            r, cx, cy = 110, 150, 150
            rad = math.radians(deg)
            tx, ty = cx + r * math.cos(rad), cy - r * math.sin(rad)
            self.canvas_rom.create_text(tx, ty, text=label,
                                         fill=THEME["text_muted"],
                                         font=(FONT_FAMILY, 9, "bold"))
        self.canvas_rom.create_oval(146, 146, 154, 154,
                                     fill=THEME["text_primary"], outline="")

    # ===================================================================
    #  7h.  TAB  ·  ANTI-COMPENSATION
    # ===================================================================

    def _build_tab_compensation(self, tab):
        """Build the Anti-Compensation tab: monitoring controls, the live status and
        score panels, the session stats, the per-reach chart and the sensitivity slider."""
        # The chart at the bottom needs a generous fixed height to stay
        # readable, so we wrap everything in a scrollable frame instead
        # of fighting for vertical space with the controls above it.
        scroll = ctk.CTkScrollableFrame(
            tab, fg_color="transparent",
            scrollbar_button_color=THEME["border"],
            scrollbar_button_hover_color=THEME["text_muted"])
        scroll.pack(fill="both", expand=True, padx=8, pady=8)
        tab = scroll   # everything below packs into the scrollable frame

        make_section_title(
            tab, "Anti-Compensation Monitor",
            "Detects in real time whether the elbow is being used. "
            "Bending the elbow = GOOD. Shoulder up without elbow use = COMPENSATION. "
            "Shoulder going down (return to rest) = GOOD.")

        # ── Top row: buttons + session time ──
        top = ctk.CTkFrame(tab, fg_color="transparent")
        top.pack(fill="x", padx=8, pady=(0, 12))

        self.btn_start_comp = ctk.CTkButton(
            top, text="▶  START  MONITORING",
            width=200, height=42, font=FONTS["body_bold"],
            fg_color=THEME["success"], hover_color=THEME["success_hover"],
            text_color="#0e1117",
            command=self._start_compensation)
        self.btn_start_comp.pack(side="left", padx=2)
        Tooltip(self.btn_start_comp,
                "Start a monitoring session. The current upper-arm angle is "
                "stored as the session baseline. The monitor then runs "
                "continuously and updates the score from the last 1.5 s "
                "of motion.")

        self.btn_stop_comp = ctk.CTkButton(
            top, text="⏹  STOP",
            width=110, height=42, font=FONTS["body_bold"],
            fg_color=THEME["danger"], hover_color=THEME["danger_hover"],
            command=self._stop_compensation)
        self.btn_stop_comp.pack(side="left", padx=2)
        self.btn_stop_comp.configure(state="disabled")

        # (TARE button removed here — use the central tare buttons in the
        #  main header. They automatically reset everything needed here.)

        btn_csv = ctk.CTkButton(
            top, text="⤓  Export Report",
            width=170, height=42, font=FONTS["body_bold"],
            fg_color=THEME["bg_elevated"],
            hover_color=THEME["bg_card_alt"],
            border_width=1, border_color=THEME["border"],
            command=self._export_compensation_report)
        btn_csv.pack(side="left", padx=2)
        Tooltip(btn_csv,
                "Save a visual session report as a PNG image: per-reach "
                "compensation scores, score timeline, shoulder elevation "
                "and elbow flexion. Suitable for sharing with a clinician.")

        self.lbl_baseline_comp = ctk.CTkLabel(
            top, text="Last reach:  --",
            font=FONTS["caption"], text_color=THEME["text_secondary"])
        self.lbl_baseline_comp.pack(side="left", padx=(20, 0))
        Tooltip(self.lbl_baseline_comp,
                "Score of the most recently completed reach.")

        sw = ctk.CTkFrame(top, fg_color="transparent"); sw.pack(side="right")
        ctk.CTkLabel(sw, text="SESSION", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(anchor="e")
        self.lbl_comp_session = ctk.CTkLabel(sw, text="00:00",
                                              font=FONTS["h2"],
                                              text_color=THEME["accent"])
        self.lbl_comp_session.pack(anchor="e")

        # ── Status + Score card (two panels) ──
        row = ctk.CTkFrame(tab, fg_color="transparent")
        row.pack(fill="x", padx=8, pady=(0, 12))

        status_card = make_card(row)
        status_card.pack(side="left", fill="both", expand=True, padx=(0, 6))
        ctk.CTkLabel(status_card, text="STATUS", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(
            anchor="w", padx=18, pady=(14, 6))
        self.comp_status_box = ctk.CTkFrame(
            status_card, fg_color=THEME["bg_card_alt"],
            corner_radius=10, height=120,
            border_width=2, border_color=THEME["border_subtle"])
        self.comp_status_box.pack(fill="x", padx=18, pady=(0, 18))
        self.comp_status_box.pack_propagate(False)
        self.lbl_comp_status = ctk.CTkLabel(
            self.comp_status_box, text="INACTIVE",
            font=(FONT_FAMILY, 28, "bold"),
            text_color=THEME["text_muted"])
        self.lbl_comp_status.pack(expand=True)

        score_card = make_card(row)
        score_card.pack(side="left", fill="both", expand=True, padx=(6, 0))
        ctk.CTkLabel(score_card, text="COMPENSATION  SCORE",
                     font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(
            anchor="w", padx=18, pady=(14, 4))
        self.lbl_comp_score = ctk.CTkLabel(
            score_card, text="0%", font=FONTS["display"],
            text_color=THEME["accent"])
        self.lbl_comp_score.pack(anchor="w", padx=18)
        Tooltip(self.lbl_comp_score,
                "Live compensation score, updated continuously from the "
                "last 1.5 s of motion. Peak values are held briefly and "
                "then decay, so short events stay visible. "
                "0 = good, 25–40 = warning, 40+ = compensation. "
                "Scaled by the Sensitivity slider.")

        self.canvas_comp_bar = tk.Canvas(
            score_card, width=300, height=20,
            bg=THEME["bg_card_alt"], highlightthickness=1,
            highlightbackground=THEME["border_subtle"])
        self.canvas_comp_bar.pack(fill="x", padx=18, pady=(8, 14))

        detail = ctk.CTkFrame(score_card, fg_color="transparent")
        detail.pack(fill="x", padx=18, pady=(0, 14))
        self.lbl_comp_drift    = self._mini_metric(
            detail, "SHOULDER ↑", tooltip=
            "How far the upper arm is currently raised. Combines the "
            "rise above the session baseline with the rise inside the "
            "last 1.5 s, and takes the bigger of the two. A high value "
            "without matching elbow use triggers the alarm.")
        self.lbl_comp_sway     = self._mini_metric(
            detail, "ELBOW Δ", color=THEME["chart_assist"], tooltip=
            "Recent elbow motion in the last 1.5 s (range between min "
            "and max motor angle). Older elbow motion no longer counts. "
            "< 3° = wobble (full alarm), proportional credit up to the "
            "zone-expected amount.")

        # ── Session statistics (compact: reaches + compensations only) ──
        stats = make_card(tab); stats.pack(fill="x", padx=8, pady=(0, 12))
        ctk.CTkLabel(stats, text="SESSION", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(
            anchor="w", padx=18, pady=(14, 4))
        sr = ctk.CTkFrame(stats, fg_color="transparent")
        sr.pack(fill="x", padx=18, pady=(0, 14))

        self.lbl_comp_pct_good = self._stat(sr, "TOTAL REACHES",
            THEME["success"], tooltip=
            "Total completed reaches in this session. A reach is any period "
            "of active motion (motor or upper arm > 8 °/s).")
        # (lbl_comp_pct_warn kept for compatibility, but unused)
        self.lbl_comp_pct_warn = ctk.CTkLabel(sr, text="", width=0,
                                              fg_color="transparent")
        self.lbl_comp_pct_alm  = self._stat(sr, "GOOD REACHES",
            THEME["accent"], tooltip=
            "Reaches where the elbow contributed during motion, or where "
            "the upper arm net descended. Higher = better.")
        blk = ctk.CTkFrame(sr, fg_color="transparent")
        blk.pack(side="left", fill="x", expand=True)
        head_ev = ctk.CTkLabel(blk, text="COMPENSATIONS  ⓘ", font=FONTS["caption"],
                                text_color=THEME["text_muted"])
        head_ev.pack(anchor="w")
        self.lbl_comp_count = ctk.CTkLabel(blk, text="0", font=FONTS["h2"],
                                            text_color=THEME["danger"])
        self.lbl_comp_count.pack(anchor="w")
        Tooltip(head_ev,
                "Number of reaches that ended as COMPENSATION (score above "
                "the alarm threshold).")
        Tooltip(self.lbl_comp_count,
                "Number of compensation reaches in this session.")

        # ── Real-time chart — fixed generous height so the bars are readable ──
        chart_card = make_card(tab)
        chart_card.pack(fill="x", padx=8, pady=(0, 12))
        self.fig_comp = Figure(figsize=(10, 5), dpi=100, constrained_layout=True)
        self.fig_comp.patch.set_facecolor(THEME["bg_card"])
        self.ax_comp = self.fig_comp.add_subplot(111)
        style_axes(self.ax_comp)
        self.canvas_comp_chart = FigureCanvasTkAgg(self.fig_comp, master=chart_card)
        comp_chart_widget = self.canvas_comp_chart.get_tk_widget()
        comp_chart_widget.configure(height=500)
        comp_chart_widget.pack(fill="x", padx=8, pady=8)

        # ── Sensitivity: one master slider scaling both thresholds ──
        d = make_card(tab); d.pack(fill="x", padx=8, pady=(0, 8))
        ctk.CTkLabel(d, text="SENSITIVITY", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(
            anchor="w", padx=18, pady=(14, 4))

        self.lbl_sensitivity = ctk.CTkLabel(
            d, text=f"Sensitivity     5  ·  warning 25% · alarm 40%",
            font=FONTS["body"], text_color=THEME["text_primary"], anchor="w")
        self.lbl_sensitivity.pack(fill="x", padx=18, pady=(4, 2))
        self.sl_sensitivity = ctk.CTkSlider(
            d, from_=1, to=10, number_of_steps=18,
            command=self._set_sensitivity,
            progress_color=THEME["accent"],
            button_color=THEME["accent"],
            button_hover_color=THEME["accent_hover"])
        self.sl_sensitivity.set(5)   # middle = defaults (25% / 40%)
        self.sl_sensitivity.pack(fill="x", padx=18, pady=(0, 8))

        info = ctk.CTkLabel(
            d, anchor="w", font=FONTS["caption"],
            text_color=THEME["text_secondary"],
            text="← less sensitive (high thresholds)         "
                 "more sensitive (low thresholds) →")
        info.pack(fill="x", padx=18, pady=(0, 14))
        Tooltip(self.lbl_sensitivity,
                "One slider controls both thresholds together. Higher "
                "sensitivity flags lighter upper-arm contributions as "
                "compensation; lower sensitivity requires the score to "
                "be genuinely high for an alarm.")
        Tooltip(self.sl_sensitivity,
                "Position 5 (default): warning 25 %, alarm 40 %. "
                "Position 1: ~35 % / 55 % (tolerant). "
                "Position 10: ~15 % / 25 % (very alert).")

        # Compensation pulse state for blinking status box
        self._comp_pulse = 0

    def _mini_metric(self, parent, title: str,
                     color: Optional[str] = None,
                     tooltip: Optional[str] = None) -> ctk.CTkLabel:
        """Make one small live metric (title + value) for the compensation panel."""
        if color is None:
            color = THEME["text_primary"]
        col = ctk.CTkFrame(parent, fg_color="transparent")
        col.pack(side="left", fill="x", expand=True)
        # "?"-suffix in the title when a tooltip exists, so the user sees
        # that hovering is possible.
        title_disp = f"{title}  ⓘ" if tooltip else title
        head = ctk.CTkLabel(col, text=title_disp, font=FONTS["small"],
                            text_color=THEME["text_muted"])
        head.pack(anchor="w")
        lbl = ctk.CTkLabel(col, text="--°", font=FONTS["h3"], text_color=color)
        lbl.pack(anchor="w")
        if tooltip:
            Tooltip(head, tooltip)
            Tooltip(lbl,  tooltip)
        return lbl

    def _stat(self, parent, title: str, color: str,
              tooltip: Optional[str] = None) -> ctk.CTkLabel:
        """Make one big session-statistic block (title + value) for the stats row."""
        blk = ctk.CTkFrame(parent, fg_color="transparent")
        blk.pack(side="left", fill="x", expand=True)
        title_disp = f"{title}  ⓘ" if tooltip else title
        head = ctk.CTkLabel(blk, text=title_disp, font=FONTS["caption"],
                            text_color=THEME["text_muted"])
        head.pack(anchor="w")
        lbl = ctk.CTkLabel(blk, text="--%", font=FONTS["h2"], text_color=color)
        lbl.pack(anchor="w")
        if tooltip:
            Tooltip(head, tooltip)
            Tooltip(lbl,  tooltip)
        return lbl

    # ===================================================================
    #  7j.  TAB  ·  GAME MODE
    # ===================================================================

    def _build_tab_game(self, tab):
        """Build the Game Mode tab: the game selector and the play canvas."""
        make_section_title(tab, "Therapy Challenge",
                           "Play a game to make rehabilitation more engaging "
                           "and goal-directed.")

        self.game_selector = ctk.CTkSegmentedButton(
            tab, values=["Catch Blocks", "Ghost Arm (Rhythm)"],
            command=self._switch_game_mode,
            font=FONTS["body_bold"],
            selected_color=THEME["accent"],
            selected_hover_color=THEME["accent_hover"])
        self.game_selector.set("Catch Blocks")
        self.game_selector.pack(pady=(0, 10), padx=40, fill="x")

        self.frame_ghost_settings = ctk.CTkFrame(tab, fg_color="transparent")
        self.lbl_ghost_speed = ctk.CTkLabel(
            self.frame_ghost_settings,
            text="Speed  (reps per minute)     15",
            font=FONTS["body"], text_color=THEME["text_primary"])
        self.lbl_ghost_speed.pack(pady=(4, 2))
        self.slider_ghost_speed = ctk.CTkSlider(
            self.frame_ghost_settings, from_=5, to=30, number_of_steps=25,
            command=lambda v: self.lbl_ghost_speed.configure(
                text=f"Speed  (reps per minute)     {int(v)}"),
            progress_color=THEME["accent"],
            button_color=THEME["accent"],
            button_hover_color=THEME["accent_hover"])
        self.slider_ghost_speed.set(15)
        self.slider_ghost_speed.pack(fill="x", padx=20, pady=(0, 6))

        sc = make_card(tab); sc.pack(fill="x", padx=40, pady=10, ipady=6)
        row = ctk.CTkFrame(sc, fg_color="transparent"); row.pack(fill="x", padx=18, pady=14)
        col = ctk.CTkFrame(row, fg_color="transparent"); col.pack(side="left")
        ctk.CTkLabel(col, text="SCORE", font=FONTS["small"],
                     text_color=THEME["text_muted"]).pack(anchor="w")
        self.lbl_game_score = ctk.CTkLabel(col, text="0", font=FONTS["display"],
                                            text_color=THEME["accent"])
        self.lbl_game_score.pack(anchor="w")

        ctk.CTkButton(row, text="Reset  Score",
                      command=self._reset_game,
                      fg_color=THEME["bg_elevated"],
                      hover_color=THEME["bg_card_alt"],
                      border_width=1, border_color=THEME["border"],
                      text_color=THEME["text_primary"],
                      width=130, height=36, font=FONTS["body_bold"]
                      ).pack(side="right")

        self.canvas_game = tk.Canvas(tab, width=600, height=320,
                                      bg=THEME["bg_canvas"],
                                      highlightthickness=1,
                                      highlightbackground=THEME["border"])
        self.canvas_game.pack(pady=12)
        self._draw_game_grid()

    # ===================================================================
    #  7k.  TAB  ·  ODRIVE TUNING  (PID + input shaping)
    # ===================================================================

    def _build_tab_pid(self, tab):
        """
        ODrive tuning tab. Sliders send `w axis0.controller.config.<x>`
        commands via the Arduino to the ODrive. Changes are runtime-only —
        a power-cycle restores the values you stored on the ODrive (we do
        NOT call save_configuration).
        """
        scroll = ctk.CTkScrollableFrame(
            tab, fg_color="transparent",
            scrollbar_button_color=THEME["border"],
            scrollbar_button_hover_color=THEME["text_muted"])
        scroll.pack(fill="both", expand=True, padx=8, pady=8)

        make_section_title(
            scroll, "ODrive  Motor Tuning",
            "Adjust the PID gains of the ODrive at runtime. "
            "Power-cycle the ODrive to restore your saved values.")

        # General warning banner
        info_card = make_card(scroll)
        info_card.pack(fill="x", padx=8, pady=(0, 14))
        ctk.CTkLabel(
            info_card, font=FONTS["caption"], anchor="w", justify="left",
            wraplength=900, text_color=THEME["text_secondary"],
            text=(
                "⚠  Move the sliders in small steps and keep an eye on the "
                "motor — extreme values can cause oscillation or "
                "uncontrolled motion. When in doubt: hit STOP and power-"
                "cycle. The startup values on the ODrive are untouched — "
                "these sliders only send temporary changes until the next "
                "power-cycle.")
        ).pack(fill="x", padx=18, pady=14)

        # ---- Position control ----
        pos_card = self._settings_card(scroll, "Position Control  (transitions)")
        ctk.CTkLabel(
            pos_card, font=FONTS["caption"], anchor="w", justify="left",
            wraplength=900, text_color=THEME["text_secondary"],
            text=(
                "Sets how firmly the ODrive holds a target position — active "
                "during 'Go to Angle' and the calibration moves. Has no "
                "effect during therapy (which runs in torque mode).")
        ).pack(fill="x", padx=14, pady=(2, 8))
        self.sl_pid_pos = self._pid_slider(
            pos_card, "pos_gain", 1.0, 80.0, 20.0,
            "SET_POS_GAIN",
            "Higher → tighter target tracking, faster response. "
            "Too high → oscillation or overshoot. Typical 10–40.")

        # ---- Velocity control ----
        vel_card = self._settings_card(scroll, "Velocity Control")
        ctk.CTkLabel(
            vel_card, font=FONTS["caption"], anchor="w", justify="left",
            wraplength=900, text_color=THEME["text_secondary"],
            text=(
                "Sets how the ODrive internally regulates motor velocity. "
                "Affects position mode and indirectly torque mode (via "
                "the current loop).")
        ).pack(fill="x", padx=14, pady=(2, 8))
        self.sl_pid_velg = self._pid_slider(
            vel_card, "vel_gain", 0.01, 1.50, 0.16,
            "SET_VEL_GAIN",
            "Higher → faster response to velocity errors. "
            "Too high → motor starts buzzing or vibrating. Typical 0.05–0.30.")
        self.sl_pid_veli = self._pid_slider(
            vel_card, "vel_integrator_gain", 0.0, 3.0, 0.33,
            "SET_VEL_INT",
            "Compensates for sustained errors (such as gravity). "
            "Higher → more 'stubborn' against disturbances. "
            "Too high → overshoot or swinging. Typical 0.1–0.6.")

        # ---- Input shaping ----
        bw_card = self._settings_card(scroll, "Input Shaping")
        ctk.CTkLabel(
            bw_card, font=FONTS["caption"], anchor="w", justify="left",
            wraplength=900, text_color=THEME["text_secondary"],
            text=(
                "Filters the command signal to the motor. Lower = softer, "
                "smoother motion. Higher = sharper and faster, but also "
                "audibly/physically more abrupt.")
        ).pack(fill="x", padx=14, pady=(2, 8))
        self.sl_pid_bw = self._pid_slider(
            bw_card, "input_filter_bandwidth (Hz)", 0.5, 20.0, 2.0,
            "SET_BW",
            "Bandwidth of the input filter in Hz. Around 2 Hz you get "
            "smooth, soft transitions; at 10+ Hz the motor responds "
            "almost immediately (may feel sharp or vibrate).")

        # ---- "Reset all" shortcut ----
        apply_row = ctk.CTkFrame(scroll, fg_color="transparent")
        apply_row.pack(fill="x", padx=8, pady=(12, 24))
        ctk.CTkButton(
            apply_row, text="↻  Reset all sliders to defaults",
            fg_color=THEME["bg_elevated"],
            hover_color=THEME["bg_card_alt"],
            border_width=1, border_color=THEME["border"],
            text_color=THEME["text_primary"],
            height=36, font=FONTS["body_bold"],
            command=self._reset_pid_sliders
        ).pack(side="right")

    def _pid_slider(self, parent, label: str,
                    mn: float, mx: float, start: float,
                    arduino_cmd: str, tooltip_text: str):
        """Build a PID slider with a live-updating label that also sends
        the matching `arduino_cmd` to the ODrive on every change."""
        # Decide precision
        prec = 2 if mx - mn <= 2.0 else (3 if mx <= 5.0 else 2)

        lbl = ctk.CTkLabel(
            parent, anchor="w", font=FONTS["body"],
            text_color=THEME["text_primary"],
            text=f"{label}     {start:.{prec}f}")
        lbl.pack(fill="x", padx=14, pady=(8, 2))

        def on_change(v, _lbl=lbl, _cmd=arduino_cmd, _label=label, _p=prec):
            _lbl.configure(text=f"{_label}     {float(v):.{_p}f}")
            self.serial.send(f"{_cmd}:{float(v):.{_p}f}")

        sl = ctk.CTkSlider(parent, from_=mn, to=mx, command=on_change,
                           progress_color=THEME["accent"],
                           button_color=THEME["accent"],
                           button_hover_color=THEME["accent_hover"])
        sl.set(start)
        sl.pack(fill="x", padx=14, pady=(0, 6))

        # Remember default for the reset button
        sl._pid_default = start
        sl._pid_label   = lbl
        sl._pid_name    = label
        sl._pid_prec    = prec
        sl._pid_cmd     = arduino_cmd

        info = ctk.CTkLabel(parent, anchor="w", font=FONTS["caption"],
                            text_color=THEME["text_secondary"],
                            wraplength=900, justify="left",
                            text=tooltip_text)
        info.pack(fill="x", padx=14, pady=(0, 12))

        Tooltip(lbl, tooltip_text)
        Tooltip(sl,  tooltip_text)
        return sl

    def _reset_pid_sliders(self):
        """Restore every PID slider to its visual default and send the
        default to the ODrive."""
        for sl in (self.sl_pid_pos, self.sl_pid_velg,
                   self.sl_pid_veli, self.sl_pid_bw):
            default = sl._pid_default
            sl.set(default)
            sl._pid_label.configure(
                text=f"{sl._pid_name}     {default:.{sl._pid_prec}f}")
            self.serial.send(f"{sl._pid_cmd}:{default:.{sl._pid_prec}f}")

    # ===================================================================
    #  7l.  TAB  ·  LIVE GRAPHS
    # ===================================================================

    def _build_tab_graphs(self, tab):
        """Build the Live Graphs tab: a 2x2 grid of time plots filled by _loop_graphs."""
        # The four live graphs (position, velocity, acceleration, torque).
        fig = Figure(figsize=(6, 4), dpi=100)
        fig.patch.set_facecolor(THEME["bg_card"])
        self.ax1 = fig.add_subplot(221)
        self.ax2 = fig.add_subplot(222)
        self.ax3 = fig.add_subplot(223)
        self.ax4 = fig.add_subplot(224)
        for ax in (self.ax1, self.ax2, self.ax3, self.ax4):
            style_axes(ax)
        fig.tight_layout()
        self.fig_graphs = fig
        self.graph_canvas = FigureCanvasTkAgg(fig, master=tab)
        self.graph_canvas.get_tk_widget().pack(fill="both", expand=True,
                                                padx=8, pady=8)

    # ===================================================================
    #  8.  TELEMETRY HANDLER
    # ===================================================================

    def _on_serial_line(self, line: str):
        """Handle one telemetry line from the Arduino. Each line is "KEY:value"
        (POS/VEL/TRQ/STATE/IMU1/IMU2); we update the matching state, buffers and
        labels. IMU1 is sign-flipped and both IMUs have their tare offset removed."""
        if not line:
            return
        try:
            head, _, val = line.partition(":")
            head = head.upper()
            if head == "POS":
                angle = float(val); self.current_motor_angle = angle
                self._update_arm_visual(angle)
                self._check_rom_test(angle)
                self._update_game_visual(angle)
                self.buf.push_pos(angle, self.current_target,
                                  time.time() - self.start_time)
                self.sock_visual.sendto(
                    f"{self.current_imu1:.2f},{angle:.2f}".encode(),
                    (UDP_IP, UDP_PORT))
                self.lbl_kpi_target.configure(text=f"{self.current_target:.1f}°")
            elif head == "VEL":
                v = float(val)
                self.last_motor_vel = v
                self.buf.push_vel(v)
                self.smooth.push(v)
            elif head == "TRQ":
                t = float(val); self.buf.push_trq(t)
                self.lbl_kpi_trq.configure(text=f"{t:+.2f} Nm")
            elif head == "STATE":
                self._set_state_code(val.strip())
            elif head == "IMU1":
                self.raw_imu1 = -float(val)
                self.current_imu1 = self.raw_imu1 - self.imu1_offset
                self.buf.push_imu1(self.current_imu1)
                self.lbl_imu1.configure(
                    text=f"Upper Arm   {self.current_imu1:6.1f}°")
            elif head == "IMU2":
                self.raw_imu2 = float(val)
                self.current_imu2 = self.raw_imu2 - self.imu2_offset
                self.buf.push_imu2(self.current_imu2)
                self.lbl_imu2.configure(
                    text=f"Lower Arm   {self.current_imu2:6.1f}°")
        except ValueError:
            pass  # Unparseable line — just skip

    def _set_state_code(self, code: str):
        """Translate the firmware's STATE number (0..5) into a status label + colour."""
        mapping = {
            "0": ("Waiting for Index...",   THEME["warning"]),
            "1": ("Ready to calibrate",     THEME["warning"]),
            "2": ("Moving to 0°...",        THEME["warning"]),
            "3": ("READY",                  THEME["success"]),
            "4": ("THERAPY ACTIVE",         THEME["accent"]),
            "5": ("Adjusting settings...",  THEME["warning"]),
        }
        if code in mapping:
            text, color = mapping[code]
            self._set_status(text, color)

    def _set_status(self, text: str, color: str):
        """Update the sidebar status text and the coloured status dot."""
        self.lbl_status.configure(text=text, text_color=color)
        self.status_dot.configure(text_color=color)

    # ===================================================================
    #  9.  LOOPS  (all tk.after schedulers)
    # ===================================================================

    def _loop_serial(self):
        """Drain the serial buffer every 30 ms and feed each line to the handler."""
        self.serial.pump(self._on_serial_line)
        self.after(30, self._loop_serial)

    def _loop_check_connection(self):
        """Once per second: show OFFLINE if no telemetry arrived, and update the timer."""
        if not self.serial.alive:
            self._set_status("OFFLINE / NO POWER", THEME["danger"])
        elapsed = int(time.time() - self.start_time)
        self.lbl_uptime.configure(text=f"{elapsed // 60:02d}:{elapsed % 60:02d}")
        self.after(1000, self._loop_check_connection)

    def _loop_graphs(self):
        """Redraw the four live time-plots every 500 ms from the telemetry buffers."""
        try:
            min_len = min(len(self.buf.t),  len(self.buf.pos),
                          len(self.buf.vel), len(self.buf.acc),
                          len(self.buf.trq))
            if min_len > 2:
                for ax in (self.ax1, self.ax2, self.ax3, self.ax4):
                    ax.cla(); style_axes(ax)
                t = list(self.buf.t)[-min_len:]
                p = list(self.buf.pos)[-min_len:]
                v = list(self.buf.vel)[-min_len:]
                a = list(self.buf.acc)[-min_len:]
                tq = list(self.buf.trq)[-min_len:]

                for ax, ys, title, col in [
                    (self.ax1, p,  "Position  (°)",   THEME["chart_pos"]),
                    (self.ax2, v,  "Velocity",        THEME["chart_vel"]),
                    (self.ax3, a,  "Acceleration",    THEME["chart_acc"]),
                    (self.ax4, tq, "Torque  (Current)", THEME["chart_trq"]),
                ]:
                    ax.plot(t, ys, color=col, linewidth=2)
                    ax.fill_between(t, ys, color=col, alpha=0.15)
                    ax.set_title(title, color=THEME["text_primary"],
                                 fontsize=11, fontweight="bold", loc="left")
                self.fig_graphs.tight_layout()
                self.graph_canvas.draw_idle()
        except Exception:
            pass
        self.after(500, self._loop_graphs)

    def _loop_3d(self):
        """Redraw the 3D arm every 50 ms. Forward kinematics from the upper-arm IMU
        and the motor angle give the shoulder, elbow and wrist points; the mirror
        arm (if on) is the same pose flipped across the body's centre line."""
        try:
            rad1 = math.radians(self.current_imu1)
            rad2 = math.radians(self.current_motor_angle)
            L1, L2 = 28.0, 24.0

            x0, y0, z0 = 20, 0, 0
            x1 = 22
            y1 = y0 + L1 * math.sin(rad1)
            z1 = z0 - L1 * math.cos(rad1)
            tot = rad1 + rad2
            x2 = 22
            y2 = y1 + L2 * math.sin(tot)
            z2 = z1 - L2 * math.cos(tot)

            self.arm_line_3d.set_data([x0, x1, x2], [y0, y1, y2])
            self.arm_line_3d.set_3d_properties([z0, z1, z2])
            self.shoulder_3d.set_data([x0], [y0])
            self.shoulder_3d.set_3d_properties([z0])
            self.elbow_3d.set_data([x1], [y1])
            self.elbow_3d.set_3d_properties([z1])
            self.wrist_3d.set_data([x2], [y2])
            self.wrist_3d.set_3d_properties([z2])

            # Mirror arm: mirrored over the X=0 sagittal line (left/right).
            # Y and Z (up/down and front/back) stay the same, so both arms
            # show the same elbow flexion but mirrored.
            if self.mirror_active:
                self.mirror_line_3d.set_data([-x0, -x1, -x2], [y0, y1, y2])
                self.mirror_line_3d.set_3d_properties([z0, z1, z2])
                self.mirror_shoulder_3d.set_data([-x0], [y0])
                self.mirror_shoulder_3d.set_3d_properties([z0])
                self.mirror_elbow_3d.set_data([-x1], [y1])
                self.mirror_elbow_3d.set_3d_properties([z1])
                self.mirror_wrist_3d.set_data([-x2], [y2])
                self.mirror_wrist_3d.set_3d_properties([z2])

            self.canvas_3d.draw_idle()
        except Exception:
            pass
        self.after(50, self._loop_3d)

    def _toggle_mirror(self):
        """Mirror Therapy on/off. Shows a mirrored arm on the other side
        of the body for post-stroke neuroplasticity training."""
        self.mirror_active = bool(self.switch_mirror.get())
        for obj in (self.mirror_line_3d, self.mirror_shoulder_3d,
                    self.mirror_elbow_3d, self.mirror_wrist_3d):
            obj.set_visible(self.mirror_active)
        try:
            self.canvas_3d.draw_idle()
        except Exception:
            pass

    def _loop_compensation(self):
        """Every 150 ms: feed the latest pose to the detector and redraw its UI + chart."""
        try:
            # Tick the detector
            self.comp.tick(self.current_imu1,
                           self.current_imu2,
                           self.current_motor_angle,
                           self.last_motor_vel)

            # Always render — even in inactive state, so we see the empty
            # chart with threshold lines instead of a blank panel until
            # something happens.
            self._render_compensation_ui()
            self._render_compensation_chart()
        except Exception:
            pass
        self.after(150, self._loop_compensation)

    # ===================================================================
    #  10.  HEADER / TOP-CARD UPDATES
    # ===================================================================

    def _update_arm_visual(self, degrees: float):
        """Update the header's big angle read-out and the little 2D elbow sketch."""
        self.lbl_degrees.configure(text=f"{degrees:.1f}°")
        c = self.canvas_arm
        c.delete("dyn")
        ex, ey, L = 150, 100, 100
        rad = math.radians(degrees)
        px = ex + L * math.cos(rad); py = ey - L * math.sin(rad)
        c.create_arc(ex - L, ey - L, ex + L, ey + L,
                     start=0, extent=100, style="arc",
                     outline=THEME["border"], width=1, tags="dyn")
        c.create_line(ex, ey, px, py, width=10, fill=THEME["accent"],
                      tags="dyn", capstyle=tk.ROUND)
        c.create_oval(px - 7, py - 7, px + 7, py + 7,
                      fill=THEME["accent"], outline="", tags="dyn")
        c.create_oval(ex - 8, ey - 8, ex + 8, ey + 8,
                      fill=THEME["text_primary"], outline="", tags="dyn")

    # ===================================================================
    #  11.  CALIBRATION / SETTINGS / ANGLE
    # ===================================================================

    def _tare_pose(self, target_upper: float, target_lower: float):
        """Central tare. Single source of truth for IMU orientation.

        Every subsystem (compensation monitor, 3D arm, motion visualisation)
        simply reads `self.current_imu1` and `self.current_imu2`. Setting
        the offsets here propagates automatically.

        Effects:
          - Upper / lower arm offsets adjusted
          - Compensation monitor (if active) gets a new baseline
          - Visual confirmation in the header
        """
        self.imu1_offset = self.raw_imu1 - target_upper
        self.imu2_offset = self.raw_imu2 - target_lower

        # Compensation monitor: IMU frame has changed, reset its state
        if hasattr(self, "comp") and self.comp.active:
            self.comp.reset_baseline(self.current_imu1,
                                      self.current_imu2,
                                      self.current_motor_angle)

        # Visual feedback (disappears after 2 s)
        pose = "vertical" if target_upper == 0 else (
               "horizontal" if target_upper == 90 else f"{target_upper:.0f}°")
        self.lbl_tare_feedback.configure(
            text=f"✓  Calibrated: {pose}")
        self.after(2000, lambda: self.lbl_tare_feedback.configure(text=""))
        print(f"[Tare] upper_arm={target_upper}°, lower_arm={target_lower}°")

    def _switch_mode(self, choice: str):
        """Send the matching MODE_ command when the therapist flips the mode switch."""
        if   choice == "Assistive (Help)":   self.serial.send("MODE_ASSISTIVE")
        elif choice == "Resistive (Train)":  self.serial.send("MODE_RESISTIVE")

    def _activate_settings(self):
        """Enter settings mode: tell the firmware, then unlock the sliders for editing."""
        self.serial.send("CHANGE_SETTINGS")
        for sl in self.sliders: sl.configure(state="normal")
        self.btn_save.configure(state="normal")
        self.btn_start.configure(state="disabled")

    def _save_settings(self):
        """Push every slider value to the firmware as a SET_ command, then lock again."""
        s = self.serial
        s.send(f"SET_LIM_MIN:{self.sl_lim_min.get():.1f}")
        s.send(f"SET_LIM_MAX:{self.sl_lim_max.get():.1f}")
        s.send(f"SET_A_TORQUE:{self.sl_a_torque.get():.2f}")
        s.send(f"SET_A_FACTOR:{self.sl_a_factor.get():.2f}")
        s.send(f"SET_A_DAMP:{self.sl_a_damp.get():.2f}")
        s.send(f"SET_T_DAMP:{self.sl_t_resist.get():.2f}")
        s.send(f"SET_SPRING:{self.sl_v_spring.get():.2f}")
        s.send(f"SET_MAX_TQ:{self.sl_v_torque.get():.2f}")
        s.send(f"SET_HAPTIC:{self.sl_v_haptic.get():.2f}")
        s.send(f"SET_SHOCK:{self.sl_v_shock.get():.2f}")
        s.send("SAVE_DONE")
        for sl in self.sliders: sl.configure(state="disabled")
        self.btn_save.configure(state="disabled")
        self.btn_start.configure(state="normal")

    def _check_lim_min(self, _value):
        """Keep the min-limit slider at least 5° below the max, and update its label."""
        if self.sl_lim_min.get() > self.sl_lim_max.get() - 5:
            self.sl_lim_min.set(self.sl_lim_max.get() - 5)
        self.lbl_lim_min.configure(
            text=f"Minimum Angle (Extension)     {self.sl_lim_min.get():.2f}")

    def _check_lim_max(self, _value):
        """Keep the max-limit slider at least 5° above the min, and update its label."""
        if self.sl_lim_max.get() < self.sl_lim_min.get() + 5:
            self.sl_lim_max.set(self.sl_lim_min.get() + 5)
        self.lbl_lim_max.configure(
            text=f"Maximum Angle (Flexion)     {self.sl_lim_max.get():.2f}")

    def _set_angle(self, angle: float):
        """Command the motor to a target angle (preset button or slider)."""
        self.current_target = float(angle)
        self.serial.send(f"SET_ANGLE:{angle:.1f}")

    # ===================================================================
    #  12.  ROM TEST
    # ===================================================================

    def _start_rom_test(self):
        """Begin a ROM test: switch to free-ride mode and start tracking min/max angle."""
        self.rom.active = True
        self.rom.rom_min = 100.0
        self.rom.rom_max = 0.0
        self.serial.send("MODE_FREERIDE")
        self.serial.send("start")
        self.lbl_rom_instruction.configure(
            text="Move the arm fully back and forth now.",
            text_color=THEME["accent"])

    def _stop_rom_test(self):
        """End the ROM test, stop the motor and restore the previously selected mode."""
        self.rom.active = False
        self.serial.send("stop")
        self._switch_mode(self.mode_switch.get())
        self.lbl_rom_instruction.configure(
            text="ROM Test completed — see results below.",
            text_color=THEME["success"])

    def _check_rom_test(self, angle: float):
        """During a ROM test, widen the recorded min/max and redraw the range dial."""
        if not self.rom.active:
            return
        if angle < self.rom.rom_min: self.rom.rom_min = angle
        if angle > self.rom.rom_max: self.rom.rom_max = angle
        self.lbl_rom_result.configure(
            text=f"Extension  {self.rom.rom_min:5.1f}°     ◆     "
                 f"Flexion  {self.rom.rom_max:5.1f}°")
        self.canvas_rom.delete("slice")
        self.canvas_rom.create_arc(
            50, 50, 250, 250,
            start=self.rom.rom_min,
            extent=(self.rom.rom_max - self.rom.rom_min),
            fill=THEME["success"], outline="", tags="slice")

    # ===================================================================
    #  13.  COMPENSATION UI
    # ===================================================================

    def _start_compensation(self):
        """Start a monitoring session: capture the baseline pose and arm the detector."""
        self.comp.start(self.current_imu1, self.current_imu2,
                        self.current_motor_angle)
        self.btn_start_comp.configure(state="disabled")
        self.btn_stop_comp.configure(state="normal")
        self.lbl_baseline_comp.configure(
            text="No reach yet", text_color=THEME["text_secondary"])

    def _stop_compensation(self):
        """Stop the monitoring session and reset the status box to inactive."""
        self.comp.stop()
        self.btn_start_comp.configure(state="normal")
        self.btn_stop_comp.configure(state="disabled")
        self._set_compensation_box("INACTIVE")

    def _export_compensation_report(self):
        """Ask for a file name and save the session report PNG (handles errors)."""
        if not self.comp.has_log:
            messagebox.showinfo("Export", "No data to export yet.")
            return
        default_name = f"compensation_{time.strftime('%Y%m%d_%H%M%S')}.png"
        path = filedialog.asksaveasfilename(
            title="Save session report",
            defaultextension=".png",
            initialfile=default_name,
            filetypes=[("PNG image", "*.png")])
        if not path:
            return
        try:
            self.comp.export_report(Path(path))
            messagebox.showinfo("Export", f"Report saved:\n{path}")
        except Exception as e:
            messagebox.showerror("Export",
                                  f"Could not save report:\n{e}")

    def _set_sensitivity(self, v: float):
        """Master sensitivity slider — scales both compensation thresholds.

        Position 5 (default) → defaults (warn 25%, alarm 40%).
        Position 1 (low)     → about 35% / 55%  (only gross compensation).
        Position 10 (high)   → about 15% / 25%  (very alert).
        """
        sens = float(v)
        # factor: position 5 → 1.0, position 1 → ~1.4, position 10 → ~0.6
        factor = 1.5 - sens * 0.1
        factor = max(0.45, factor)
        self.comp.level_warn  = 25.0 * factor
        self.comp.level_alarm = 40.0 * factor
        self.lbl_sensitivity.configure(
            text=f"Sensitivity     {int(round(sens))}  ·  warning "
                 f"{self.comp.level_warn:.0f}% · alarm "
                 f"{self.comp.level_alarm:.0f}%")

    def _set_compensation_box(self, level: str):
        """Colour and label the big status box for the current compensation level."""
        styles = {
            "INACTIVE":     (THEME["bg_card_alt"], THEME["border_subtle"],
                             "INACTIVE",     THEME["text_muted"]),
            "REST":         (THEME["bg_card_alt"], THEME["accent_dim"],
                             "REST POSITION", THEME["accent"]),
            "GOOD":         (THEME["bg_card_alt"], THEME["success"],
                             "GOOD",         THEME["success"]),
            "WARNING":      (THEME["bg_card_alt"], THEME["warning"],
                             "WATCH OUT",    THEME["warning"]),
            "COMPENSATION": (THEME["bg_card_alt"], THEME["danger"],
                             "COMPENSATION", THEME["danger"]),
        }
        bg, bd, tx, fg = styles.get(level, styles["INACTIVE"])
        self.comp_status_box.configure(fg_color=bg, border_color=bd)
        self.lbl_comp_status.configure(text=tx, text_color=fg)

    def _render_compensation_ui(self):
        """Refresh the compensation panel: score, status box (flashing on alarm),
        progress bar, the SHOULDER/ELBOW mini-metrics and the session counters."""
        comp = self.comp
        lvl  = comp.level

        # Score colour
        score_color = (THEME["danger"]  if comp.score >= comp.level_alarm else
                       THEME["warning"] if comp.score >= comp.level_warn  else
                       THEME["accent"])
        # Score in % — show LIVE indicator if a reach is in progress
        suffix = "  ●" if comp.reach_state == "ACTIVE" else ""
        self.lbl_comp_score.configure(text=f"{comp.score:.0f}%{suffix}",
                                       text_color=score_color)

        # Status box — compensation → flash via pulse
        if lvl == "COMPENSATION":
            self._comp_pulse = (self._comp_pulse + 1) % 2
            if self._comp_pulse:
                self.comp_status_box.configure(fg_color=THEME["danger"],
                                                border_color=THEME["danger"])
                self.lbl_comp_status.configure(text="COMPENSATION",
                                                text_color=THEME["text_primary"])
            else:
                self.comp_status_box.configure(fg_color=THEME["bg_card_alt"],
                                                border_color=THEME["danger"])
                self.lbl_comp_status.configure(text="COMPENSATION",
                                                text_color=THEME["danger"])
        else:
            self._set_compensation_box(lvl)

        # Progress bar
        c = self.canvas_comp_bar
        c.delete("fill")
        max_width = 296
        # Score is in %, so 0..100 nominal; use 60% as full bar width minimum
        max_score = max(comp.level_alarm * 1.5, 60.0)
        perc = clamp(comp.score / max_score, 0.0, 1.0)
        if perc > 0.001:
            c.create_rectangle(2, 2, 2 + max_width * perc, 18,
                               fill=score_color, outline="", tags="fill")

        # Detail metrics — angles in degrees, score in %
        sd = comp.shoulder_delta
        ed = comp.elbow_delta_max
        self.lbl_comp_drift.configure(
            text=f"{sd:+.1f}°",
            text_color=(THEME["danger"]  if sd > 15 else
                        THEME["warning"] if sd > 5  else
                        THEME["success"] if sd < -2 else
                        THEME["text_primary"]))
        self.lbl_comp_sway.configure(
            text=f"{ed:.1f}°",
            text_color=(THEME["success"] if ed >= self.comp.ELBOW_MIN_FLOOR
                        else THEME["text_primary"]))

        # Baseline label reused: last reach + reach-active indicator
        if comp.reach_active:
            text = "● reach in progress..."
            color = THEME["accent"]
        elif comp.reach_count > 0:
            text = f"Last reach: {comp.last_reach_score:.0f}%"
            color = (THEME["danger"]  if comp.last_reach_score >= comp.level_alarm else
                     THEME["warning"] if comp.last_reach_score >= comp.level_warn  else
                     THEME["success"])
        else:
            text = "No reach yet"
            color = THEME["text_secondary"]
        self.lbl_baseline_comp.configure(text=text, text_color=color)

        # Session stats: total reaches + good reaches + compensations
        good = max(0, comp.reach_count - comp.event_count)
        self.lbl_comp_pct_good.configure(text=f"{comp.reach_count}")
        self.lbl_comp_pct_alm.configure(text=f"{good}")
        self.lbl_comp_count.configure(text=f"{comp.event_count}")

        # Session time
        s = int(comp.session_seconds)
        self.lbl_comp_session.configure(text=f"{s // 60:02d}:{s % 60:02d}")

    def _render_compensation_chart(self):
        """Per-reach bar chart — one bar per completed reach.
        Colour = level (green / amber / red), value labelled above each bar."""
        try:
            ax = self.ax_comp
            ax.cla(); style_axes(ax)

            # Y-axis always 0..100
            ax.set_ylim(0, 105)
            ax.set_ylabel("Score  (%)",
                          color=THEME["text_primary"], fontsize=11,
                          fontweight="bold")
            ax.set_xlabel("Reach  →",
                          color=THEME["text_primary"], fontsize=11,
                          fontweight="bold")

            # Background bands (subtle, for orientation)
            ax.axhspan(0, self.comp.level_warn,
                       color=THEME["success"], alpha=0.10)
            ax.axhspan(self.comp.level_warn, self.comp.level_alarm,
                       color=THEME["warning"], alpha=0.12)
            ax.axhspan(self.comp.level_alarm, 105,
                       color=THEME["danger"], alpha=0.12)

            # Threshold lines made prominent
            ax.axhline(y=self.comp.level_warn, color=THEME["warning"],
                       linestyle="--", linewidth=1.2, alpha=0.85,
                       label=f"WARNING  ({self.comp.level_warn:.0f}%)")
            ax.axhline(y=self.comp.level_alarm, color=THEME["danger"],
                       linestyle="--", linewidth=1.2, alpha=0.85,
                       label=f"COMPENSATION  ({self.comp.level_alarm:.0f}%)")

            scores = list(self.comp.history)
            if not scores:
                ax.set_title("Waiting for the first reach...",
                             color=THEME["text_secondary"], fontsize=12,
                             fontweight="bold", loc="left")
                ax.set_xlim(0.5, 5.5)
                ax.set_xticks(range(1, 6))
                ax.legend(loc="upper right", fontsize=10,
                          facecolor=THEME["bg_card"],
                          edgecolor=THEME["border"],
                          labelcolor=THEME["text_primary"])
                # constrained_layout (set on the Figure) handles spacing
                # automatically; no tight_layout() call needed.
                self.canvas_comp_chart.draw_idle()
                return

            x = list(range(1, len(scores) + 1))

            # Colour per bar based on the level
            colors = []
            for s in scores:
                if s >= self.comp.level_alarm:
                    colors.append(THEME["danger"])
                elif s >= self.comp.level_warn:
                    colors.append(THEME["warning"])
                else:
                    colors.append(THEME["success"])

            # Bars — thick contrasting edges
            bars = ax.bar(x, scores, color=colors, width=0.7,
                          edgecolor=THEME["text_primary"], linewidth=1.4,
                          zorder=3)

            # Value labels ABOVE every bar
            for xi, s, col in zip(x, scores, colors):
                label_y = s + 3
                if label_y > 100:
                    label_y = s - 6   # put label inside the bar when too high
                ax.text(xi, label_y, f"{s:.0f}%",
                        ha="center", va="bottom" if label_y > s else "top",
                        color=THEME["text_primary"], fontsize=11,
                        fontweight="bold", zorder=4)

            ax.set_title(f"Reach history  ·  {len(scores)} reach"
                         f"{'es' if len(scores) != 1 else ''}",
                         color=THEME["text_primary"], fontsize=12,
                         fontweight="bold", loc="left")

            # X-axis: always show at least 5 reach slots
            x_max = max(5, len(scores))
            ax.set_xlim(0.5, x_max + 0.5)
            ax.set_xticks(range(1, x_max + 1))
            ax.tick_params(axis='both', labelsize=10)

            ax.legend(loc="upper right", fontsize=10,
                      facecolor=THEME["bg_card"],
                      edgecolor=THEME["border"],
                      labelcolor=THEME["text_primary"])

            # constrained_layout on the Figure handles spacing automatically.
            self.canvas_comp_chart.draw_idle()
        except Exception:
            pass

    # ===================================================================
    #  14.  GAME MODE
    # ===================================================================

    def _switch_game_mode(self, choice: str):
        """Switch the active game, reset its score, and show the Ghost-Arm speed slider
        only when that mode is selected."""
        self.game.mode = choice
        self.game.score = 0
        self.game.ghost_score = 0
        self.lbl_game_score.configure(text="0")
        if choice == "Ghost Arm (Rhythm)":
            self.frame_ghost_settings.pack(pady=(5, 0), fill="x", padx=40)
        else:
            self.frame_ghost_settings.pack_forget()

    def _reset_game(self):
        """Reset the current game's score and target back to the start values."""
        self.game.score = 0
        self.game.ghost_score = 0
        self.game.target_y = 150
        self.lbl_game_score.configure(text="0")

    def _draw_game_grid(self):
        """Draw the background grid of the game canvas."""
        c = self.canvas_game
        c.create_rectangle(0, 0, 600, 320, fill=THEME["bg_canvas"], outline="")
        for x in range(0, 600, 30):
            c.create_line(x, 0, x, 320, fill=THEME["border_subtle"])
        for y in range(0, 320, 30):
            c.create_line(0, y, 600, y, fill=THEME["border_subtle"])

    def _update_game_visual(self, angle: float):
        """Redraw the active game for the current elbow angle and update its score.
        "Catch Blocks" rewards holding the cursor in the target; "Ghost Arm" rewards
        matching a moving reference arm."""
        c = self.canvas_game
        c.delete("all"); self._draw_game_grid()

        if self.game.mode == "Catch Blocks":
            c.create_line(65, 20, 65, 300, fill=THEME["border"], width=1)
            player_y = 280 - (clamp(angle, 0.0, 100.0) / 100.0) * 260
            c.create_oval(50, player_y - 16, 80, player_y + 16,
                          fill=THEME["accent"],
                          outline=THEME["text_primary"], width=2)
            in_zone = abs(player_y - self.game.target_y) < 20
            color = THEME["success"] if self.game.hold_frames > 0 else THEME["warning"]
            c.create_rectangle(200, self.game.target_y - 22,
                                252, self.game.target_y + 22,
                                fill=color, outline="")
            if in_zone:
                self.game.hold_frames += 1
                pb = min(52, (self.game.hold_frames / 20.0) * 52)
                c.create_rectangle(200, self.game.target_y + 26,
                                    200 + pb, self.game.target_y + 32,
                                    fill=THEME["success"], outline="")
                if self.game.hold_frames >= 20:
                    self.game.score += 1
                    self.lbl_game_score.configure(text=f"{self.game.score}")
                    self.game.hold_frames = 0
                    self.game.target_y = random.randint(40, 260)
            else:
                self.game.hold_frames = 0
                c.create_rectangle(200, self.game.target_y + 26,
                                    252, self.game.target_y + 32,
                                    fill=THEME["border"], outline="")

        elif self.game.mode == "Ghost Arm (Rhythm)":
            freq = self.slider_ghost_speed.get() / 60.0
            t = time.time()
            ghost_angle = 50.0 + 30.0 * math.sin(2 * math.pi * freq * t)

            ex, ey, L = 150, 250, 150
            gr = math.radians(ghost_angle)
            gpx, gpy = ex + L * math.cos(gr), ey - L * math.sin(gr)
            c.create_line(ex, ey, gpx, gpy, width=15,
                          fill=THEME["text_muted"], capstyle=tk.ROUND,
                          dash=(8, 8))
            pr = math.radians(angle)
            ppx, ppy = ex + L * math.cos(pr), ey - L * math.sin(pr)
            c.create_line(ex, ey, ppx, ppy, width=8,
                          fill=THEME["accent"], capstyle=tk.ROUND)
            in_sync = abs(angle - ghost_angle) < 10.0
            joint = THEME["success"] if in_sync else THEME["danger"]
            c.create_oval(ex - 14, ey - 14, ex + 14, ey + 14,
                          fill=joint, outline=THEME["text_primary"], width=2)
            if in_sync:
                self.game.ghost_score += 1
            self.lbl_game_score.configure(
                text=f"{self.game.ghost_score // 10}")


# ===========================================================================
#  8.  ENTRY POINT
# ===========================================================================

def main():
    app = RehabApp()
    app.mainloop()


if __name__ == "__main__":
    main()
