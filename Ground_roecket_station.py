#!/usr/bin/env python3
"""
Rocket Telemetry Ground Station  (PyQt6 + pyqtgraph + Folium/Leaflet)
=====================================================================

Reads telemetry from an HC-12 radio (appears as a serial / COM port on the
laptop) and displays it on a dark/light aerospace dashboard.

Install:
    pip install PyQt6 PyQt6-WebEngine pyqtgraph pyserial numpy folium

Run:
    python gui.py                              # pick port in the GUI
    python gui.py --port COM5                  # preselect + auto-connect
    python gui.py --port /dev/ttyUSB0 --baud 9600
    python gui.py --demo                       # simulated flight, no hardware
    python gui.py --light                      # start in sunlight-readable light mode
    python gui.py --csv D:/flights/launch1     # custom log prefix/folder

Build standalone .exe (PyInstaller):
    pip install pyinstaller
    pyinstaller --noconfirm --onefile --windowed --name RocketGroundStation gui.py

Incoming stream = BINARY telemetry packets mixed with optional text lines.
Packet (26 bytes, little-endian):
    0-1    sync word 0xAA55 (on the wire: 55 AA)
    2-5    uint32  uptime, ms
    6      uint8   state
    7-10   float32 total acceleration, m/s^2
    11-14  float32 fused (Kalman) altitude, m
    15     uint8   continuity: tens digit = drogues, ones digit = mains (22 = 2 drogues, 2 mains)
    16-19  int32   latitude  * 10,000,000
    20-23  int32   longitude * 10,000,000
    24     uint8   GPS satellites
    25     uint8   XOR checksum of bytes 2..24
Text lines (e.g. "TRANSITION: STATE_BOOST", command replies) go to the event log.

Everything received is written to a timestamped CSV in ./telemetry_logs/
(or wherever --csv points) the moment you press CONNECT.
"""

import argparse
import csv
import html
import json
import math
import queue
import random
import struct
import subprocess
import sys
import threading
import time
import webbrowser
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import serial
from PyQt6 import QtCore, QtGui, QtWidgets
from serial.tools import list_ports

try:
    import folium
    HAS_FOLIUM = True
except ImportError:
    folium = None
    HAS_FOLIUM = False

try:
    from PyQt6.QtWebEngineCore import QWebEngineSettings
    from PyQt6.QtWebEngineWidgets import QWebEngineView
    HAS_WEBENGINE = True
except ImportError:
    QWebEngineSettings = None
    QWebEngineView = None
    HAS_WEBENGINE = False

# --------------------------------------------------------------------------
# Configuration -- edit these to match your flight-computer firmware
# --------------------------------------------------------------------------
STATE_NAMES = {
    0: "ERROR",
    1: "PAD",
    2: "ARMED",            # <- assumed name; set to whatever your firmware calls state 2
    3: "BOOST",
    4: "COAST",
    5: "APOGEE SEARCH",
    6: "FIRE DROGUE",
    7: "DROGUE DESCENT",
    8: "FIRE MAIN",
    9: "MAIN DESCENT",
}
EJECTION_STATES = {6: "DROGUE", 8: "MAIN"}   # charge firing  -> flashing red
DEPLOYED_STATES = {7: "DROGUE", 9: "MAIN"}   # chute out      -> green

# Pad-locked test command safety limits
TEST_ALLOWED_STATES = {1}      # test commands unlock ONLY while the rocket reports PAD
OVERRIDE_SECONDS = 60          # the lockout override switches itself off after this long

# One-click command buttons (Serial Monitor tab): (button label, text sent, kind)
#   kind: "safe"     = sends immediately
#         "confirm"  = asks Yes/No first
#         "pad_test" = asks Yes/No AND only unlocks while rocket is on the PAD (or override active)
#         "fire"     = fires a charge: asks warning confirmation AND only unlocks while on the PAD
#         "reboot"   = asks first (extra warning if an active flight state is reported)
QUICK_COMMANDS = [
    ("READ LOG", "READ", "safe"),
    ("CLEAR LOG", "CLEAR", "confirm"),
    ("TEST BARO", "test_baro", "pad_test"),
    ("TEST DROGUE", "TEST_DROGUE", "fire"),
    ("TEST MAIN", "TEST_MAIN", "fire"),
    ("REBOOT", "REBOOT", "reboot"),
]

DEFAULT_BAUD = 115200
BAUD_CHOICES = ["9600", "19200", "38400", "57600", "115200", "230400"]
MIN_SATS_FOR_FIX = 4          # GPS "fix" = at least this many sats and non-zero lat/lng
STALE_LINK_SECONDS = 3.0      # warn if no telemetry row for this long while connected
PLOT_REFRESH_MS = 33          # ~30 FPS plot refresh (decoupled from data rate)
DEMO_KEY = "__DEMO__"

# --------------------------------------------------------------------------
# Themes (Dark Aerospace & Sunlight-Readable Light Mode)
# --------------------------------------------------------------------------
THEMES = {
    "dark": {
        "plot_bg": "#0e1420",
        "plot_fg": "#e6edf3",
        "axis_pen": "#8fa0b8",
        "alt_curve": "#4dd0e1",
        "acc_curve": "#ffb020",
        "track_curve": "#55e6a5",
        "max_line": "#ff5555",
        "state_line": "#b58cff",
        "state_label_fg": "#d7baff",
        "state_label_bg": "#171e2b",
        "log_ts": "#6b7a90",
        "log_tx": "#ffb020",
        "log_rx": "#c9d4e5",
        "log_err": "#ff6b6b",
        "log_warn": "#ffb020",
        "log_trans": "#4dd0e1",
        "log_ok": "#5be584",
    },
    "light": {
        "plot_bg": "#ffffff",
        "plot_fg": "#0f172a",
        "axis_pen": "#475569",
        "alt_curve": "#0284c7",
        "acc_curve": "#d97706",
        "track_curve": "#059669",
        "max_line": "#dc2626",
        "state_line": "#7c3aed",
        "state_label_fg": "#4c1d95",
        "state_label_bg": "#ede9fe",
        "log_ts": "#64748b",
        "log_tx": "#b45309",
        "log_rx": "#1e293b",
        "log_err": "#dc2626",
        "log_warn": "#d97706",
        "log_trans": "#0284c7",
        "log_ok": "#15803d",
    },
}

DARK_STYLESHEET = """
QWidget { background:#0b0f17; color:#e6edf3; font-family:'Segoe UI','Inter','Helvetica Neue',sans-serif; font-size:13px; }
QLabel { background:transparent; }
QFrame#plotBox { background:#0e1420; border:2px solid #2c3a52; border-radius:10px; }
QFrame#topbar { background:#111826; border:1px solid #243044; border-radius:10px; }
QLabel#appTitle { font-size:16px; font-weight:700; letter-spacing:3px; color:#4dd0e1; }
QFrame#card { background:#131a26; border:1px solid #243044; border-radius:10px; }
QFrame#card[kind="warn"]     { background:#33290d; border:2px solid #ffb020; }
QFrame#card[kind="ok"]       { background:#0e3a22; border:2px solid #2ee66b; }
QFrame#card[kind="alert"]    { background:#6a0d14; border:2px solid #ff3b3b; }
QFrame#card[kind="alertdim"] { background:#2a0b0e; border:2px solid #7a1f24; }
QLabel#cardTitle { color:#7d8da6; font-size:11px; font-weight:600; letter-spacing:2px; }
QLabel#cardValue { font-size:16px; font-weight:700; }
QLabel#cardSub   { color:#8fa0b8; font-size:11px; }
QLabel#sectionTitle { color:#7d8da6; font-size:11px; font-weight:600; letter-spacing:2px; }
QLabel#telemetryBar { background:#131a26; border:1px solid #243044; border-radius:6px; padding:5px 8px; color:#c9d4e5; font-family:'Consolas','DejaVu Sans Mono',monospace; font-size:12px; }
QComboBox, QPushButton { background:#182134; border:1px solid #2c3a52; border-radius:6px; padding:6px 10px; color:#e6edf3; }
QComboBox:disabled, QPushButton:disabled { color:#5b6a80; }
QComboBox QAbstractItemView { background:#182134; color:#e6edf3; selection-background-color:#2c5a8a; }
QPushButton:hover { background:#1f2b42; }
QPushButton#connectBtn { background:#14532d; border:1px solid #2ee66b; font-weight:700; min-width:120px; color:#ffffff; }
QPushButton#connectBtn[connected="true"] { background:#6a0d14; border:1px solid #ff3b3b; color:#ffffff; }
QPlainTextEdit { background:#080b11; border:1px solid #243044; border-radius:8px; color:#e6edf3;
                 font-family:'Consolas','DejaVu Sans Mono',monospace; font-size:12px; }
QCheckBox { background:transparent; }
QSplitter::handle { background:#1a2333; }
QTabWidget::pane { border:1px solid #243044; border-radius:6px; }
QTabBar::tab { background:#111826; border:1px solid #243044; border-bottom:none; padding:6px 16px;
               border-top-left-radius:6px; border-top-right-radius:6px; color:#8fa0b8; }
QTabBar::tab:selected { background:#182134; color:#4dd0e1; }
QLineEdit { background:#182134; border:1px solid #2c3a52; border-radius:6px; padding:6px 10px; color:#e6edf3; }
QLineEdit:disabled { color:#5b6a80; }
QPushButton#quickFire { background:#6a0d14; border:1px solid #ff3b3b; font-weight:700; color:#ffffff; }
QPushButton#quickFire:disabled { background:#182134; border:1px solid #2c3a52; color:#5b6a80; }
QPushButton#quickPadTest { background:#163b50; border:1px solid #4dd0e1; font-weight:700; color:#e6edf3; }
QPushButton#quickPadTest:disabled { background:#182134; border:1px solid #2c3a52; color:#5b6a80; }
QCheckBox#overrideChk { color:#ff8a8a; font-weight:700; }
QStatusBar { background:#0b0f17; color:#8fa0b8; }
"""

LIGHT_STYLESHEET = """
QWidget { background:#f1f5f9; color:#0f172a; font-family:'Segoe UI','Inter','Helvetica Neue',sans-serif; font-size:13px; }
QLabel { background:transparent; color:#0f172a; }
QFrame#plotBox { background:#ffffff; border:2px solid #cbd5e1; border-radius:10px; }
QFrame#topbar { background:#ffffff; border:1px solid #cbd5e1; border-radius:10px; }
QLabel#appTitle { font-size:16px; font-weight:700; letter-spacing:3px; color:#0284c7; }
QFrame#card { background:#ffffff; border:1px solid #cbd5e1; border-radius:10px; }
QFrame#card[kind="warn"]     { background:#fef3c7; border:2px solid #d97706; }
QFrame#card[kind="ok"]       { background:#dcfce7; border:2px solid #16a34a; }
QFrame#card[kind="alert"]    { background:#fee2e2; border:2px solid #dc2626; }
QFrame#card[kind="alertdim"] { background:#fef2f2; border:2px solid #f87171; }
QLabel#cardTitle { color:#475569; font-size:11px; font-weight:700; letter-spacing:2px; }
QLabel#cardValue { font-size:16px; font-weight:700; color:#0f172a; }
QLabel#cardSub   { color:#475569; font-size:11px; }
QLabel#sectionTitle { color:#475569; font-size:11px; font-weight:700; letter-spacing:2px; }
QLabel#telemetryBar { background:#ffffff; border:1px solid #cbd5e1; border-radius:6px; padding:5px 8px; color:#1e293b; font-family:'Consolas','DejaVu Sans Mono',monospace; font-size:12px; }
QComboBox, QPushButton { background:#ffffff; border:1px solid #94a3b8; border-radius:6px; padding:6px 10px; color:#0f172a; }
QComboBox:disabled, QPushButton:disabled { color:#94a3b8; background:#f8fafc; border-color:#cbd5e1; }
QComboBox QAbstractItemView { background:#ffffff; color:#0f172a; selection-background-color:#bae6fd; selection-color:#0f172a; }
QPushButton:hover { background:#e2e8f0; }
QPushButton#connectBtn { background:#16a34a; border:1px solid #15803d; font-weight:700; min-width:120px; color:#ffffff; }
QPushButton#connectBtn[connected="true"] { background:#dc2626; border:1px solid #b91c1c; color:#ffffff; }
QPlainTextEdit { background:#ffffff; border:1px solid #cbd5e1; border-radius:8px; color:#0f172a;
                 font-family:'Consolas','DejaVu Sans Mono',monospace; font-size:12px; }
QCheckBox { background:transparent; color:#0f172a; }
QSplitter::handle { background:#cbd5e1; }
QTabWidget::pane { border:1px solid #cbd5e1; border-radius:6px; background:#ffffff; }
QTabBar::tab { background:#e2e8f0; border:1px solid #cbd5e1; border-bottom:none; padding:6px 16px;
               border-top-left-radius:6px; border-top-right-radius:6px; color:#475569; }
QTabBar::tab:selected { background:#ffffff; color:#0284c7; font-weight:600; }
QLineEdit { background:#ffffff; border:1px solid #94a3b8; border-radius:6px; padding:6px 10px; color:#0f172a; }
QLineEdit:disabled { color:#94a3b8; background:#f8fafc; }
QPushButton#quickFire { background:#dc2626; border:1px solid #b91c1c; font-weight:700; color:#ffffff; }
QPushButton#quickFire:disabled { background:#f8fafc; border:1px solid #cbd5e1; color:#94a3b8; }
QPushButton#quickPadTest { background:#0284c7; border:1px solid #0369a1; font-weight:700; color:#ffffff; }
QPushButton#quickPadTest:disabled { background:#f8fafc; border:1px solid #cbd5e1; color:#94a3b8; }
QCheckBox#overrideChk { color:#dc2626; font-weight:700; }
QStatusBar { background:#f1f5f9; color:#475569; }
"""


# --------------------------------------------------------------------------
# Geodesic helpers for recovery (Haversine distance & azimuth bearing)
# --------------------------------------------------------------------------
def haversine_m(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    """Great-circle ground distance in meters between two WGS84 coordinates."""
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lng2 - lng1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


def initial_bearing_deg(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    """Initial azimuth bearing (0..360 deg, North=0, East=90) from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lng2 - lng1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def cardinal_dir(deg: float) -> str:
    dirs = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    return dirs[int((deg + 22.5) // 45.0) % 8]


# --------------------------------------------------------------------------
# Binary packet protocol (26 bytes, little-endian)
# --------------------------------------------------------------------------
SYNC = b"\x55\xAA"                       # 0xAA55 little-endian
PKT_LEN = 26
#                        sync  ms   state acc  alt  cont  lat  lng  sats chk
PKT_STRUCT = struct.Struct("<H    I    B     f    f    B     i    i    B    B".replace(" ", ""))
assert PKT_STRUCT.size == PKT_LEN


def packet_checksum(pkt) -> int:
    """XOR of bytes 2..24 inclusive (everything between the sync word and the checksum)."""
    x = 0
    for b in pkt[2:25]:
        x ^= b
    return x


def decode_packet(pkt: bytes):
    """26 raw bytes -> (Time_ms, State, Accel, Alt, Lat, Lng, Sats, Continuity), or None if corrupt."""
    if len(pkt) != PKT_LEN or packet_checksum(pkt) != pkt[25]:
        return None
    _sync, t_ms, state, acc, alt, cont, lat_i, lng_i, sats, _chk = PKT_STRUCT.unpack(pkt)
    if not (math.isfinite(acc) and math.isfinite(alt)):
        return None
    return (t_ms, state, round(acc, 4), round(alt, 4),          # float32 -> tidy decimals
            lat_i / 10_000_000.0, lng_i / 10_000_000.0, sats, cont)


def build_packet(t_ms, state, acc, alt, lat, lng, sats, cont) -> bytes:
    """Used by the demo mode (and handy for tests): builds a valid packet."""
    raw = PKT_STRUCT.pack(0xAA55, t_ms & 0xFFFFFFFF, state, acc, alt, cont,
                          int(round(lat * 1e7)), int(round(lng * 1e7)), sats, 0)
    return raw[:25] + bytes([packet_checksum(raw)])


class PacketStream:
    """Splits a raw serial byte stream into telemetry packets and text lines.

    Scans byte by byte for 55 AA, reads 26 bytes, validates the XOR checksum.
    On a bad checksum it skips just the 0x55 and re-syncs (so one corrupted
    packet never costs more than itself). 0xAA is never valid ASCII, so the
    sync word cannot appear inside the text messages.
    feed() returns a list of ("packet", row, raw_bytes) / ("text", line, None) / ("bad", None, None).
    """

    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()

    def _text(self, chunk, events):
        # keep printable ASCII + tab/CR/LF only (drops binary junk)
        self.text.extend(b for b in chunk if b in (9, 10, 13) or 32 <= b < 127)
        while True:
            i = self.text.find(b"\n")
            if i < 0:
                break
            line = self.text[:i].decode("ascii", errors="replace").strip()
            del self.text[:i + 1]
            if line:
                events.append(("text", line, None))
        if len(self.text) > 512:
            self.text.clear()

    def feed(self, data):
        events = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                keep = 1 if self.buf.endswith(b"\x55") else 0   # a lone 0x55 might start a sync word
                n = len(self.buf) - keep
                if n > 0:
                    self._text(self.buf[:n], events)
                    del self.buf[:n]
                break
            if i > 0:
                self._text(self.buf[:i], events)
                del self.buf[:i]
            if len(self.buf) < PKT_LEN:
                break                                            # wait for the rest of the packet
            pkt = bytes(self.buf[:PKT_LEN])
            row = decode_packet(pkt)
            if row is not None:
                self.text.clear()                                # drop any half-line junk before a good packet
                events.append(("packet", row, pkt))
                del self.buf[:PKT_LEN]
            else:
                events.append(("bad", None, None))
                del self.buf[:1]                                 # skip this 0x55 and re-sync
        return events


# --------------------------------------------------------------------------
# CSV logger (owned/used by the reader thread)
# --------------------------------------------------------------------------
class CsvLogger:
    HEADER = ["pc_time", "kind", "Time_ms", "State", "AccelTotal", "AltFused",
              "Lat", "Lng", "Sats", "Continuity", "raw"]

    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._fh)
        self._w.writerow(self.HEADER)
        self._fh.flush()

    @staticmethod
    def _stamp():
        return datetime.now().isoformat(timespec="milliseconds")

    def log_data(self, row, raw):
        row = ["" if v is None or (isinstance(v, float) and math.isnan(v)) else v for v in row]
        self._w.writerow([self._stamp(), "DATA", *row, raw])
        self._fh.flush()          # flush each line: survives a laptop crash mid-flight

    def log_text(self, raw):
        self._w.writerow([self._stamp(), "TEXT", "", "", "", "", "", "", "", "", raw])
        self._fh.flush()

    def log_tx(self, raw):
        self._w.writerow([self._stamp(), "TX", "", "", "", "", "", "", "", "", raw])
        self._fh.flush()

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# Background reader threads
# --------------------------------------------------------------------------
class TelemetrySource(QtCore.QThread):
    """Base class: turns binary packets and text lines into Qt signals and CSV rows."""
    telemetry = QtCore.pyqtSignal(object)   # tuple of 8 values
    message = QtCore.pyqtSignal(str)        # free-text event log line
    failed = QtCore.pyqtSignal(str)         # fatal error text
    raw_line = QtCore.pyqtSignal(str, str)  # ("RX"|"TX", text) for the Serial Monitor tab
    bad_packet = QtCore.pyqtSignal()        # a packet failed its checksum and was dropped

    def __init__(self, logger: CsvLogger, parent=None):
        super().__init__(parent)
        self._logger = logger
        self._running = True
        self._tx = queue.Queue()            # commands from the GUI thread -> reader thread
        self._stream = PacketStream()
        self._bad = 0

    # ---- called from the GUI thread ----
    def send_text(self, text: str, eol: str = ""):
        self._tx.put(text + eol)

    # ---- called from the reader thread ----
    def _drain_tx(self):
        while True:
            try:
                payload = self._tx.get_nowait()
            except queue.Empty:
                return
            self.write_out(payload)
            shown = payload.rstrip("\r\n")
            self._logger.log_tx(shown)
            self.raw_line.emit("TX", shown)

    def write_out(self, text: str):
        pass

    def stop(self):
        self._running = False

    def process_bytes(self, chunk: bytes):
        """Feed raw serial bytes: binary packets -> telemetry, text lines -> event log."""
        for kind, a, b in self._stream.feed(chunk):
            if kind == "packet":
                self.handle_packet(a, b)
            elif kind == "text":
                self.handle_line(a)
            else:
                self._bad += 1
                self.bad_packet.emit()
                if self._bad == 1 or self._bad % 25 == 0:
                    self.message.emit(f"WARN: {self._bad} corrupted telemetry packet(s) dropped (bad checksum)")

    def handle_packet(self, row, raw: bytes):
        self._logger.log_data(row, raw.hex(" ").upper())
        t_ms, state, acc, alt, lat, lng, sats, cont = row
        self.raw_line.emit("RX", f"PKT t={t_ms} state={state} acc={acc:.2f} alt={alt:.2f} "
                                 f"lat={lat:.6f} lng={lng:.6f} sats={sats} cont={cont:02d}")
        self.telemetry.emit(row)

    def handle_line(self, text: str):
        text = text.strip()
        if not text:
            return
        self.raw_line.emit("RX", text)
        self._logger.log_text(text)
        self.message.emit(text)

    def run(self):
        try:
            self.loop()
        except Exception as exc:                      # never let the thread die silently
            self.failed.emit(f"Reader crashed: {exc!r}")
        finally:
            self._logger.close()

    def loop(self):
        raise NotImplementedError


class SerialSource(TelemetrySource):
    def __init__(self, port, baud, logger, parent=None):
        super().__init__(logger, parent)
        self.port, self.baud = port, baud
        self._ser = None

    def write_out(self, text):
        if self._ser is not None:
            self._ser.write(text.encode("utf-8"))
            self._ser.flush()

    def loop(self):
        try:
            ser = serial.serial_for_url(self.port, do_not_open=True)
            ser.baudrate, ser.timeout, ser.write_timeout = self.baud, 0.1, 1
            ser.dtr = False        # keep DTR/RTS low so merely connecting does NOT reset a wired board
            ser.rts = False
            ser.open()
            self._ser = ser
        except (serial.SerialException, OSError, ValueError) as exc:
            self.failed.emit(f"Cannot open {self.port}: {exc}")
            return
        self.message.emit(f"[GUI] Opened {self.port} @ {self.baud} baud")
        with ser:
            while self._running:
                try:
                    self._drain_tx()
                    chunk = ser.read(ser.in_waiting or 1)
                except (serial.SerialException, OSError) as exc:
                    self.failed.emit(f"Serial error (radio unplugged?): {exc}")
                    break
                if not chunk:
                    continue
                self.process_bytes(chunk)


class DemoSource(TelemetrySource):
    """Simulated flight (20 Hz) so the GUI can be tested without hardware."""

    _reboot_flag = False

    def write_out(self, text):
        cmd = text.strip()
        up = cmd.upper()
        if cmd.lower() == "test_baro":
            self.message.emit("BARO_TEST: sampling barometer (BMP/MS5611)...")
            self.message.emit("BARO_TEST: P=1013.25 hPa | Temp=26.4 C | Alt_AGL=0.50 m | RMS_noise=0.03 m -> READY")
        elif up in ("TEST_DROGUE", "TEST_MAIN"):
            ch = up.split("_", 1)[1]
            self.message.emit(f"TEST: {ch} CHANNEL FIRED (demo - no hardware)")
        elif up == "READ":
            self.message.emit("FLASH LOG: 1 flight stored, 4,820 samples (READY)")
        elif up == "CLEAR":
            self.message.emit("FLASH LOG: erased OK")
        elif up in ("REBOOT", "RESTART", "RESET"):
            self._reboot_flag = True
        else:
            self.message.emit(f"[DEMO] command received: {cmd}")

    def loop(self):
        self.message.emit("DEMO MODE: simulated flight (no serial port)")
        while self._running:
            self._reboot_flag = False
            self.message.emit("BOOT: demo flight computer starting")
            self.message.emit("ALL SENSORS NOMINAL")
            self._flight()

    def _flight(self):
        dt = 0.05
        t_ms, alt, v, state, st = 0, 0.5, 0.0, 1, 0.0
        lat0, lng0, dlat, dlng = 14.599512, 120.984221, 0.0, 0.0
        landed = False

        def go(new_state, name):
            nonlocal state, st
            state, st = new_state, 0.0
            self.message.emit(f"TRANSITION: {name}")

        while self._running:
            self._drain_tx()
            if self._reboot_flag:
                return                      # simulated reboot: restart the whole flight
            acc = 0.0
            if state == 1:
                alt = 0.5 + random.uniform(-0.05, 0.05)
                acc = abs(random.gauss(0.1, 0.05))
                if st >= 5:
                    go(2, "STATE_BOOST")
            elif state == 2:
                acc = 55 + random.gauss(0, 1.5)
                v += (acc - 9.81) * dt
                alt += v * dt
                if st >= 2.5:
                    go(3, "STATE_COAST")
            elif state == 3:
                drag = 0.002 * v * abs(v)
                acc = abs(drag) + random.gauss(0, 0.3)
                v += (-9.81 - drag) * dt
                alt += v * dt
                if v <= 0:
                    go(4, "STATE_APOGEE_SEARCH")
            elif state in (4, 5, 6):
                v += -9.81 * dt
                alt += v * dt
                acc = abs(random.gauss(0.2, 0.1))
                if state == 4 and st >= 0.8:
                    go(5, "STATE_APOGEE_DETECTED")
                elif state == 5 and st >= 0.4:
                    go(6, "STATE_FIRE_DROGUE")
                elif state == 6 and st >= 1.0:
                    go(7, "STATE_DROGUE_DESCENT")
            elif state == 7:
                v += (-22.0 - v) * 0.15
                alt += v * dt
                acc = abs(random.gauss(0.5, 0.3))
                if alt <= 150:
                    go(8, "STATE_FIRE_MAIN")
            elif state == 8:
                v += (-22.0 - v) * 0.15
                alt += v * dt
                acc = abs(random.gauss(0.5, 0.3))
                if st >= 1.0:
                    go(9, "STATE_MAIN_DESCENT")
            elif state == 9:
                v += (-5.5 - v) * 0.2
                alt += v * dt
                acc = abs(random.gauss(0.3, 0.2))
                if alt <= 0.5 and not landed:
                    alt, v, landed = 0.5, 0.0, True
                    self.message.emit("LANDING DETECTED")
            if landed:
                alt, acc = 0.5 + random.uniform(-0.03, 0.03), abs(random.gauss(0.1, 0.05))
            alt = max(alt, 0.5)

            if state >= 2 and not landed:               # wind drift
                dlat += random.gauss(8e-7, 4e-7)
                dlng += random.gauss(1.2e-6, 5e-7)
            sats = random.choice([7, 8, 8, 8, 9])
            wire_state = {1: 1, 2: 3, 3: 4, 4: 5, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9}[state]
            self.process_bytes(build_packet(t_ms, wire_state, acc, alt,
                                            lat0 + dlat, lng0 + dlng, sats, 22))
            t_ms += int(dt * 1000)
            st += dt
            time.sleep(dt)


class ReplaySource(TelemetrySource):
    """Replays a saved CSV flight log from ./telemetry_logs/ at adjustable speed."""

    def __init__(self, csv_path: Path, speed: float = 1.0, parent=None):
        super().__init__(logger=None, parent=parent)
        self.csv_path = Path(csv_path)
        self.speed = max(0.1, float(speed))

    def set_speed(self, speed: float):
        self.speed = max(0.1, float(speed))

    def run(self):
        try:
            self.loop()
        except Exception as exc:
            self.failed.emit(f"Replay error: {exc!r}")

    def loop(self):
        if not self.csv_path.is_file():
            self.failed.emit(f"Replay file not found: {self.csv_path}")
            return
        self.message.emit(f"[REPLAY] Loading {self.csv_path.name} ({self.speed:.1f}x speed)")
        with open(self.csv_path, "r", encoding="utf-8", errors="replace") as fh:
            reader = csv.DictReader(fh)
            prev_t_ms = None
            for r in reader:
                if not self._running:
                    break
                kind = (r.get("kind") or "").strip().upper()
                if kind == "DATA":
                    try:
                        t_ms = int(float(r["Time_ms"]))
                        state = int(float(r["State"]))
                        acc = float(r["AccelTotal"])
                        alt = float(r["AltFused"])
                        lat = float(r["Lat"]) if r.get("Lat") else float("nan")
                        lng = float(r["Lng"]) if r.get("Lng") else float("nan")
                        sats = int(float(r["Sats"])) if r.get("Sats") else 0
                        cont = int(float(r["Continuity"])) if r.get("Continuity") else None
                    except (KeyError, ValueError):
                        continue
                    if prev_t_ms is not None and t_ms > prev_t_ms:
                        dt_s = min(0.5, (t_ms - prev_t_ms) / 1000.0) / self.speed
                        if dt_s > 0.002:
                            time.sleep(dt_s)
                    prev_t_ms = t_ms
                    row = (t_ms, state, acc, alt, lat, lng, sats, cont)
                    self.raw_line.emit(
                        "RX",
                        f"REPLAY t={t_ms} state={state} acc={acc:.2f} alt={alt:.2f} "
                        f"lat={lat:.6f} lng={lng:.6f} sats={sats}"
                    )
                    self.telemetry.emit(row)
                elif kind == "TEXT":
                    raw = (r.get("raw") or "").strip()
                    if raw:
                        self.raw_line.emit("RX", raw)
                        self.message.emit(raw)
        if self._running:
            self.message.emit(f"[REPLAY] Finished playing {self.csv_path.name}")


# --------------------------------------------------------------------------
# Non-blocking Audio Callouts (TTS) for Flight Events
# --------------------------------------------------------------------------
class VoiceAnnouncer:
    """Background TTS announcer using pyttsx3 if installed or Windows System.Speech."""

    def __init__(self):
        self.enabled = True
        self._q = queue.Queue()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def say(self, phrase: str):
        if not self.enabled or not phrase:
            return
        # Drop backlog if multiple events fire at once so callouts stay current
        while self._q.qsize() > 3:
            try:
                self._q.get_nowait()
            except queue.Empty:
                break
        self._q.put(phrase)

    def _worker(self):
        engine = None
        try:
            import pyttsx3
            engine = pyttsx3.init()
            engine.setProperty("rate", 175)
        except Exception:
            engine = None

        while True:
            phrase = self._q.get()
            if phrase is None:
                break
            if not self.enabled:
                continue
            try:
                if engine is not None:
                    engine.say(phrase)
                    engine.runAndWait()
                elif sys.platform == "win32":
                    safe = phrase.replace("'", " ").replace('"', " ")
                    cmd = (
                        "Add-Type -AssemblyName System.Speech; "
                        "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                        "$s.Rate = 1; "
                        f"$s.Speak('{safe}')"
                    )
                    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0x08000000)
                    subprocess.run(
                        ["powershell", "-NoProfile", "-NonInteractive", "-Command", cmd],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=8,
                        creationflags=flags,
                    )
            except Exception:
                pass


# --------------------------------------------------------------------------
# Data buffer (numpy, grows dynamically so full flight is retained)
# --------------------------------------------------------------------------
class SeriesBuffer:
    """Grow storage instead of discarding the beginning of a flight."""
    fields = ("t", "alt", "acc", "vz", "lat", "lng", "state", "sats")

    def __init__(self, cap=400_000):
        self.cap, self.n = cap, 0
        for name in self.fields:
            setattr(self, name, np.empty(cap))

    def append(self, t, alt, acc, vz, lat, lng, state, sats):
        if self.n == self.cap:
            self.cap *= 2
            for name in self.fields:
                old = getattr(self, name)
                new = np.empty(self.cap)
                new[:self.n] = old
                setattr(self, name, new)
        for name, value in zip(self.fields, (t, alt, acc, vz, lat, lng, state, sats)):
            getattr(self, name)[self.n] = value
        self.n += 1

    def clear(self):
        self.n = 0


# --------------------------------------------------------------------------
# UI widgets
# --------------------------------------------------------------------------
class Card(QtWidgets.QFrame):
    """Dashboard status card with a title, big value and small sub-line."""

    def __init__(self, title: str):
        super().__init__()
        self.setObjectName("card")
        self.setProperty("kind", "normal")
        self._kind = "normal"
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(2)
        self.title = QtWidgets.QLabel(title.upper())
        self.title.setObjectName("cardTitle")
        self.value = QtWidgets.QLabel("---")
        self.value.setObjectName("cardValue")
        self.sub = QtWidgets.QLabel(" ")
        self.sub.setObjectName("cardSub")
        for w in (self.title, self.value, self.sub):
            lay.addWidget(w)

    def set(self, value=None, sub=None, kind=None):
        if value is not None:
            self.value.setText(value)
        if sub is not None:
            self.sub.setText(sub or " ")
        if kind is not None and kind != self._kind:
            self._kind = kind
            self.setProperty("kind", kind)
            self.style().unpolish(self)
            self.style().polish(self)


def nice_step(raw):
    """Round a raw tick spacing to a 'nice' 1/2/5 x 10^n value (stable, readable labels)."""
    if not math.isfinite(raw) or raw <= 0:
        return 1.0
    exp = math.floor(math.log10(raw))
    base = raw / 10 ** exp
    mult = 1 if base < 1.5 else 2 if base < 3.5 else 5 if base < 7.5 else 10
    return mult * 10 ** exp


class StableYRange:
    """Y-axis range that does NOT twitch with every sample.

    - snaps to 'nice' tick multiples, with 8 % padding and a minimum span
      (so sensor noise on a flat line doesn't zoom the axis to +-0.001)
    - GROWS immediately when data leaves the range
    - SHRINKS only after the data has needed a much smaller range for a while
    update() returns (lo, hi) when the range changed, otherwise None.
    """

    def __init__(self, min_span, shrink_after=4.0):
        self.min_span = min_span
        self.shrink_after = shrink_after
        self.reset()

    def reset(self):
        self.lo = self.hi = None
        self._shrink_since = None

    @staticmethod
    def _snap(lo, hi):
        step = nice_step((hi - lo) / 5.0)
        return math.floor(lo / step) * step, math.ceil(hi / step) * step

    def update(self, dmin, dmax):
        span = max(dmax - dmin, 0.0)
        pad = max(span, self.min_span) * 0.08
        lo, hi = dmin - pad, dmax + pad
        if hi - lo < self.min_span:                       # centre the minimum span on the data
            mid = (dmin + dmax) / 2.0
            lo, hi = mid - self.min_span / 2.0, mid + self.min_span / 2.0
        n_lo, n_hi = self._snap(lo, hi)

        if self.lo is None:                               # first range
            self.lo, self.hi = n_lo, n_hi
            return self.lo, self.hi

        if n_lo < self.lo or n_hi > self.hi:              # data escaped -> grow now
            sp = max(n_hi, self.hi) - min(n_lo, self.lo)
            new_lo = n_lo - 0.25 * sp if n_lo < self.lo else self.lo     # extra headroom so a
            new_hi = n_hi + 0.25 * sp if n_hi > self.hi else self.hi     # climbing trace doesn't
            self.lo, self.hi = self._snap(new_lo, new_hi)                # re-scale every frame
            self._shrink_since = None
            return self.lo, self.hi

        if (n_hi - n_lo) < 0.5 * (self.hi - self.lo):     # much roomier than needed -> shrink slowly
            now = time.monotonic()
            if self._shrink_since is None:
                self._shrink_since = now
            elif now - self._shrink_since >= self.shrink_after:
                self.lo, self.hi = n_lo, n_hi
                self._shrink_since = None
                return self.lo, self.hi
        else:
            self._shrink_since = None
        return None


class AdaptiveAxis(pg.AxisItem):
    """Choose 1/2/5 steps from the visible range AND available pixels."""
    def tickSpacing(self, minVal, maxVal, size):
        span = abs(maxVal - minVal)
        target = max(2, int(size / (110 if self.orientation in ("bottom", "top") else 55)))
        raw = max(span / target, 1e-12)
        power = 10 ** math.floor(math.log10(raw))
        step = next(v * power for v in (1, 2, 5, 10) if v * power >= raw)
        return [(step, 0)]

    def tickStrings(self, values, scale, spacing):
        decimals = max(0, -math.floor(math.log10(max(spacing, 1e-12))))
        return [f"{v:.{min(decimals, 10)}f}" if abs(v) < 1e7 else f"{v:.3g}" for v in values]


class PointInspector(QtCore.QObject):
    """Snap to a real sample within 14 screen pixels; click pins its details."""
    def __init__(self, plot, samples, describe, label):
        super().__init__(plot)
        self.plot, self.samples, self.describe, self.label = plot, samples, describe, label
        self.pinned = False
        self.dot = pg.ScatterPlotItem(size=11, pen=pg.mkPen("white", width=2), brush="#e056fd")
        plot.addItem(self.dot, ignoreBounds=True)
        self.proxy = pg.SignalProxy(plot.scene().sigMouseMoved, rateLimit=30, slot=lambda e: self.inspect(e[0]))
        plot.scene().sigMouseClicked.connect(self.click)

    def clear(self):
        self.pinned = False
        self.dot.setData([], [])
        self.label.setText("Hover near a trace to inspect. Click to pin; double-click to release.")

    def click(self, event):
        if event.button() != QtCore.Qt.MouseButton.LeftButton:
            return
        if event.double():
            self.clear()
        elif self.plot.getViewBox().sceneBoundingRect().contains(event.scenePos()):
            self.pinned = False
            self.pinned = self.inspect(event.scenePos())

    def inspect(self, pos):
        if self.pinned:
            return False
        vb = self.plot.getViewBox()
        if not vb.sceneBoundingRect().contains(pos):
            return False
        x, y, indices = self.samples()
        if len(x) == 0:
            return False
        pt = vb.mapSceneToView(pos)
        sx, sy = vb.viewPixelSize()
        dist = ((x - pt.x()) / max(abs(sx), 1e-15)) ** 2 + ((y - pt.y()) / max(abs(sy), 1e-15)) ** 2
        dist = np.where(np.isfinite(dist), dist, np.inf)
        j = int(np.argmin(dist))
        if dist[j] > 14 ** 2:
            self.dot.setData([], [])
            return False
        self.dot.setData([x[j]], [y[j]])
        self.label.setText(self.describe(int(indices[j])))
        return True


# --------------------------------------------------------------------------
# Folium + Leaflet.js Interactive Map Widget & Tactical Vector Radar (100% Offline)
# --------------------------------------------------------------------------
def _build_folium_live_html(default_lat=14.599512, default_lng=120.984221, is_dark=True) -> str:
    """Generate a clean, high-performance Leaflet HTML document with a live telemetry bridge.

    Directly initializes the map container and exposes `window.gsMap` so 20 Hz
    telemetry updates smoothly update the rocket position, flight track, apogee,
    and landing prediction without reloading the webpage or throwing null errors.
    Uses 100% free, keyless tile providers (Esri World Imagery, OpenStreetMap, Topo)
    and supports local offline tile packs (./tiles/{z}/{x}/{y}.png).
    """
    bg = "#0e1420" if is_dark else "#f1f5f9"
    card_bg = "#131a26" if is_dark else "#ffffff"
    fg = "#e6edf3" if is_dark else "#0f172a"
    border = "#243044" if is_dark else "#cbd5e1"
    track_color = "#34d399" if is_dark else "#059669"
    is_dark_str = "true" if is_dark else "false"

    app_dir = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent
    local_js = app_dir / "assets" / "leaflet" / "leaflet.js"
    local_css = app_dir / "assets" / "leaflet" / "leaflet.css"
    if local_js.is_file() and local_css.is_file():
        css_tag = f'<link rel="stylesheet" href="{local_css.as_uri()}"/>'
        js_tag = f'<script src="{local_js.as_uri()}"></script>'
    else:
        css_tag = '<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>'
        js_tag = '<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>'

    local_tiles_dir = app_dir / "tiles"
    if not local_tiles_dir.is_dir():
        local_tiles_dir = app_dir / "map_tiles"
    local_tile_url = (local_tiles_dir / "{z}" / "{x}" / "{y}.png").as_uri().replace("%7Bz%7D", "{z}").replace("%7Bx%7D", "{x}").replace("%7By%7D", "{y}") if local_tiles_dir.is_dir() else "tiles/{z}/{x}/{y}.png"

    return f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
{css_tag}
{js_tag}
<style>
  html, body, #map {{ height: 100%; width: 100%; margin: 0; padding: 0; background: {bg}; }}
  .leaflet-container {{ background: {bg} !important; font-family: 'Segoe UI', -apple-system, sans-serif; }}
  .leaflet-control-layers {{ background: {card_bg} !important; color: {fg} !important; border-radius: 6px; border: 1px solid {border} !important; font-size: 12px; }}
  .leaflet-control-layers-expanded {{ padding: 8px 12px; }}
  .leaflet-control-scale-line {{ background: rgba(0,0,0,0.65) !important; color: #ffffff !important; border-color: #ffffff !important; }}
  /* Keyless Aerospace Dark Mode tile filter (converts standard OSM into a dark tactical grid without CARTO API key) */
  .aerospace-dark-tiles img, .aerospace-dark-tiles .leaflet-tile {{
    filter: invert(100%) hue-rotate(180deg) brightness(85%) contrast(90%) !important;
  }}
</style>
</head>
<body>
<div id="map"></div>
<script>
(function() {{
    if (typeof L === 'undefined') {{
        console.warn("Leaflet library not loaded (offline mode)");
        window.gsMap = {{
            setTheme: function() {{}},
            setTrackPoints: function() {{}},
            fitTrack: function() {{}},
            centerRocket: function() {{}},
            centerPad: function() {{}},
            clear: function() {{}}
        }};
        return;
    }}

    var map = L.map('map', {{
        preferCanvas: true,
        zoomControl: true,
        attributionControl: true
    }}).setView([{default_lat}, {default_lng}], 16);

    // 100% Free / Public layers - Zero API keys required (no CARTO API key needed)
    var osmStandard = L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
        attribution: '&copy; OpenStreetMap contributors',
        subdomains: 'abc',
        maxZoom: 19
    }});
    var darkOsm = L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
        attribution: '&copy; OpenStreetMap contributors',
        subdomains: 'abc',
        maxZoom: 19,
        className: 'aerospace-dark-tiles'
    }});
    var satLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
        attribution: 'Tiles &copy; Esri World Imagery',
        maxZoom: 19
    }});
    var topoLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
        attribution: 'Tiles &copy; Esri Topo',
        maxZoom: 19
    }});
    var localTileLayer = L.tileLayer('{local_tile_url}', {{
        attribution: 'Local Offline Cache (./tiles/)',
        maxZoom: 19,
        errorTileUrl: ''
    }});

    var isDark = {is_dark_str};
    var currentBase = isDark ? darkOsm : osmStandard;
    currentBase.addTo(map);

    L.control.layers({{
        "Aerospace Dark (No Key)": darkOsm,
        "Satellite (Esri Imagery)": satLayer,
        "Topographic (Esri Topo)": topoLayer,
        "OpenStreetMap (Standard)": osmStandard,
        "Local Offline Tiles (./tiles/)": localTileLayer
    }}, null, {{position: 'topright', collapsed: true}}).addTo(map);

    L.control.scale({{imperial: false, metric: true, position: 'bottomleft'}}).addTo(map);

    var trackPath = L.polyline([], {{color: '{track_color}', weight: 3.5, opacity: 0.9}}).addTo(map);
    var losLine = L.polyline([], {{color: '#f59e0b', weight: 2, dashArray: '6, 6', opacity: 0.85}}).addTo(map);
    var driftLine = L.polyline([], {{color: '#ec4899', weight: 2.5, dashArray: '4, 6', opacity: 0.9}}).addTo(map);
    var landingCircle = null;
    var landingMarker = null;
    var padMarker = null;
    var rocketMarker = null;
    var apogeeMarker = null;
    var userPanned = false;

    map.on('dragstart', function() {{ userPanned = true; }});

    var padIcon = L.divIcon({{
        className: 'gs-pad-icon',
        html: '<div style="width:14px;height:14px;border-radius:50%;background:#3b82f6;border:2.5px solid #ffffff;box-shadow:0 0 6px rgba(0,0,0,0.6);" title="Launch Pad"></div>',
        iconSize: [14, 14],
        iconAnchor: [7, 7]
    }});
    var rocketIcon = L.divIcon({{
        className: 'gs-rocket-icon',
        html: '<div style="width:18px;height:18px;border-radius:50%;background:#ef4444;border:3px solid #ffffff;box-shadow:0 0 10px rgba(239,68,68,0.9);" title="Rocket"></div>',
        iconSize: [18, 18],
        iconAnchor: [9, 9]
    }});
    var apogeeIcon = L.divIcon({{
        className: 'gs-apogee-icon',
        html: '<div style="width:12px;height:12px;border-radius:50%;background:#a855f7;border:2px solid #ffffff;box-shadow:0 0 6px rgba(0,0,0,0.6);" title="Apogee"></div>',
        iconSize: [12, 12],
        iconAnchor: [6, 6]
    }});
    var landingIcon = L.divIcon({{
        className: 'gs-landing-icon',
        html: '<div style="width:14px;height:14px;border-radius:50%;background:#ec4899;border:2px dashed #ffffff;box-shadow:0 0 8px rgba(236,72,153,0.9);" title="Predicted Landing"></div>',
        iconSize: [14, 14],
        iconAnchor: [7, 7]
    }});

    window.gsMap = {{
        setTheme: function(dark) {{
            isDark = !!dark;
            var newBg = isDark ? '#0e1420' : '#f1f5f9';
            document.body.style.background = newBg;
            var mapEl = document.getElementById('map');
            if (mapEl) mapEl.style.background = newBg;
            if (isDark) {{
                if (map.hasLayer(osmStandard)) {{
                    map.removeLayer(osmStandard);
                    darkOsm.addTo(map);
                }}
            }} else {{
                if (map.hasLayer(darkOsm)) {{
                    map.removeLayer(darkOsm);
                    osmStandard.addTo(map);
                }}
            }}
            trackPath.setStyle({{color: isDark ? '#34d399' : '#059669'}});
        }},
        setTrackPoints: function(pts, padCoord, apogeeCoord, landingPred, curInfo, autoFollow) {{
            trackPath.setLatLngs(pts);
            if (padCoord) {{
                if (!padMarker) {{
                    padMarker = L.marker(padCoord, {{icon: padIcon}}).addTo(map)
                        .bindPopup('<b>Launch Pad</b><br>' + padCoord[0].toFixed(6) + ', ' + padCoord[1].toFixed(6));
                }} else {{
                    padMarker.setLatLng(padCoord);
                }}
            }}
            if (apogeeCoord) {{
                if (!apogeeMarker) {{
                    apogeeMarker = L.marker([apogeeCoord[0], apogeeCoord[1]], {{icon: apogeeIcon}}).addTo(map);
                }} else {{
                    apogeeMarker.setLatLng([apogeeCoord[0], apogeeCoord[1]]);
                }}
                apogeeMarker.bindPopup('<b>Max Altitude (Apogee)</b><br>' + apogeeCoord[2].toFixed(1) + ' m');
            }}
            if (pts && pts.length > 0) {{
                var last = pts[pts.length - 1];
                if (!rocketMarker) {{
                    rocketMarker = L.marker(last, {{icon: rocketIcon}}).addTo(map);
                }} else {{
                    rocketMarker.setLatLng(last);
                }}
                if (curInfo) {{
                    rocketMarker.bindPopup(
                        '<b>Rocket (' + curInfo.state + ')</b><br>' +
                        'Alt: ' + curInfo.alt.toFixed(1) + ' m | Vz: ' + curInfo.vz.toFixed(1) + ' m/s<br>' +
                        'Pos: ' + last[0].toFixed(6) + ', ' + last[1].toFixed(6) + '<br>' +
                        'Sats: ' + curInfo.sats
                    );
                }}
                if (padCoord) {{
                    losLine.setLatLngs([padCoord, last]);
                }}
                if (landingPred) {{
                    var lpt = [landingPred[0], landingPred[1]];
                    var lrad = landingPred[2];
                    driftLine.setLatLngs([last, lpt]);
                    if (!landingCircle) {{
                        landingCircle = L.circle(lpt, {{
                            radius: lrad, color: '#ec4899', weight: 2,
                            fillColor: '#ec4899', fillOpacity: 0.18
                        }}).addTo(map);
                    }} else {{
                        landingCircle.setLatLng(lpt);
                        landingCircle.setRadius(lrad);
                    }}
                    if (!landingMarker) {{
                        landingMarker = L.marker(lpt, {{icon: landingIcon}}).addTo(map);
                    }} else {{
                        landingMarker.setLatLng(lpt);
                    }}
                    landingMarker.bindPopup(
                        '<b>Predicted Landing Zone</b><br>' +
                        lpt[0].toFixed(6) + ', ' + lpt[1].toFixed(6) + '<br>' +
                        'Radius: &plusmn;' + lrad.toFixed(0) + ' m'
                    );
                }} else {{
                    driftLine.setLatLngs([]);
                    if (landingCircle) {{ map.removeLayer(landingCircle); landingCircle = null; }}
                    if (landingMarker) {{ map.removeLayer(landingMarker); landingMarker = null; }}
                }}
                if (autoFollow && !userPanned) {{
                    if (pts.length === 1) {{
                        map.setView(last, Math.max(map.getZoom(), 17));
                    }} else if (pts.length > 1) {{
                        var b = trackPath.getBounds();
                        if (padCoord) b.extend(padCoord);
                        if (landingPred) b.extend([landingPred[0], landingPred[1]]);
                        if (b.isValid()) {{
                            map.fitBounds(b, {{padding: [35, 35], maxZoom: 18}});
                        }}
                    }}
                }}
            }}
        }},
        fitTrack: function() {{
            userPanned = false;
            var pts = trackPath.getLatLngs();
            if (pts && pts.length > 1) {{
                var b = trackPath.getBounds();
                if (padMarker) b.extend(padMarker.getLatLng());
                if (landingMarker) b.extend(landingMarker.getLatLng());
                map.fitBounds(b, {{padding: [35, 35], maxZoom: 18}});
            }} else if (pts && pts.length === 1) {{
                map.setView(pts[0], 17);
            }}
        }},
        centerRocket: function() {{
            if (rocketMarker) {{
                map.setView(rocketMarker.getLatLng(), Math.max(map.getZoom(), 17));
            }}
        }},
        centerPad: function() {{
            if (padMarker) {{
                map.setView(padMarker.getLatLng(), Math.max(map.getZoom(), 17));
            }}
        }},
        clear: function() {{
            trackPath.setLatLngs([]);
            losLine.setLatLngs([]);
            driftLine.setLatLngs([]);
            if (landingCircle) {{ map.removeLayer(landingCircle); landingCircle = null; }}
            if (landingMarker) {{ map.removeLayer(landingMarker); landingMarker = null; }}
            if (padMarker) {{ map.removeLayer(padMarker); padMarker = null; }}
            if (rocketMarker) {{ map.removeLayer(rocketMarker); rocketMarker = null; }}
            if (apogeeMarker) {{ map.removeLayer(apogeeMarker); apogeeMarker = null; }}
            userPanned = false;
        }}
    }};
}})();
</script>
</body>
</html>"""


class GPSMap(QtWidgets.QWidget):
    """Interactive GPS map with dual-mode display:
    1. Interactive Leaflet/Folium satellite web view (with keyless OpenStreetMap and Esri World Imagery).
    2. Tactical Vector Radar (100% offline PyQtGraph rendering with metric range rings, cardinal axes, LOS, and drift vector).
    """

    def __init__(self, theme="dark"):
        super().__init__()
        self.theme = theme
        self.pad_coord = None         # (lat, lng) of launch pad
        self.apogee_coord = None      # (lat, lng, alt) at max altitude
        self.landing_pred = None      # (pred_lat, pred_lng, radius_m) during descent
        self.last_valid_fix = None    # (lat, lng, alt, sats, t, state)
        self._web_ready = False
        self._last_pushed_count = 0
        self._last_ring_pad = None

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        # --- Map control bar ---
        bar = QtWidgets.QHBoxLayout()
        bar.setSpacing(6)

        self.btn_toggle_view = QtWidgets.QPushButton("🎯 Vector Radar (Offline)")
        self.btn_toggle_view.setToolTip("Toggle between Web Satellite Map and 100% Offline Tactical Vector Radar")
        self.btn_toggle_view.clicked.connect(self.toggle_view_mode)
        bar.addWidget(self.btn_toggle_view)

        self.fit_chk = QtWidgets.QCheckBox("Auto-follow")
        self.fit_chk.setChecked(True)
        self.fit_chk.setToolTip("Automatically pan/zoom the map/radar to keep the launch pad and rocket in view")
        bar.addWidget(self.fit_chk)

        self.btn_fit = QtWidgets.QPushButton("Fit Track")
        self.btn_fit.clicked.connect(self.fit_track_now)
        bar.addWidget(self.btn_fit)

        self.btn_rocket = QtWidgets.QPushButton("Rocket")
        self.btn_rocket.setToolTip("Center on current rocket coordinates")
        self.btn_rocket.clicked.connect(self.center_rocket)
        bar.addWidget(self.btn_rocket)

        self.btn_set_pad = QtWidgets.QPushButton("Latch Pad")
        self.btn_set_pad.setToolTip("Set current GPS position as the Launch Pad reference for distance/bearing")
        self.btn_set_pad.clicked.connect(self.latch_pad_here)
        bar.addWidget(self.btn_set_pad)

        self.btn_copy = QtWidgets.QPushButton("Copy GPS")
        self.btn_copy.setToolTip("Copy last valid GPS coordinates & Google Maps link to clipboard")
        self.btn_copy.clicked.connect(self.copy_coordinates)
        bar.addWidget(self.btn_copy)

        self.btn_export = QtWidgets.QPushButton("Export HTML")
        self.btn_export.setToolTip("Save a standalone interactive Folium HTML map and open it in your browser")
        self.btn_export.clicked.connect(self.export_folium_html)
        bar.addWidget(self.btn_export)

        bar.addStretch(1)
        layout.addLayout(bar)

        # --- Stacked viewport: Web Map (Folium/Leaflet) ⟷ Tactical Vector Radar (100% Offline) ---
        self.stack = QtWidgets.QStackedWidget()
        self.web_view = None

        # 1. Native Tactical Vector Radar (PyQtGraph) - ALWAYS initialized, 100% offline
        self.plot = pg.PlotWidget(axisItems={"bottom": AdaptiveAxis("bottom"), "left": AdaptiveAxis("left")})
        self.plot.setBackground(THEMES[theme]["plot_bg"])
        self.plot.setLabel("bottom", "Longitude (°)")
        self.plot.setLabel("left", "Latitude (°)")
        self.plot.setAspectLocked(True, ratio=1.0)
        for name in ("left", "bottom"):
            self.plot.getAxis(name).enableAutoSIPrefix(False)
        self.plot.showGrid(x=True, y=True, alpha=0.15)
        self.plot.disableAutoRange()
        self.plot.getViewBox().sigRangeChangedManually.connect(lambda *_: self.fit_chk.setChecked(False))

        # Radar range rings and crosshairs
        self.radar_rings_item = self.plot.plot(
            pen=pg.mkPen("#334155" if theme == "dark" else "#cbd5e1", width=1.2, style=QtCore.Qt.PenStyle.DashLine),
            connect="finite"
        )
        self.radar_crosshair_item = self.plot.plot(
            pen=pg.mkPen("#334155" if theme == "dark" else "#cbd5e1", width=1.0, style=QtCore.Qt.PenStyle.DotLine),
            connect="finite"
        )
        self.ring_labels = []

        # Radar flight path and markers
        self.track = self.plot.plot(pen=pg.mkPen(THEMES[theme]["track_curve"], width=2.8), connect="finite")
        self.los_line = self.plot.plot(pen=pg.mkPen("#f59e0b", width=1.8, style=QtCore.Qt.PenStyle.DashLine), connect="finite")
        self.drift_line = self.plot.plot(pen=pg.mkPen("#ec4899", width=2.2, style=QtCore.Qt.PenStyle.DashLine), connect="finite")
        self.landing_circle_item = self.plot.plot(pen=pg.mkPen("#ec4899", width=1.8, style=QtCore.Qt.PenStyle.DotLine), connect="finite")

        self.pad_item = pg.ScatterPlotItem(size=14, symbol='s', brush="#3b82f6", pen=pg.mkPen("white", width=2))
        self.apogee_item = pg.ScatterPlotItem(size=14, symbol='star', brush="#a855f7", pen=pg.mkPen("white", width=1.5))
        self.current = pg.ScatterPlotItem(size=16, symbol='o', brush="#ef4444", pen=pg.mkPen("white", width=2))
        self.landing_item = pg.ScatterPlotItem(size=13, symbol='d', brush="#ec4899", pen=pg.mkPen("white", width=1.5))

        self.plot.addItem(self.pad_item)
        self.plot.addItem(self.apogee_item)
        self.plot.addItem(self.landing_item)
        self.plot.addItem(self.current)

        # 2. Interactive Web Map (Folium/Leaflet + QWebEngineView)
        if HAS_WEBENGINE:
            self.web_view = QWebEngineView(self)
            settings = self.web_view.settings()
            settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
            settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
            self.web_view.loadFinished.connect(self._on_web_loaded)
            html_str = _build_folium_live_html(is_dark=(theme == "dark"))
            self.web_view.setHtml(html_str, QtCore.QUrl("about:blank"))
            self.stack.addWidget(self.web_view)    # Index 0: Web View
            self.stack.addWidget(self.plot)        # Index 1: Tactical Vector Radar
            self.stack.setCurrentIndex(0)
        else:
            self.stack.addWidget(self.plot)
            self.stack.setCurrentIndex(0)
            self.btn_toggle_view.setEnabled(False)
            self.btn_toggle_view.setText("🎯 Vector Radar (Active)")

        layout.addWidget(self.stack, 1)

        # --- Recovery telemetry readout bar ---
        self.recovery_bar = QtWidgets.QLabel("PAD: unset  |  DOWNRANGE: ---  |  BEARING: ---  |  ELEVATION: ---")
        self.recovery_bar.setObjectName("telemetryBar")
        self.recovery_bar.setWordWrap(True)
        layout.addWidget(self.recovery_bar)

        engine_note = (
            "Web Map (Keyless OSM & Esri) + Tactical Radar ready."
            if (HAS_WEBENGINE and HAS_FOLIUM)
            else "Web Map + Tactical Radar ready (run `pip install folium` for Folium HTML export)."
            if HAS_WEBENGINE
            else "100% Offline Tactical Vector Radar active."
        )
        self.status = QtWidgets.QLabel(f"Waiting for GPS fix (>= {MIN_SATS_FOR_FIX} sats). {engine_note}")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

    def toggle_view_mode(self):
        """Toggle between interactive web satellite map and 100% offline tactical vector radar."""
        if self.web_view is not None and self.stack.currentWidget() == self.web_view:
            self.stack.setCurrentWidget(self.plot)
            self.btn_toggle_view.setText("🗺️ Web Map")
            self.btn_toggle_view.setToolTip("Switch to Leaflet satellite/street web map")
            self.status.setText("🎯 Tactical Vector Radar active (100% Offline mode — zero internet needed).")
            self.fit_track_now()
        else:
            if self.web_view is not None:
                self.stack.setCurrentWidget(self.web_view)
                self.btn_toggle_view.setText("🎯 Vector Radar (Offline)")
                self.btn_toggle_view.setToolTip("Switch to 100% offline Tactical Vector Radar with range rings")
                self.status.setText("🗺️ Web Map active (Keyless OSM & Esri Satellite).")
                self.fit_track_now()

    def _rebuild_radar_rings(self, pad_lat: float, pad_lng: float):
        """Draw concentric metric range rings (50m to 5km) and cardinal crosshairs around the pad."""
        if self.plot is None:
            return
        for lbl in self.ring_labels:
            try:
                self.plot.removeItem(lbl)
            except Exception:
                pass
        self.ring_labels.clear()

        radii_m = [50, 100, 250, 500, 1000, 2000, 5000]
        cos_lat = max(0.05, math.cos(math.radians(pad_lat)))
        m_per_lat = 111139.0
        m_per_lng = 111139.0 * cos_lat

        ring_x = []
        ring_y = []
        theta = np.linspace(0, 2 * np.pi, 48)
        text_color = "#94a3b8" if self.theme == "dark" else "#64748b"

        for r in radii_m:
            r_lat = r / m_per_lat
            r_lng = r / m_per_lng
            xs = pad_lng + r_lng * np.cos(theta)
            ys = pad_lat + r_lat * np.sin(theta)
            ring_x.extend(xs)
            ring_x.append(np.nan)
            ring_y.extend(ys)
            ring_y.append(np.nan)

            tag = f"{r}m" if r < 1000 else f"{r/1000:.1f}km"
            txt = pg.TextItem(text=tag, color=text_color, anchor=(0.5, 1.2))
            txt.setPos(pad_lng, pad_lat + r_lat)
            self.plot.addItem(txt, ignoreBounds=True)
            self.ring_labels.append(txt)

        self.radar_rings_item.setData(ring_x, ring_y, connect="finite")

        max_r = 5000.0
        max_lat_d = max_r / m_per_lat
        max_lng_d = max_r / m_per_lng
        ch_x = [pad_lng, pad_lng, np.nan, pad_lng - max_lng_d, pad_lng + max_lng_d]
        ch_y = [pad_lat - max_lat_d, pad_lat + max_lat_d, np.nan, pad_lat, pad_lat]
        self.radar_crosshair_item.setData(ch_x, ch_y, connect="finite")

        for label, pos in [
            ("N", (pad_lng, pad_lat + max_lat_d)),
            ("S", (pad_lng, pad_lat - max_lat_d)),
            ("E", (pad_lng + max_lng_d, pad_lat)),
            ("W", (pad_lng - max_lng_d, pad_lat)),
        ]:
            c_txt = pg.TextItem(text=f"<b>{label}</b>", color="#38bdf8", anchor=(0.5, 0.5))
            c_txt.setPos(*pos)
            self.plot.addItem(c_txt, ignoreBounds=True)
            self.ring_labels.append(c_txt)

    def _on_web_loaded(self, ok: bool):
        self._web_ready = bool(ok)
        if self._web_ready:
            self._run_js(f"if (window.gsMap) window.gsMap.setTheme({'true' if self.theme == 'dark' else 'false'});")
        else:
            if self.web_view is not None and self.stack.currentWidget() == self.web_view:
                self.stack.setCurrentWidget(self.plot)
                self.btn_toggle_view.setText("🗺️ Web Map")
                self.status.setText("⚠️ Offline / Web Map unavailable — switched to 100% Offline Tactical Vector Radar.")

    def _run_js(self, code: str):
        if self.web_view is not None and self._web_ready:
            self.web_view.page().runJavaScript(code)

    def apply_theme(self, theme: str):
        self.theme = theme
        is_dark = (theme == "dark")
        t = THEMES[theme]
        if self.web_view is not None:
            self._run_js(f"if (window.gsMap) window.gsMap.setTheme({'true' if is_dark else 'false'});")
        if self.plot is not None:
            self.plot.setBackground(t["plot_bg"])
            self.track.setPen(pg.mkPen(t["track_curve"], width=2.8))
            self.radar_rings_item.setPen(pg.mkPen("#334155" if is_dark else "#cbd5e1", width=1.2, style=QtCore.Qt.PenStyle.DashLine))
            self.radar_crosshair_item.setPen(pg.mkPen("#334155" if is_dark else "#cbd5e1", width=1.0, style=QtCore.Qt.PenStyle.DotLine))
            for name in ("left", "bottom"):
                ax = self.plot.getAxis(name)
                ax.setTextPen(pg.mkPen(t["plot_fg"]))
                ax.setPen(pg.mkPen(t["axis_pen"], width=1.5))
            if self.pad_coord is not None:
                self._rebuild_radar_rings(*self.pad_coord)

    def fit_track_now(self):
        self.fit_chk.setChecked(True)
        if self.web_view is not None:
            self._run_js("if (window.gsMap) window.gsMap.fitTrack();")
        if self.plot is not None and self.last_valid_fix is not None:
            x, y = self.track.getData()
            if x is not None and len(x) > 0:
                valid_mask = np.isfinite(x) & np.isfinite(y)
                if np.any(valid_mask):
                    xmin = min(self.pad_coord[1] if self.pad_coord else np.inf, float(np.min(x[valid_mask])))
                    xmax = max(self.pad_coord[1] if self.pad_coord else -np.inf, float(np.max(x[valid_mask])))
                    ymin = min(self.pad_coord[0] if self.pad_coord else np.inf, float(np.min(y[valid_mask])))
                    ymax = max(self.pad_coord[0] if self.pad_coord else -np.inf, float(np.max(y[valid_mask])))
                    pad_x = max((xmax - xmin) * 0.15, 0.001)
                    pad_y = max((ymax - ymin) * 0.15, 0.001)
                    self.plot.setRange(xRange=(xmin - pad_x, xmax + pad_x), yRange=(ymin - pad_y, ymax + pad_y), padding=0)

    def center_rocket(self):
        if self.web_view is not None:
            self._run_js("if (window.gsMap) window.gsMap.centerRocket();")
        if self.plot is not None and self.last_valid_fix is not None:
            lat, lng, *_ = self.last_valid_fix
            self.plot.setRange(xRange=(lng - 0.0015, lng + 0.0015), yRange=(lat - 0.0015, lat + 0.0015), padding=0)

    def center_pad(self):
        if self.web_view is not None:
            self._run_js("if (window.gsMap) window.gsMap.centerPad();")
        if self.plot is not None and self.pad_coord is not None:
            lat, lng = self.pad_coord
            self.plot.setRange(xRange=(lng - 0.0015, lng + 0.0015), yRange=(lat - 0.0015, lat + 0.0015), padding=0)

    def latch_pad_here(self):
        if self.last_valid_fix is None:
            QtWidgets.QMessageBox.information(self, "Latch Pad", "No valid GPS fix received yet.")
            return
        lat, lng, *_ = self.last_valid_fix
        self.pad_coord = (lat, lng)
        self._last_pushed_count = -1  # force refresh
        self._rebuild_radar_rings(lat, lng)

    def copy_coordinates(self):
        if self.last_valid_fix is None:
            QtWidgets.QMessageBox.information(self, "Copy GPS", "No valid GPS coordinates available yet.")
            return
        lat, lng, alt, sats, *_ = self.last_valid_fix
        text = f"{lat:.6f}, {lng:.6f} (Alt {alt:.1f} m, Sats {sats}) https://maps.google.com/?q={lat:.6f},{lng:.6f}"
        QtWidgets.QApplication.clipboard().setText(text)
        self.status.setText(f"Copied to clipboard: {lat:.6f}°, {lng:.6f}° (Google Maps link included)")

    def export_folium_html(self):
        if not HAS_FOLIUM:
            QtWidgets.QMessageBox.warning(
                self, "Folium Not Installed",
                "Install folium to export standalone HTML maps:\n\n    pip install folium"
            )
            return
        win = self.window()
        data = getattr(win, "data", None)
        if data is None or data.n == 0:
            QtWidgets.QMessageBox.information(self, "Export Map", "No telemetry data to export yet.")
            return

        n = data.n
        lat, lng, alt, sats = data.lat[:n], data.lng[:n], data.alt[:n], data.sats[:n]
        valid = (
            np.isfinite(lat) & np.isfinite(lng)
            & (np.abs(lat) <= 85.05112878) & (np.abs(lng) <= 180)
            & ((lat != 0) | (lng != 0)) & (sats >= MIN_SATS_FOR_FIX)
        )
        ids = np.flatnonzero(valid)
        if len(ids) == 0:
            QtWidgets.QMessageBox.information(self, "Export Map", "No valid GPS fixes in the current session.")
            return

        pts = [[float(lat[i]), float(lng[i])] for i in ids]
        center = pts[-1]
        m = folium.Map(location=center, zoom_start=16, tiles="OpenStreetMap", control_scale=True)
        folium.TileLayer(
            tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            attr="Esri World Imagery",
            name="Satellite (Esri Imagery)",
        ).add_to(m)
        folium.TileLayer(
            tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}",
            attr="Esri Topo",
            name="Topographic (Esri Topo)",
        ).add_to(m)

        folium.PolyLine(pts, color="#10b981", weight=4, opacity=0.9, tooltip="Rocket Flight Track").add_to(m)
        pad = self.pad_coord or (pts[0][0], pts[0][1])
        folium.Marker(
            location=[pad[0], pad[1]],
            popup=f"Launch Pad: {pad[0]:.6f}, {pad[1]:.6f}",
            tooltip="Launch Pad",
            icon=folium.Icon(color="blue", icon="flag"),
        ).add_to(m)
        if self.apogee_coord is not None:
            alat, alng, aalt = self.apogee_coord
            folium.Marker(
                location=[alat, alng],
                popup=f"Apogee: {aalt:.1f} m ({alat:.6f}, {alng:.6f})",
                tooltip=f"Apogee ({aalt:.1f} m)",
                icon=folium.Icon(color="purple", icon="star"),
            ).add_to(m)
        last_i = int(ids[-1])
        folium.Marker(
            location=pts[-1],
            popup=f"Rocket / Landing: {pts[-1][0]:.6f}, {pts[-1][1]:.6f} | Alt: {alt[last_i]:.1f} m",
            tooltip="Rocket Last Position",
            icon=folium.Icon(color="red", icon="rocket", prefix="fa"),
        ).add_to(m)
        if self.landing_pred is not None:
            plat, plng, prad = self.landing_pred
            folium.Circle(
                location=[plat, plng],
                radius=prad,
                color="#ec4899",
                fill=True,
                fill_opacity=0.2,
                tooltip=f"Predicted Landing Zone (±{prad:.0f} m)",
            ).add_to(m)
        folium.LayerControl().add_to(m)

        out_dir = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent
        out_path = out_dir / "telemetry_logs" / f"flight_map_{datetime.now().strftime('%Y%m%d_%H%M%S')}.html"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        m.save(str(out_path))
        webbrowser.open(out_path.as_uri())
        self.status.setText(f"Saved Folium HTML map to {out_path}")

    def clear_map(self):
        self.pad_coord = None
        self.apogee_coord = None
        self.landing_pred = None
        self.last_valid_fix = None
        self._last_pushed_count = 0
        self._last_ring_pad = None
        if self.web_view is not None:
            self._run_js("if (window.gsMap) window.gsMap.clear();")
        if self.plot is not None:
            self.track.setData([], [])
            self.pad_item.setData([], [])
            self.current.setData([], [])
            self.apogee_item.setData([], [])
            self.los_line.setData([], [])
            self.drift_line.setData([], [])
            self.landing_item.setData([], [])
            self.landing_circle_item.setData([], [])
            self.radar_rings_item.setData([], [])
            self.radar_crosshair_item.setData([], [])
            for lbl in self.ring_labels:
                try:
                    self.plot.removeItem(lbl)
                except Exception:
                    pass
            self.ring_labels.clear()
        self.recovery_bar.setText("PAD: unset  |  DOWNRANGE: ---  |  BEARING: ---  |  ELEVATION: ---")
        self.status.setText("Map cleared. Waiting for GPS fix.")

    def update_track(self, data):
        n = data.n
        lat, lng, alt, vz, sats, states = (
            data.lat[:n], data.lng[:n], data.alt[:n], data.vz[:n], data.sats[:n], data.state[:n]
        )
        valid = (
            np.isfinite(lat) & np.isfinite(lng)
            & (np.abs(lat) <= 85.05112878) & (np.abs(lng) <= 180)
            & ((lat != 0) | (lng != 0)) & (sats >= MIN_SATS_FOR_FIX)
        )
        ids = np.flatnonzero(valid)
        x = np.where(valid, lng, np.nan)
        y = np.where(valid, lat, np.nan)

        if len(ids) == 0:
            self.status.setText(f"No valid GPS fix — waiting for coordinates and >= {MIN_SATS_FOR_FIX} satellites.")
            return x, y

        i = int(ids[-1])
        cur_lat, cur_lng, cur_alt, cur_vz = float(lat[i]), float(lng[i]), float(alt[i]), float(vz[i])
        cur_sats = int(sats[i])
        cur_state = STATE_NAMES.get(int(states[i]), str(int(states[i])))
        self.last_valid_fix = (cur_lat, cur_lng, cur_alt, cur_sats, float(data.t[i]), cur_state)

        if self.pad_coord is None:
            first_i = int(ids[0])
            self.pad_coord = (float(lat[first_i]), float(lng[first_i]))

        # Track apogee coordinates among valid GPS samples
        max_idx = int(ids[int(np.argmax(alt[ids]))])
        self.apogee_coord = (float(lat[max_idx]), float(lng[max_idx]), float(alt[max_idx]))

        # Estimate landing ellipse & wind drift vector during airborne descent
        self.landing_pred = None
        if cur_alt > 6.0 and cur_vz < -1.2 and len(ids) >= 6:
            j0 = int(ids[max(0, len(ids) - 20)])
            dt_d = float(data.t[i] - data.t[j0])
            if dt_d > 0.25:
                v_lat = float(lat[i] - lat[j0]) / dt_d
                v_lng = float(lng[i] - lng[j0]) / dt_d
                t_rem = min(300.0, max(0.0, (cur_alt - 0.5) / abs(cur_vz)))
                pred_lat = cur_lat + v_lat * t_rem
                pred_lng = cur_lng + v_lng * t_rem
                drift_m = haversine_m(cur_lat, cur_lng, pred_lat, pred_lng)
                radius_m = max(12.0, min(250.0, 0.20 * drift_m + 0.08 * cur_alt))
                self.landing_pred = (round(pred_lat, 7), round(pred_lng, 7), round(radius_m, 1))

        # Compute recovery metrics from Launch Pad to Rocket
        pad_lat, pad_lng = self.pad_coord
        ground_m = haversine_m(pad_lat, pad_lng, cur_lat, cur_lng)
        brg = initial_bearing_deg(pad_lat, pad_lng, cur_lat, cur_lng)
        slant_m = math.hypot(ground_m, max(0.0, cur_alt))
        elev_deg = math.degrees(math.atan2(max(0.0, cur_alt), max(0.1, ground_m)))
        pred_txt = ""
        if self.landing_pred is not None:
            p_dist = haversine_m(pad_lat, pad_lng, self.landing_pred[0], self.landing_pred[1])
            p_brg = initial_bearing_deg(pad_lat, pad_lng, self.landing_pred[0], self.landing_pred[1])
            pred_txt = f"  |  EST. LANDING: {p_dist:,.0f} m @ {p_brg:03.0f}° (±{self.landing_pred[2]:.0f} m)"
        self.recovery_bar.setText(
            f"PAD: {pad_lat:.5f}°, {pad_lng:.5f}°  |  "
            f"DOWNRANGE: {ground_m:,.1f} m  |  "
            f"BEARING: {brg:05.1f}° ({cardinal_dir(brg)})  |  "
            f"SLANT: {slant_m:,.1f} m (Elev {elev_deg:.1f}°){pred_txt}"
        )
        fix_label = "GPS FIX" if valid[-1] else "NO FIX (showing last valid position)"
        self.status.setText(
            f"{fix_label}: {cur_lat:.6f}°, {cur_lng:.6f}°  |  "
            f"Sats: {cur_sats}  |  Alt: {cur_alt:.1f} m  |  Vz: {cur_vz:+.1f} m/s  |  T={data.t[i]:.2f} s"
        )

        # 1. Update Folium/Leaflet web map (if ready)
        if self.web_view is not None and self._web_ready:
            if len(ids) != self._last_pushed_count:
                self._last_pushed_count = len(ids)
                step = max(1, len(ids) // 600)
                sampled_ids = ids[::step]
                if sampled_ids[-1] != ids[-1]:
                    sampled_ids = np.append(sampled_ids, ids[-1])
                pts_list = [[round(float(lat[k]), 7), round(float(lng[k]), 7)] for k in sampled_ids]
                pad_json = json.dumps([round(pad_lat, 7), round(pad_lng, 7)])
                apo_json = json.dumps([round(self.apogee_coord[0], 7), round(self.apogee_coord[1], 7), round(self.apogee_coord[2], 1)])
                land_json = json.dumps(list(self.landing_pred)) if self.landing_pred is not None else "null"
                info_json = json.dumps({"alt": cur_alt, "vz": cur_vz, "sats": cur_sats, "state": cur_state})
                auto_js = "true" if self.fit_chk.isChecked() else "false"
                self._run_js(
                    f"if (window.gsMap) window.gsMap.setTrackPoints({json.dumps(pts_list)}, {pad_json}, {apo_json}, {land_json}, {info_json}, {auto_js});"
                )

        # 2. ALWAYS update Tactical Vector Radar (self.plot)
        if self.plot is not None:
            self.track.setData(x, y, connect="finite")
            self.pad_item.setData([pad_lng], [pad_lat])
            self.current.setData([cur_lng], [cur_lat])
            if self.apogee_coord is not None:
                self.apogee_item.setData([self.apogee_coord[1]], [self.apogee_coord[0]])
            else:
                self.apogee_item.setData([], [])

            # Line of sight from launch pad to current rocket position
            self.los_line.setData([pad_lng, cur_lng], [pad_lat, cur_lat], connect="finite")

            # Landing prediction circle and drift line
            if self.landing_pred is not None:
                plat, plng, prad = self.landing_pred
                self.drift_line.setData([cur_lng, plng], [cur_lat, plat], connect="finite")
                self.landing_item.setData([plng], [plat])
                c_theta = np.linspace(0, 2 * np.pi, 32)
                c_lat = plat + (prad / 111139.0) * np.sin(c_theta)
                c_lng = plng + (prad / (111139.0 * max(0.05, math.cos(math.radians(plat))))) * np.cos(c_theta)
                self.landing_circle_item.setData(c_lng, c_lat, connect="finite")
            else:
                self.drift_line.setData([], [])
                self.landing_item.setData([], [])
                self.landing_circle_item.setData([], [])

            # Rebuild range rings if pad changed
            if getattr(self, "_last_ring_pad", None) != (pad_lat, pad_lng):
                self._last_ring_pad = (pad_lat, pad_lng)
                self._rebuild_radar_rings(pad_lat, pad_lng)

            # Auto-follow on Tactical Radar
            if self.fit_chk.isChecked():
                valid_mask = np.isfinite(x) & np.isfinite(y)
                if np.any(valid_mask):
                    xmin = min(pad_lng, float(np.min(x[valid_mask])))
                    xmax = max(pad_lng, float(np.max(x[valid_mask])))
                    ymin = min(pad_lat, float(np.min(y[valid_mask])))
                    ymax = max(pad_lat, float(np.max(y[valid_mask])))
                    if self.landing_pred is not None:
                        xmin = min(xmin, self.landing_pred[1])
                        xmax = max(xmax, self.landing_pred[1])
                        ymin = min(ymin, self.landing_pred[0])
                        ymax = max(ymax, self.landing_pred[0])
                    pad_x = max((xmax - xmin) * 0.15, 0.001)
                    pad_y = max((ymax - ymin) * 0.15, 0.001)
                    self.plot.setRange(xRange=(xmin - pad_x, xmax + pad_x), yRange=(ymin - pad_y, ymax + pad_y), padding=0)

        return x, y


def make_plot(title, ylabel, color, ymin, ymax, theme="dark"):
    t = THEMES[theme]
    pw = pg.PlotWidget(axisItems={"bottom": AdaptiveAxis("bottom"), "left": AdaptiveAxis("left")})
    pw.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
    pw.setBackground(t["plot_bg"])
    pw.setTitle(title, color=t["plot_fg"], size="14pt")
    pw.showGrid(x=True, y=True, alpha=0.35)
    label_style = {"color": t["plot_fg"], "font-size": "13pt"}
    pw.setLabel("left", ylabel, **label_style)
    pw.setLabel("bottom", "Time (s)", **label_style)

    tick_font = QtGui.QFont()
    tick_font.setPointSize(12)
    tick_font.setBold(True)
    axis_pen = pg.mkPen(t["axis_pen"], width=1.5)
    for name in ("left", "bottom"):
        ax = pw.getAxis(name)
        ax.setTickFont(tick_font)
        ax.setTextPen(pg.mkPen(t["plot_fg"]))
        ax.setPen(axis_pen)
        ax.enableAutoSIPrefix(False)                  # no "(x0.001)" axis scaling
        ax.setStyle(maxTextLevel=0, tickTextOffset=8)  # number only the major ticks
    # top + right axes with no numbers: they just close the box around the plot area
    for name in ("top", "right"):
        pw.showAxis(name, True)
        ax = pw.getAxis(name)
        ax.setPen(axis_pen)
        ax.setStyle(showValues=False, tickLength=0)
    pw.getAxis("top").setHeight(2)
    pw.getAxis("right").setWidth(2)
    pw.getAxis("left").setWidth(96)                   # fixed sizes: plot area never jumps around
    pw.getAxis("bottom").setHeight(58)
    pw.setClipToView(True)
    pw.setYRange(ymin, ymax, padding=0)
    curve = pw.plot(pen=pg.mkPen(color, width=3))
    curve.setDownsampling(auto=True, method="peak")
    pw.disableAutoRange()
    return pw, curve


def boxed(widget):
    """Wrap a widget in a bordered 'card' so each graph sits in its own box."""
    box = QtWidgets.QFrame()
    box.setObjectName("plotBox")
    lay = QtWidgets.QVBoxLayout(box)
    lay.setContentsMargins(6, 6, 6, 6)
    lay.addWidget(widget)
    return box


# --------------------------------------------------------------------------
# Main window
# --------------------------------------------------------------------------
class GroundStation(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.theme_name = "light" if getattr(args, "light", False) else "dark"
        self.setWindowTitle("Rocket Ground Station")
        self.resize(1520, 940)
        self.setMinimumSize(1280, 760)

        self.source = None
        self._quick_lock_until = 0.0
        self.voice = VoiceAnnouncer()
        self.data = SeriesBuffer()
        self.dirty = False
        self.flash = False
        self._reset_session_vars()

        self._build_ui()
        self._set_connected_ui(False)
        self.refresh_ports()

        # Timers: plot refresh, ejection flashing, link health
        self.plot_timer = QtCore.QTimer(self, interval=PLOT_REFRESH_MS, timeout=self.refresh_plots)
        self.plot_timer.start()
        self.flash_timer = QtCore.QTimer(self, interval=350, timeout=self._on_flash)
        self.flash_timer.start()
        self.link_timer = QtCore.QTimer(self, interval=500, timeout=self.update_link_card)
        self.link_timer.start()

        # Graphs get most of the width; the console/monitor tabs live in a column on the right.
        QtCore.QTimer.singleShot(50, self._square_split)

        # CLI shortcuts
        if getattr(args, "replay", None):
            QtCore.QTimer.singleShot(0, lambda: self.start_replay(Path(args.replay)))
        elif args.demo:
            self.port_combo.setCurrentIndex(self.port_combo.findData(DEMO_KEY))
            QtCore.QTimer.singleShot(0, self.connect_source)
        elif args.port:
            i = self.port_combo.findData(args.port)
            if i < 0:
                self.port_combo.addItem(args.port, args.port)
                i = self.port_combo.count() - 1
            self.port_combo.setCurrentIndex(i)
            QtCore.QTimer.singleShot(0, self.connect_source)

    # ---------------- UI construction ----------------
    def _build_ui(self):
        t = THEMES[self.theme_name]
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 8)
        root.setSpacing(10)

        # --- top bar: serial controls + replay + voice + theme switch ---
        bar = QtWidgets.QFrame()
        bar.setObjectName("topbar")
        h = QtWidgets.QHBoxLayout(bar)
        h.setContentsMargins(14, 8, 14, 8)

        # App logo + title
        base_dir = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
        logo_png = base_dir / "logo.png"
        if not logo_png.is_file():
            logo_png = Path(__file__).resolve().parent / "logo.png"
        if logo_png.is_file():
            self.logo_label = QtWidgets.QLabel()
            pix = QtGui.QPixmap(str(logo_png)).scaled(
                32, 32,
                QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                QtCore.Qt.TransformationMode.SmoothTransformation
            )
            self.logo_label.setPixmap(pix)
            self.logo_label.setToolTip("Rocket Ground Station")
            h.addWidget(self.logo_label)

        title = QtWidgets.QLabel("ROCKET GROUND STATION")
        title.setObjectName("appTitle")
        h.addWidget(title)
        h.addStretch(1)

        h.addWidget(QtWidgets.QLabel("Port"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(260)
        h.addWidget(self.port_combo)
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        h.addWidget(self.refresh_btn)

        h.addWidget(QtWidgets.QLabel("Baud"))
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems(BAUD_CHOICES)
        self.baud_combo.setCurrentText(str(self.args.baud))
        self.baud_combo.setFixedWidth(92)
        h.addWidget(self.baud_combo)

        self.connect_btn = QtWidgets.QPushButton("CONNECT")
        self.connect_btn.setObjectName("connectBtn")
        self.connect_btn.setProperty("connected", False)
        self.connect_btn.clicked.connect(self.toggle_connection)
        h.addWidget(self.connect_btn)

        self.replay_btn = QtWidgets.QPushButton("Replay CSV...")
        self.replay_btn.setToolTip("Open and replay a recorded CSV telemetry flight log")
        self.replay_btn.clicked.connect(self.pick_replay_csv)
        h.addWidget(self.replay_btn)

        self.replay_speed_combo = QtWidgets.QComboBox()
        self.replay_speed_combo.setToolTip("CSV Replay playback speed")
        for label, spd in (("1x", 1.0), ("2x", 2.0), ("5x", 5.0), ("10x", 10.0)):
            self.replay_speed_combo.addItem(label, spd)
        self.replay_speed_combo.currentIndexChanged.connect(self._on_replay_speed_changed)
        h.addWidget(self.replay_speed_combo)

        self.reset_btn = QtWidgets.QPushButton("Reset View")
        self.reset_btn.clicked.connect(self.reset_session)
        h.addWidget(self.reset_btn)

        self.voice_chk = QtWidgets.QCheckBox("🔊 Voice")
        self.voice_chk.setChecked(True)
        self.voice_chk.setToolTip("Speak voice callouts for liftoff, apogee, parachute deployment, and landing")
        self.voice_chk.toggled.connect(lambda on: setattr(self.voice, "enabled", bool(on)))
        h.addWidget(self.voice_chk)

        self.theme_btn = QtWidgets.QPushButton()
        self.theme_btn.setToolTip("Switch between Dark Aerospace mode and Sunlight-Readable Light mode")
        self.theme_btn.clicked.connect(self.toggle_theme)
        self._update_theme_btn_label()
        h.addWidget(self.theme_btn)
        root.addWidget(bar)

        # --- status cards (2 rows x 7) ---
        self.card_go = Card("Launch Readiness")
        self.card_state = Card("Rocket State")
        self.card_eject = Card("Ejection Charge")
        self.card_chute = Card("Parachute")
        self.card_alt = Card("Altitude (fused)")
        self.card_vz = Card("Vertical Speed")
        self.card_max = Card("Max Altitude")
        self.card_cont = Card("Continuity")
        self.card_fix = Card("GPS Fix")
        self.card_sats = Card("Satellites")
        self.card_lat = Card("Latitude")
        self.card_lng = Card("Longitude")
        self.card_acc = Card("Acceleration")
        self.card_link = Card("Telemetry Link")
        self.all_cards = [
            self.card_go, self.card_state, self.card_eject, self.card_chute,
            self.card_alt, self.card_vz, self.card_max,
            self.card_cont, self.card_fix, self.card_sats, self.card_lat,
            self.card_lng, self.card_acc, self.card_link,
        ]
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(10)
        for i, c in enumerate(self.all_cards):
            grid.addWidget(c, i // 7, i % 7)
        for col in range(7):
            grid.setColumnStretch(col, 1)
        root.addLayout(grid)
        self.card_go.set("NO-GO", "Offline", "normal")
        self.card_eject.set("STANDBY")
        self.card_chute.set("STOWED")
        self.card_link.set("OFFLINE")

        # --- plots + console in a splitter ---
        split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)

        plots = QtWidgets.QWidget()
        pv = QtWidgets.QVBoxLayout(plots)
        pv.setContentsMargins(0, 0, 0, 0)
        ctl = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel("LIVE TELEMETRY & RECOVERY MAP")
        lbl.setObjectName("sectionTitle")
        ctl.addWidget(lbl)
        ctl.addStretch(1)
        self.autoscroll = QtWidgets.QCheckBox("Fit full flight")
        self.autoscroll.setChecked(True)
        self.autoscroll.setToolTip("On: expand axes to keep the full flight visible.\nMouse zoom/pan pauses fitting; recheck to resume.")
        self.autoscroll.toggled.connect(self._on_view_changed)
        ctl.addWidget(self.autoscroll)
        pv.addLayout(ctl)

        pg.setConfigOptions(antialias=True, foreground=t["plot_fg"])
        self.plot_alt, self.curve_alt = make_plot(
            "Altitude (AltFused)", "Altitude (m)", t["alt_curve"], 0, 10, self.theme_name
        )
        self.plot_acc, self.curve_acc = make_plot(
            "Acceleration (AccelTotal)", "Accel (m/s²)", t["acc_curve"], 0, 10, self.theme_name
        )
        self.ystab_alt = StableYRange(min_span=10.0)     # at least 10 m tall
        self.ystab_acc = StableYRange(min_span=4.0)      # at least 4 m/s² tall
        self.plot_acc.setXLink(self.plot_alt)            # drive both from the top plot
        self.max_line = pg.InfiniteLine(
            angle=0,
            movable=False,
            pen=pg.mkPen(t["max_line"], width=1, style=QtCore.Qt.PenStyle.DashLine),
        )
        self.max_line.setVisible(False)
        self.plot_alt.addItem(self.max_line)

        graph_split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        stack = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        stack.addWidget(boxed(self.plot_alt))
        stack.addWidget(boxed(self.plot_acc))
        graph_split.addWidget(stack)
        self.gps_map = GPSMap(theme=self.theme_name)
        graph_split.addWidget(boxed(self.gps_map))
        graph_split.setSizes([580, 500])
        pv.addWidget(graph_split, 1)

        self.inspect_label = QtWidgets.QLabel()
        self.inspect_label.setWordWrap(True)
        pv.addWidget(self.inspect_label)
        self.inspectors = []
        for plot, field in ((self.plot_alt, "alt"), (self.plot_acc, "acc")):
            plot.setMinimumSize(280, 200)
            plot.getViewBox().sigRangeChangedManually.connect(lambda *_: self.autoscroll.setChecked(False))
            self.inspectors.append(PointInspector(
                plot,
                lambda f=field: (self.data.t[:self.data.n], getattr(self.data, f)[:self.data.n], np.arange(self.data.n)),
                self._describe_sample,
                self.inspect_label,
            ))
        self.map_xy = (np.array([]), np.array([]))
        if self.gps_map.plot is not None:
            self.inspectors.append(PointInspector(
                self.gps_map.plot,
                lambda: (*self.map_xy, np.arange(len(self.map_xy[0]))),
                self._describe_sample,
                self.inspect_label,
            ))
        self.state_marks = []
        self.state_events = []
        for plot in (self.plot_alt, self.plot_acc):
            plot.getViewBox().sigRangeChanged.connect(self._layout_state_labels)
        self.gps_map.fit_chk.toggled.connect(self._mark_dirty)
        self.inspectors[0].clear()
        split.addWidget(plots)

        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(5000)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self.console, "Event Log")
        self.tabs.addTab(self._build_monitor_tab(), "Serial Monitor & Commands")
        split.addWidget(self.tabs)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        split.setCollapsible(0, False)
        self.tabs.setMinimumWidth(320)
        self.splitter = split
        root.addWidget(split, 1)

        self.setStatusBar(QtWidgets.QStatusBar())
        self.statusBar().showMessage("Disconnected")

    # ---------------- Theme switching ----------------
    def _update_theme_btn_label(self):
        if self.theme_name == "dark":
            self.theme_btn.setText("☀ Light Mode")
        else:
            self.theme_btn.setText("🌙 Dark Mode")

    def toggle_theme(self):
        self.theme_name = "light" if self.theme_name == "dark" else "dark"
        self.apply_theme()

    def apply_theme(self):
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.setStyleSheet(DARK_STYLESHEET if self.theme_name == "dark" else LIGHT_STYLESHEET)
        self._update_theme_btn_label()
        t = THEMES[self.theme_name]

        # Update pyqtgraph plots
        axis_pen = pg.mkPen(t["axis_pen"], width=1.5)
        label_style = {"color": t["plot_fg"], "font-size": "13pt"}
        for plot, curve, ylabel, c_key in (
            (self.plot_alt, self.curve_alt, "Altitude (m)", "alt_curve"),
            (self.plot_acc, self.curve_acc, "Accel (m/s²)", "acc_curve"),
        ):
            plot.setBackground(t["plot_bg"])
            curve.setPen(pg.mkPen(t[c_key], width=3))
            plot.setLabel("left", ylabel, **label_style)
            plot.setLabel("bottom", "Time (s)", **label_style)
            for name in ("left", "bottom"):
                ax = plot.getAxis(name)
                ax.setTextPen(pg.mkPen(t["plot_fg"]))
                ax.setPen(axis_pen)
            for name in ("top", "right"):
                plot.getAxis(name).setPen(axis_pen)

        if self.data.n > 0:
            self.plot_alt.setTitle(
                f"Altitude (AltFused)     {self.data.alt[self.data.n - 1]:.1f} m",
                color=t["plot_fg"], size="14pt"
            )
            self.plot_acc.setTitle(
                f"Acceleration (AccelTotal)     {self.data.acc[self.data.n - 1]:.2f} m/s²",
                color=t["plot_fg"], size="14pt"
            )
        else:
            self.plot_alt.setTitle("Altitude (AltFused)", color=t["plot_fg"], size="14pt")
            self.plot_acc.setTitle("Acceleration (AccelTotal)", color=t["plot_fg"], size="14pt")

        self.max_line.setPen(pg.mkPen(t["max_line"], width=1, style=QtCore.Qt.PenStyle.DashLine))
        for _plot, line in self.state_marks:
            line.setPen(pg.mkPen(t["state_line"], width=1, style=QtCore.Qt.PenStyle.DashLine))
            line.label.setColor(t["state_label_fg"])
            line.label.fill = pg.mkBrush(t["state_label_bg"])

        # Update Folium/Leaflet map theme
        self.gps_map.apply_theme(self.theme_name)

        # Re-polish dynamic property widgets
        for w in (*self.all_cards, self.connect_btn):
            w.style().unpolish(w)
            w.style().polish(w)

    def _square_split(self):
        """Size the left/right split so the graphs and map have generous space."""
        w = self.splitter.width()
        self.splitter.setSizes([max(700, w - 350), 350])

    def _describe_sample(self, i):
        d = self.data
        state = STATE_NAMES.get(int(d.state[i]), str(int(d.state[i])))
        return (f"T={d.t[i]:.3f} s | Alt={d.alt[i]:.3f} m | Vz={d.vz[i]:+.2f} m/s | Acc={d.acc[i]:.3f} m/s² | "
                f"{state} | GPS={d.lat[i]:.7f}°, {d.lng[i]:.7f}° | Sats={int(d.sats[i])}")

    def _add_state_mark(self, t_sec, state):
        t_cfg = THEMES[self.theme_name]
        name = STATE_NAMES.get(state, f"UNKNOWN {state}")
        self.state_events.append((t_sec, name))
        for plot in (self.plot_alt, self.plot_acc):
            line = pg.InfiniteLine(
                pos=t_sec,
                angle=90,
                movable=False,
                pen=pg.mkPen(t_cfg["state_line"], width=1, style=QtCore.Qt.PenStyle.DashLine),
                label=name,
                labelOpts={
                    "position": 0.95 - 0.13 * ((len(self.state_events) - 1) % 4),
                    "color": t_cfg["state_label_fg"],
                    "fill": t_cfg["state_label_bg"],
                    "rotateAxis": (1, 0),
                },
            )
            line.setToolTip(f"{name}: first received at T={t_sec:.3f} s")
            plot.addItem(line, ignoreBounds=True)
            self.state_marks.append((plot, line))

    def _clear_plot_history(self):
        for plot, line in self.state_marks:
            plot.removeItem(line)
        self.state_marks.clear()
        self.state_events.clear()
        for inspector in self.inspectors:
            inspector.clear()
        self.map_xy = (np.array([]), np.array([]))
        self.gps_map.clear_map()
        self._fit_right = None

    def _build_monitor_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(6, 6, 6, 6)

        self.monitor = QtWidgets.QPlainTextEdit()
        self.monitor.setReadOnly(True)
        self.monitor.setMaximumBlockCount(20000)
        v.addWidget(self.monitor, 1)

        opts = QtWidgets.QHBoxLayout()
        self.mon_pause = QtWidgets.QCheckBox("Pause")
        self.mon_ts = QtWidgets.QCheckBox("Timestamps")
        self.mon_ts.setChecked(True)
        self.mon_show_tel = QtWidgets.QCheckBox("Telemetry rows")
        self.mon_show_tel.setChecked(True)
        clear = QtWidgets.QPushButton("Clear")
        clear.clicked.connect(self.monitor.clear)
        for x in (self.mon_pause, self.mon_ts, self.mon_show_tel):
            opts.addWidget(x)
        opts.addStretch(1)
        opts.addWidget(clear)
        v.addLayout(opts)

        quick = QtWidgets.QGridLayout()
        quick.addWidget(QtWidgets.QLabel("Quick commands:"), 0, 0, 1, 3)
        self.quick_buttons = []
        for label, cmd, kind in QUICK_COMMANDS:
            b = QtWidgets.QPushButton(label)
            if kind == "fire":
                b.setObjectName("quickFire")
            elif kind == "pad_test":
                b.setObjectName("quickPadTest")
            b.setToolTip(f"Sends: {cmd}")
            b.clicked.connect(lambda _=False, l=label, c=cmd, k=kind: self.quick_command(l, c, k))
            idx = len(self.quick_buttons)
            quick.addWidget(b, 1 + idx // 3, idx % 3)          # 3 buttons per row
            self.quick_buttons.append((b, cmd, kind))
        rows_used = 1 + (len(QUICK_COMMANDS) + 2) // 3
        self.override_chk = QtWidgets.QCheckBox(f"Override pad lockout ({OVERRIDE_SECONDS} s)")
        self.override_chk.setObjectName("overrideChk")
        self.override_chk.setMinimumWidth(230)
        self.override_chk.setToolTip("Lets pad-locked test buttons (TEST BARO / TEST DROGUE / TEST MAIN) "
                                     "work in ANY rocket state, even with no telemetry. Switches off automatically.")
        self.override_chk.toggled.connect(self.on_override_toggled)
        quick.addWidget(self.override_chk, rows_used, 0, 1, 3)
        v.addLayout(quick)

        row = QtWidgets.QHBoxLayout()
        self.send_edit = QtWidgets.QLineEdit()
        self.send_edit.setPlaceholderText("Type a command and press Enter (e.g. test_baro, REBOOT)")
        self.send_edit.returnPressed.connect(self.send_command)
        self.eol_combo = QtWidgets.QComboBox()
        for label, val in (("LF  \\n", "\n"), ("None", ""),
                           ("CR  \\r", "\r"), ("CR+LF", "\r\n")):
            self.eol_combo.addItem(label, val)
        self.send_btn = QtWidgets.QPushButton("Send")
        self.send_btn.clicked.connect(self.send_command)
        row.addWidget(self.send_edit, 1)
        row.addWidget(self.eol_combo)
        row.addWidget(self.send_btn)
        v.addLayout(row)
        return w

    def on_raw_line(self, direction, text):
        """Serial Monitor: every line received (RX) or sent (TX)."""
        if self.mon_pause.isChecked():
            return
        if direction == "RX" and not self.mon_show_tel.isChecked() and text.startswith("PKT "):
            return
        t = THEMES[self.theme_name]
        ts = (datetime.now().strftime("%H:%M:%S.%f")[:-3] + "  ") if self.mon_ts.isChecked() else ""
        if direction == "TX":
            body = f'<span style="color:{t["log_tx"]}">TX&gt; {html.escape(text)}</span>'
        else:
            body = f'<span style="color:{t["log_rx"]}">{html.escape(text)}</span>'
        self.monitor.appendHtml(f'<span style="color:{t["log_ts"]}">{ts}</span>{body}')

    def send_command(self):
        if self.source is None:
            self.log("ERROR: not connected - cannot send")
            return
        text = self.send_edit.text()
        if not text:
            return
        self.source.send_text(text, self.eol_combo.currentData())
        self.send_edit.clear()

    def _confirm_reboot(self):
        msg = "Send REBOOT command to the flight computer now?"
        if self.cur_state in (2, 3, 4, 5, 6, 7, 8, 9):
            msg = (f"The rocket is reporting an ACTIVE FLIGHT state "
                   f"({STATE_NAMES.get(self.cur_state, self.cur_state)}).\n\n"
                   "Rebooting now resets the flight state machine and can disarm recovery.\n"
                   "Reboot anyway?")
        B = QtWidgets.QMessageBox.StandardButton
        return QtWidgets.QMessageBox.question(self, "Reboot flight computer", msg,
                                              B.Yes | B.No, B.No) == B.Yes

    def quick_command(self, label, cmd, kind):
        """One-click text command (same as typing it in the send box + Enter)."""
        if self.source is None:
            return
        B = QtWidgets.QMessageBox.StandardButton
        if kind == "fire":
            reason = self._pad_lock_reason()
            if reason:
                self.log(f"[GUI] {label} blocked: {reason}")
                return
            ans = QtWidgets.QMessageBox.warning(
                self, f"{label} (TEST)",
                f"Send {cmd}?\n\nThis fires the ejection channel IMMEDIATELY.\n"
                "Continue only if the charge is removed or a test load is fitted, and everyone is clear."
                + self._override_note(),
                B.Yes | B.No, B.No)
            if ans != B.Yes or self._pad_lock_reason():     # re-check after the dialog
                return
            self._quick_lock_until = time.monotonic() + 3.0   # block double-clicks
        elif kind == "pad_test":
            reason = self._pad_lock_reason()
            if reason:
                self.log(f"[GUI] {label} blocked: {reason}")
                return
            ans = QtWidgets.QMessageBox.question(
                self, f"{label}",
                f"Run barometric test (sends '{cmd}')?\n\n"
                "Unlocks only while the rocket is on the PAD (or with lockout override)."
                + self._override_note(),
                B.Yes | B.No, B.No)
            if ans != B.Yes or self._pad_lock_reason():
                return
        elif kind == "confirm":
            if QtWidgets.QMessageBox.question(
                    self, label, f"Send {cmd}?\n\nThis cannot be undone.",
                    B.Yes | B.No, B.No) != B.Yes:
                return
        elif kind == "reboot":
            if not self._confirm_reboot():
                return
        self.source.send_text(cmd, "\n")
        self.log(f"[GUI] Sent command: {cmd}")

    # ---------------- Pad-only lockout & override ----------------
    def _pad_lock_reason(self):
        """None if pad-locked test commands may be used right now, else why they are locked."""
        if self.source is None:
            return "not connected"
        if self._override_active():
            return None
        if self.cur_state is None or self.last_rx is None \
                or time.monotonic() - self.last_rx > STALE_LINK_SECONDS:
            return "no live telemetry (the rocket state must be visible)"
        if self.cur_state not in TEST_ALLOWED_STATES:
            return f"rocket reports {STATE_NAMES.get(self.cur_state, self.cur_state)} (test only on PAD)"
        return None

    def _override_active(self):
        return time.monotonic() < self.lock_override_until

    def _override_note(self):
        """Extra warning line for test dialogs while the lockout is overridden."""
        if not self._override_active():
            return ""
        st = STATE_NAMES.get(self.cur_state, self.cur_state) if self.cur_state is not None else "unknown"
        return f"\n\nLOCKOUT OVERRIDDEN - the rocket currently reports: {st}."

    def on_override_toggled(self, checked):
        if not checked:
            self.lock_override_until = 0.0
            self.log("[GUI] Lockout override OFF")
            self._update_quick_locks()
            return
        B = QtWidgets.QMessageBox.StandardButton
        ans = QtWidgets.QMessageBox.warning(
            self, "Override pad lockout",
            f"Override the pad-only lockout for {OVERRIDE_SECONDS} seconds?\n\n"
            "Pad-locked test buttons (TEST BARO, TEST DROGUE, TEST MAIN) will work in ANY rocket state, "
            "even without live telemetry.\n"
            "Use this only on the bench with charges removed or a test load fitted, "
            "and everyone clear of the rocket.",
            B.Yes | B.No, B.No)
        if ans == B.Yes and self.source is not None:
            self.lock_override_until = time.monotonic() + OVERRIDE_SECONDS
            self.log(f"[GUI] WARNING: lockout override ON for {OVERRIDE_SECONDS} s")
        else:
            self._clear_override(log=False)
        self._update_quick_locks()

    def _clear_override(self, log=True):
        was = self._override_active() or self.override_chk.isChecked()
        self.lock_override_until = 0.0
        self.override_chk.blockSignals(True)
        self.override_chk.setChecked(False)
        self.override_chk.setText(f"Override pad lockout ({OVERRIDE_SECONDS} s)")
        self.override_chk.blockSignals(False)
        if log and was:
            self.log("[GUI] Lockout override OFF")

    def on_bad_packet(self):
        self.bad_packets += 1

    def on_text_message(self, text):
        self.log(text)
        if "LANDING DETECTED" in text.upper() and not self._landing_announced:
            self._landing_announced = True
            self.voice.say(f"Touchdown detected. Max altitude {self.max_alt:.0f} meters.")

    def _update_quick_locks(self):
        if self.override_chk.isChecked():
            if self._override_active():
                left = math.ceil(self.lock_override_until - time.monotonic())
                self.override_chk.setText(f"OVERRIDE ACTIVE ({left} s)")
            else:
                self._clear_override()
                self.log("[GUI] Lockout override expired")
        reason = self._pad_lock_reason()
        unlocked = reason is None and time.monotonic() >= self._quick_lock_until
        override_tag = " (lockout overridden)" if self._override_active() else ""
        for b, cmd, kind in self.quick_buttons:
            if kind in ("fire", "pad_test"):
                b.setEnabled(unlocked)
                if reason:
                    b.setToolTip(f"Locked: {reason}")
                else:
                    b.setToolTip(f"Sends: {cmd}{override_tag}")

    # ---------------- helpers ----------------
    def _reset_session_vars(self):
        self.lock_override_until = 0.0
        self.max_acc = 0.0
        self.max_vz = 0.0
        self.cur_vz = 0.0
        self.bad_packets = 0
        self.max_alt = 0.0
        self.max_alt_t = 0.0
        self.cur_state = None
        self.last_cont = None
        self.last_sats = 0
        self.last_has_fix = False
        self._landing_announced = False
        self.packets = 0
        self.last_rx = None

    def _mark_dirty(self, *_):
        self.dirty = True

    def _on_view_changed(self, *_):
        """Window length / auto-scroll changed: re-fit axes from scratch."""
        self.ystab_alt.reset()
        self.ystab_acc.reset()
        self._fit_right = None
        self.dirty = True

    def log(self, text: str):
        t = THEMES[self.theme_name]
        ts = datetime.now().strftime("%H:%M:%S")
        up = text.upper()
        if any(k in up for k in ("FAIL", "ERROR", "ABORT", "FAULT")):
            color = t["log_err"]
        elif "WARN" in up:
            color = t["log_warn"]
        elif "TRANSITION" in up or "BARO_TEST" in up or "REPLAY" in up:
            color = t["log_trans"]
        elif any(k in up for k in ("NOMINAL", " OK", "READY", "PASS")):
            color = t["log_ok"]
        else:
            color = t["log_rx"]
        self.console.appendHtml(
            f'<span style="color:{t["log_ts"]}">[{ts}]</span> '
            f'<span style="color:{color}">{html.escape(text)}</span>')

    # ---------------- ports / connection / replay ----------------
    def refresh_ports(self):
        prev = self.port_combo.currentData()
        self.port_combo.clear()
        for p in sorted(list_ports.comports(), key=lambda p: p.device):
            self.port_combo.addItem(f"{p.device} — {p.description}", p.device)
        self.port_combo.addItem("Simulated flight (DEMO, no hardware)", DEMO_KEY)
        i = self.port_combo.findData(prev) if prev else -1
        if i >= 0:
            self.port_combo.setCurrentIndex(i)

    def _csv_path(self) -> Path:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if self.args.csv:
            p = Path(self.args.csv)
            if p.is_dir() or str(self.args.csv).endswith(("/", "\\")):
                return p / f"telemetry_{stamp}.csv"
            return p.parent / f"{p.stem}_{stamp}.csv"
        # Works both as a normal .py script AND when packaged into a PyInstaller .exe
        app_dir = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent
        for base in (app_dir, Path.home()):
            folder = base / "telemetry_logs"
            try:
                folder.mkdir(parents=True, exist_ok=True)
                (folder / ".write_test").touch()
                (folder / ".write_test").unlink()
                return folder / f"telemetry_{stamp}.csv"
            except OSError:
                continue
        return Path.home() / f"telemetry_{stamp}.csv"

    def _on_replay_speed_changed(self):
        if isinstance(self.source, ReplaySource):
            spd = float(self.replay_speed_combo.currentData() or 1.0)
            self.source.set_speed(spd)
            self.log(f"[REPLAY] Playback speed set to {spd:g}x")

    def pick_replay_csv(self):
        app_dir = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent
        default_dir = app_dir / "telemetry_logs"
        start_dir = str(default_dir if default_dir.is_dir() else app_dir)
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Flight Telemetry CSV to Replay", start_dir, "CSV files (*.csv);;All files (*)"
        )
        if path:
            self.start_replay(Path(path))

    def start_replay(self, csv_path: Path):
        if self.source is not None:
            self.disconnect_source()
        self.reset_session()
        spd = float(self.replay_speed_combo.currentData() or 1.0)
        src = ReplaySource(csv_path, speed=spd)
        src.telemetry.connect(self.on_telemetry)
        src.message.connect(self.on_text_message)
        src.raw_line.connect(self.on_raw_line)
        src.bad_packet.connect(self.on_bad_packet)
        src.failed.connect(self.on_source_failed)
        src.finished.connect(self.on_source_finished)
        self.source = src
        self._logger_path = csv_path
        src.start()
        self._set_connected_ui(True)
        self.statusBar().showMessage(f"Replaying CSV: {csv_path} ({spd:g}x)")

    def toggle_connection(self):
        if self.source is None:
            self.connect_source()
        else:
            self.disconnect_source()

    def connect_source(self):
        if self.source is not None:
            return
        port = self.port_combo.currentData()
        if not port:
            self.log("ERROR: no serial port selected (press Refresh, or plug in the HC-12 receiver)")
            return
        try:
            baud = int(self.baud_combo.currentText().strip())
        except ValueError:
            self.log("ERROR: baud rate must be an integer")
            return
        try:
            logger = CsvLogger(self._csv_path())
        except OSError as exc:
            self.log(f"ERROR: cannot create log file: {exc}")
            return

        self.reset_session()
        src = DemoSource(logger) if port == DEMO_KEY else SerialSource(port, baud, logger)
        src.telemetry.connect(self.on_telemetry)
        src.message.connect(self.on_text_message)
        src.raw_line.connect(self.on_raw_line)
        src.bad_packet.connect(self.on_bad_packet)
        src.failed.connect(self.on_source_failed)
        src.finished.connect(self.on_source_finished)
        self.source = src
        self._logger_path = logger.path
        src.start()
        self._set_connected_ui(True)
        label = "DEMO" if port == DEMO_KEY else f"{port} @ {baud}"
        self.log(f"[GUI] Connecting: {label}")
        self.log(f"[GUI] Logging every line to {logger.path}")
        self.statusBar().showMessage(f"Connected: {label}   |   logging to {logger.path}")

    def disconnect_source(self):
        src, self.source = self.source, None
        if src is None:
            return
        self._clear_override()
        src.stop()
        src.wait(2000)
        self._set_connected_ui(False)
        self.log(f"[GUI] Disconnected. Log: {getattr(self, '_logger_path', '')}")
        self.statusBar().showMessage("Disconnected")
        self.card_link.set("OFFLINE", "", "normal")
        self.card_go.set("NO-GO", "Offline", "normal")

    def on_source_failed(self, text):
        self.log(f"ERROR: {text}")

    def on_source_finished(self):
        # Thread ended on its own (port error or replay finished). Tidy up the UI.
        if self.source is not None and not self.source.isRunning():
            self.disconnect_source()

    def _set_connected_ui(self, connected: bool):
        is_replay = isinstance(self.source, ReplaySource)
        self.connect_btn.setText("STOP REPLAY" if (connected and is_replay) else "DISCONNECT" if connected else "CONNECT")
        self.connect_btn.setProperty("connected", connected)
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)
        for w in (self.port_combo, self.baud_combo, self.refresh_btn, self.replay_btn):
            w.setEnabled(not connected)
        for w in (self.send_edit, self.send_btn, self.eol_combo):
            w.setEnabled(connected and not is_replay)
        for b, _cmd, kind in self.quick_buttons:    # pad-locked buttons are managed by _update_quick_locks()
            b.setEnabled(connected and not is_replay and kind not in ("fire", "pad_test"))
        self._update_quick_locks()

    # ---------------- Pre-flight Go/No-Go checklist evaluation ----------------
    def _update_go_nogo_card(self):
        if self.source is None or self.last_rx is None:
            self.card_go.set("NO-GO", "No telemetry link", "normal")
            return
        age = time.monotonic() - self.last_rx
        if age > STALE_LINK_SECONDS:
            self.card_go.set("NO-GO", f"Link stale ({age:.0f} s)", "alert")
            return
        if self.cur_state == 0:
            self.card_go.set("NO-GO", "FC reports ERROR", "alert")
            return
        if self.cur_state not in (1, 2):
            st_name = STATE_NAMES.get(self.cur_state, str(self.cur_state))
            self.card_go.set("IN FLIGHT", st_name, "ok")
            return
        reasons = []
        if self.last_cont is not None:
            drogues, mains = divmod(int(self.last_cont), 10)
            if drogues <= 0 or mains <= 0:
                reasons.append(f"Cont D{drogues}/M{mains}")
        else:
            reasons.append("No cont data")
        if not self.last_has_fix:
            reasons.append(f"No GPS fix ({self.last_sats} sats)")
        if reasons:
            self.card_go.set("NO-GO", " | ".join(reasons), "warn")
        elif self.last_sats < 6:
            self.card_go.set("GO (MARGINAL)", f"Pad OK, {self.last_sats} GPS sats", "warn")
        else:
            self.card_go.set("GO FOR LAUNCH", f"All checks nominal ({self.last_sats} sats)", "ok")

    def _announce_state_transition(self, state: int, alt: float):
        phrases = {
            0: "Warning. Flight computer error.",
            2: "Rocket armed.",
            3: "Liftoff. Boost phase.",
            4: "Burnout. Coast phase.",
            6: f"Apogee, {alt:.0f} meters. Firing drogue.",
            7: "Drogue parachute deployed.",
            8: f"Firing main parachute at {alt:.0f} meters.",
            9: "Main parachute deployed.",
        }
        phrase = phrases.get(state)
        if phrase:
            self.voice.say(phrase)

    # ---------------- telemetry handling ----------------
    def on_telemetry(self, row):
        t_ms, state, acc, alt, lat, lng, sats, cont = row
        t = t_ms / 1000.0

        # Flight computer rebooted / time went backwards -> start plots fresh
        if self.data.n and t < self.data.t[self.data.n - 1] - 1.0:
            self.log("WARN: Time_ms went backwards (flight computer reset?) - clearing plots")
            self.data.clear()
            self._clear_plot_history()
            self.cur_state = None
            self.max_alt = self.max_alt_t = self.max_acc = self.max_vz = self.cur_vz = 0.0
            self._on_view_changed()

        # Compute smoothed vertical velocity Vz (m/s)
        if self.data.n > 0:
            k0 = max(0, self.data.n - 4)
            dt_v = t - float(self.data.t[k0])
            if dt_v > 0.01:
                raw_vz = (alt - float(self.data.alt[k0])) / dt_v
                self.cur_vz = 0.65 * self.cur_vz + 0.35 * raw_vz
        else:
            self.cur_vz = 0.0
        self.max_vz = max(self.max_vz, self.cur_vz)

        self.data.append(t, alt, acc, self.cur_vz, lat, lng, state, sats)
        self.dirty = True
        self.packets += 1
        self.last_rx = time.monotonic()

        if alt > self.max_alt:
            self.max_alt, self.max_alt_t = alt, t

        if state != self.cur_state:
            old = STATE_NAMES.get(self.cur_state, "-") if self.cur_state is not None else "-"
            self.log(f"[GUI] State: {old} -> {STATE_NAMES.get(state, f'UNKNOWN ({state})')}")
            self._add_state_mark(t, state)
            if self.cur_state is not None:
                self._announce_state_transition(state, self.max_alt if state in (6, 7) else alt)
            self.cur_state = state
        self._update_state_cards(state)
        self._update_quick_locks()

        self.max_acc = max(self.max_acc, acc)
        self.card_alt.set(f"{alt:,.1f} m", "Kalman-fused")
        self.card_acc.set(f"{acc:.1f} m/s²", f"peak {self.max_acc:.1f}")

        # Vertical speed, Mach, Ballistic Apogee Prediction & Parachute Descent Rate Health
        mach = abs(self.cur_vz) / 343.0
        vz_kind = "normal"
        if state in (3, 4) and self.cur_vz > 2.0:
            # Ballistic apogee prediction with aerodynamic drag factor
            pred_apo = alt + (self.cur_vz ** 2) / (2.0 * (9.81 + 0.002 * self.cur_vz ** 2))
            vz_sub = f"Est. Apogee ~{pred_apo:,.0f} m (M {mach:.2f})"
        elif state == 7:  # DROGUE DESCENT
            fast = self.cur_vz < -32.0
            vz_sub = "FAST DROGUE!" if fast else f"Drogue nominal (peak +{self.max_vz:.0f})"
            vz_kind = "warn" if fast else "ok"
        elif state == 9:  # MAIN DESCENT
            fast = self.cur_vz < -11.0
            vz_sub = "FAST MAIN DESCENT!" if fast else f"Main nominal (peak +{self.max_vz:.0f})"
            vz_kind = "alert" if fast else "ok"
        else:
            vz_sub = f"peak +{self.max_vz:.1f} m/s (M {mach:.2f})"
        self.card_vz.set(f"{self.cur_vz:+.1f} m/s", vz_sub, vz_kind)

        self.last_cont = cont
        if cont is not None:                      # tens digit = drogues, ones digit = mains
            drogues, mains = divmod(int(cont), 10)
            ok = drogues > 0 and mains > 0
            self.card_cont.set(f"D {drogues}  |  M {mains}",
                               "e-match continuity" if ok else "CHECK CONTINUITY", "ok" if ok else "warn")
        self.card_max.set(f"{self.max_alt:,.1f} m", f"at T = {self.max_alt_t:.2f} s")
        have_pos = math.isfinite(lat) and math.isfinite(lng) and not (lat == 0.0 and lng == 0.0)
        has_fix = have_pos and sats >= MIN_SATS_FOR_FIX
        self.last_has_fix = has_fix
        self.last_sats = int(sats)
        self.card_fix.set("FIX" if has_fix else "NO FIX", "", "ok" if has_fix else "warn")
        self.card_sats.set(str(sats))
        self.card_lat.set(f"{lat:.6f}°" if have_pos else "---")
        self.card_lng.set(f"{lng:.6f}°" if have_pos else "---")
        self._update_go_nogo_card()

    def _update_state_cards(self, state):
        name = STATE_NAMES.get(state, f"UNKNOWN ({state})")
        if state == 0:
            kind = "alert"                         # ERROR
        elif state in EJECTION_STATES or state in DEPLOYED_STATES:
            kind = "ok" if state in DEPLOYED_STATES else "warn"
        elif state in (2, 3, 4, 5):
            kind = "warn"
        else:
            kind = "normal"
        self.card_state.set(name, f"State code {state}", kind)

        if state in DEPLOYED_STATES:
            self.card_chute.set(f"{DEPLOYED_STATES[state]} DEPLOYED", "Parachute out", "ok")
        else:
            self.card_chute.set("STOWED", "", "normal")
        self._update_eject_card()

    def _update_eject_card(self):
        s = self.cur_state
        if s in EJECTION_STATES:
            self.card_eject.set(f"{EJECTION_STATES[s]} FIRING", "!! EJECTION EVENT !!",
                                "alert" if self.flash else "alertdim")
        elif s in DEPLOYED_STATES:
            self.card_eject.set(f"{DEPLOYED_STATES[s]} FIRED", "", "normal")
        else:
            self.card_eject.set("STANDBY", "", "normal")

    def _on_flash(self):
        self.flash = not self.flash
        if self.cur_state in EJECTION_STATES:
            self._update_eject_card()

    def update_link_card(self):
        self._update_quick_locks()
        if self.source is None:
            return
        if self.last_rx is None:
            self.card_link.set("WAITING", "no telemetry yet", "warn")
            self._update_go_nogo_card()
            return
        age = time.monotonic() - self.last_rx
        stale = age > STALE_LINK_SECONDS
        self.card_link.set(f"LOST {age:.0f} s" if stale else "LINKED",
                           f"{self.packets} ok, {self.bad_packets} bad",
                           "alert" if stale else "ok")
        self._update_go_nogo_card()

    # ---------------- plotting ----------------
    def refresh_plots(self):
        if not self.dirty or self.data.n == 0:
            return
        self.dirty = False
        n = self.data.n
        t = self.data.t[:n]
        self.curve_alt.setData(t, self.data.alt[:n])
        self.curve_acc.setData(t, self.data.acc[:n])
        self.map_xy = self.gps_map.update_track(self.data)
        self.max_line.setVisible(True)
        self.max_line.setPos(self.max_alt)

        fg = THEMES[self.theme_name]["plot_fg"]
        self.plot_alt.setTitle(
            f"Altitude (AltFused)     {self.data.alt[n - 1]:.1f} m   |   Vz: {self.data.vz[n - 1]:+.1f} m/s",
            color=fg, size="14pt"
        )
        self.plot_acc.setTitle(f"Acceleration (AccelTotal)     {self.data.acc[n - 1]:.2f} m/s²",
                               color=fg, size="14pt")

        if not self.autoscroll.isChecked():          # user is panning/zooming manually
            return

        # Keep the first sample visible and grow in readable increments.
        tt = self.data.t[:n]
        lo_x = float(tt[0])
        elapsed = max(10.0, float(tt[-1]) - lo_x)
        step = nice_step(elapsed / 6)
        right = lo_x + math.ceil(elapsed * 1.05 / step) * step
        self._fit_right = max(getattr(self, "_fit_right", None) or right, right)
        self.plot_alt.setXRange(lo_x, self._fit_right, padding=0)
        for plot, ys, stab in ((self.plot_alt, self.data.alt, self.ystab_alt),
                               (self.plot_acc, self.data.acc, self.ystab_acc)):
            res = stab.update(float(ys[:n].min()), float(ys[:n].max()))
            if res:
                plot.setYRange(res[0], res[1], padding=0)
        self._layout_state_labels()

    def _layout_state_labels(self, *_):
        # Sparse state labels at wide zoom; every event line remains and is hoverable.
        for plot in (self.plot_alt, self.plot_acc):
            last_x = [-1e9] * 4
            for j, (_, line) in enumerate((p, l) for p, l in self.state_marks if p is plot):
                x = plot.getViewBox().mapViewToScene(QtCore.QPointF(line.value(), 0)).x()
                lane = j % 4
                visible = x - last_x[lane] >= 125
                line.label.setVisible(visible)
                if visible:
                    last_x[lane] = x

    def reset_session(self):
        self._clear_override()
        self.data.clear()
        self._clear_plot_history()
        self.curve_alt.setData([], [])
        self.curve_acc.setData([], [])
        self.max_line.setVisible(False)
        self._on_view_changed()
        self._reset_session_vars()
        for c, v in ((self.card_state, "---"), (self.card_alt, "---"), (self.card_vz, "---"),
                     (self.card_max, "---"), (self.card_fix, "---"), (self.card_sats, "---"),
                     (self.card_lat, "---"), (self.card_lng, "---"),
                     (self.card_acc, "---"), (self.card_cont, "---")):
            c.set(v, "", "normal")
        self.card_go.set("NO-GO", "Waiting for telemetry", "normal")
        self.card_eject.set("STANDBY", "", "normal")
        self.card_chute.set("STOWED", "", "normal")

    def closeEvent(self, event):
        self.disconnect_source()
        super().closeEvent(event)


# --------------------------------------------------------------------------
def parse_args():
    ap = argparse.ArgumentParser(description="Rocket telemetry ground station")
    ap.add_argument("--port", help="serial port to preselect and auto-connect (e.g. COM5, /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="baud rate (default 115200)")
    ap.add_argument("--demo", action="store_true", help="run a simulated flight, no hardware needed")
    ap.add_argument("--replay", help="replay a recorded CSV telemetry file from ./telemetry_logs/")
    ap.add_argument("--light", action="store_true", help="start in sunlight-readable light mode")
    ap.add_argument("--csv", help="log folder, or file prefix (a timestamp is always appended)")
    return ap.parse_known_args()


def main():
    args, qt_args = parse_args()

    # Ensure Windows taskbar displays custom app icon instead of generic Python icon
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("myorg.rocketgroundstation.telemetry.1.0")
        except Exception:
            pass

    app = QtWidgets.QApplication([sys.argv[0]] + qt_args)
    app.setStyle("Fusion")
    app.setStyleSheet(LIGHT_STYLESHEET if args.light else DARK_STYLESHEET)

    # Set application icon (window frame & taskbar)
    base_dir = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    for icon_name in ("logo.ico", "logo.png"):
        p = base_dir / icon_name
        if not p.is_file():
            p = Path(__file__).resolve().parent / icon_name
        if p.is_file():
            app_icon = QtGui.QIcon(str(p))
            app.setWindowIcon(app_icon)
            break

    win = GroundStation(args)
    if "app_icon" in locals() and not app_icon.isNull():
        win.setWindowIcon(app_icon)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
