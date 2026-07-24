"""
EchoPath - Serial Communication Module
Handles all communication with the ESP32 over USB Serial (UART).

Serial protocol (ESP32 → Python):
  DIST:<cm>     — ultrasonic distance reading every ~100 ms
  CMD:<cmd>     — echo of the last command the ESP32 received
  EchoPath Ready — startup banner

Serial protocol (Python → ESP32):
  LEFT\\n        — two short beeps, indicates obstacle on left
  RIGHT\\n       — one long beep,   indicates obstacle on right
  CENTER\\n      — continuous tone, indicates obstacle ahead
  BEEP\\n        — single short beep (object-in-range alert)
  NONE\\n        — buzzer off / all clear
"""

import serial
import serial.tools.list_ports
import threading
import time
from collections import deque

# ---- CONFIG ----
_FALLBACK_PORT  = "COM10"   # used if auto-detect fails
BAUD            = 115200
RECONNECT_DELAY = 2.0       # seconds between reconnect attempts
MEDIAN_WINDOW   = 5         # rolling median window for distance smoothing

# Known USB-Serial chip descriptions used by ESP32 boards
_ESP32_KEYWORDS = ("CH340", "CP210", "CP2102", "FTDI", "Silicon Labs", "USB Serial")

def _auto_detect_port() -> str:
    """Scan connected serial ports and return the first one that looks like
    an ESP32 (CH340 / CP210x / FTDI chip). Falls back to _FALLBACK_PORT."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").upper()
        if any(kw.upper() in desc for kw in _ESP32_KEYWORDS):
            print(f"[serial_comm] Auto-detected ESP32 on {p.device}  ({p.description})")
            return p.device
    print(f"[serial_comm] No ESP32 found by auto-detect — using fallback {_FALLBACK_PORT}")
    return _FALLBACK_PORT

PORT = _auto_detect_port()

# ---- STATE ----
_latest_distance  = -1
_distance_history = deque(maxlen=MEDIAN_WINDOW)
_lock             = threading.Lock()
_ser              = None
_running          = True
_connected        = False


# ── Internal helpers ──────────────────────────────────────────────────────────

def _open_serial():
    """Open the serial port and wait for the ESP32 to finish rebooting.
    Opening a serial connection toggles DTR which resets most ESP32 boards —
    if we start reading/writing immediately we can lose data or send a
    command mid-reboot."""
    # Re-detect each time we try to connect (port may change after replug)
    port = _auto_detect_port()
    s = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(2)           # let the ESP32 reboot and settle
    return s


def _read_serial_loop():
    """Background thread: continuously read lines from the ESP32.
    Automatically reconnects if the serial connection drops."""
    global _latest_distance, _ser, _connected

    while _running:
        try:
            if _ser is None:
                _ser = _open_serial()
                _connected = True
                print(f"[serial_comm] Connected on {PORT} @ {BAUD} baud")

            if _ser.in_waiting:
                raw  = _ser.readline().decode(errors="ignore").strip()

                if raw.startswith("DIST:"):
                    try:
                        value = int(raw.split(":")[1])
                        with _lock:
                            if value > 0:           # ignore -1 (sensor timeout)
                                _distance_history.append(value)
                            _latest_distance = value
                    except ValueError:
                        pass    # malformed line — skip silently

                elif raw.startswith("CMD:"):
                    # ESP32 echoes back every command it receives
                    print(f"[serial_comm] ESP32 ACK  → {raw}")

                elif raw == "EchoPath Ready":
                    print(f"[serial_comm] ESP32 booted and ready.")

        except (serial.SerialException, OSError) as e:
            _connected = False
            print(f"[serial_comm] Serial error: {e}  — retrying in {RECONNECT_DELAY}s")
            try:
                if _ser:
                    _ser.close()
            except Exception:
                pass
            _ser = None
            time.sleep(RECONNECT_DELAY)

        time.sleep(0.01)


# ── Public API ────────────────────────────────────────────────────────────────

def is_connected() -> bool:
    """Returns True if the serial port is currently open and active."""
    return _connected and _ser is not None


def get_distance() -> int:
    """Returns the median-smoothed distance in cm, or -1 if no valid reading."""
    with _lock:
        if not _distance_history:
            return -1
        return sorted(_distance_history)[len(_distance_history) // 2]


def send_command(command: str):
    """Send a direction command (LEFT / RIGHT / CENTER / NONE) to the ESP32.
    The ESP32 firmware translates these into distinct buzzer patterns."""
    global _ser
    if _ser is None:
        return
    try:
        _ser.write((command + "\n").encode())
        print(f"[serial_comm] {time.strftime('%H:%M:%S')} SENT  {command}")
    except Exception as e:
        print(f"[serial_comm] Failed to send command '{command}': {e}")


def send_beep():
    """Send a single short BEEP command — used when an object is detected
    within the ultrasonic threshold but no specific direction override is needed.
    The ESP32 must handle 'BEEP' the same as 'CENTER' or similar short tone."""
    send_command("CENTER")      # CENTER = continuous tone; Python controls timing


def shutdown():
    """Call on program exit to cleanly release the serial port."""
    global _running, _connected
    _running   = False
    _connected = False
    time.sleep(0.15)
    try:
        if _ser:
            _ser.close()
            print("[serial_comm] Serial port closed.")
    except Exception:
        pass


# ── Start background reader immediately on import ─────────────────────────────
_thread = threading.Thread(target=_read_serial_loop, daemon=True)
_thread.start()
