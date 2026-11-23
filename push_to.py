#!/usr/bin/env python3
"""Push-to console for the University telescope.

Reads real HA / DEC from the two circle-reading ESP32-CAMs, computes LST
from the observatory coordinates and UTC time, converts real HA to RA,
sounds a tracking tone whose pitch rises as the telescope converges on
the target and double-beeps inside the tolerance box, while showing
target / real coordinates and LST on a text-only terminal screen.

Usage:
    python3 push_to.py                      # talk to the ESP32-CAMs
    python3 push_to.py --simulate           # arrow keys push RA/DEC by hand
    python3 push_to.py --simulate --auto    # offline auto-convergence demo
    python3 push_to.py --once               # print one display frame, exit
"""

import argparse
import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request
import wave

if os.name == "nt":
    import msvcrt
else:
    import select
    import termios
    import tty

# ----------------------------------------------------------------------------
# Site and system configuration (edit to match the installation)
# ----------------------------------------------------------------------------

LATITUDE_DEG = 45.0        # observatory latitude, degrees north positive
LONGITUDE_EAST_DEG = 6.0   # observatory longitude, degrees east positive

ESP32_HA_URL = "http://192.168.4.1/pos"    # circle reader on the HA axis
ESP32_DEC_URL = "http://192.168.4.2/pos"   # circle reader on the DEC axis

TIMEOUT_SECONDS = 60.0     # target reached -> stop after this long in tolerance
HA_TOLERANCE_MIN = 2.0     # tolerance on hour angle, minutes of time
DEC_TOLERANCE_ARCMIN = 30.0

HA_CODE_MINUTES = 5.0      # one HA code cell spans 5 min of hour angle
DEC_CODE_DEG = 0.5         # one DEC code cell spans 30 arcmin

KEY_STEP_RA_H = HA_CODE_MINUTES / 60.0      # RA step per arrow repeat
KEY_STEP_DEC_DEG = DEC_CODE_DEG             # DEC step per arrow repeat

TONE_MIN_HZ = 400.0        # frequency when far away (or diverging)
TONE_MAX_HZ = 2000.0       # frequency when exactly on target
CONVERGE_RANGE_DEG = 10.0  # angular distance over which the pitch rises
BEEP_SECONDS = 0.07
BEEP_PERIOD_SECONDS = 0.30

ansi = sys.stdout.isatty()

# ----------------------------------------------------------------------------
# Sidereal time and coordinate conversions
# ----------------------------------------------------------------------------


def lst_hours(utc_epoch_s, lon_east_deg):
    """Local sidereal time in hours from UTC epoch seconds and east longitude."""
    jd = utc_epoch_s / 86400.0 + 2440587.5
    t = jd - 2451545.0
    gmst_deg = (280.46061837 + 360.98564736629 * t) % 360.0
    return ((gmst_deg + lon_east_deg) % 360.0) / 15.0


def ha_to_ra(ha_hours, lst):
    """Hour angle -> right ascension, both in hours."""
    return (lst - ha_hours) % 24.0


def separation_deg(ra1_hours, dec1_deg, ra2_hours, dec2_deg):
    """Angular separation on the sky between two (RA, DEC) positions."""
    a1, d1 = math.radians(ra1_hours * 15.0), math.radians(dec1_deg)
    a2, d2 = math.radians(ra2_hours * 15.0), math.radians(dec2_deg)
    cos_d = (
        math.sin(d1) * math.sin(d2)
        + math.cos(d1) * math.cos(d2) * math.cos(a1 - a2)
    )
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_d))))


# ----------------------------------------------------------------------------
# Reading the circles (ESP32-CAM readers)
# ----------------------------------------------------------------------------


def fetch_axis(url):
    """GET /pos from one ESP32-CAM, return the float value it reports."""
    with urllib.request.urlopen(url, timeout=2.0) as resp:
        for line in resp.read().decode("ascii", "replace").splitlines():
            if line.startswith("VALUE="):
                return float(line.split("=", 1)[1])
    raise ValueError("no VALUE= line in reply from %s" % url)


def read_scope():
    """Return (real HA hours, real DEC degrees) from the circle readers."""
    rha = fetch_axis(ESP32_HA_URL)
    rdec = fetch_axis(ESP32_DEC_URL)
    return rha, rdec


class KeyboardArrows:
    """Non-blocking arrow-key reader; returns per-frame motion requests.

    read_motion() drains stdin and returns (d_ra_hours, d_dec_deg):
    right arrow -> RA +, left -> RA -, up -> DEC +, down -> DEC -.
    Ctrl-C always raises KeyboardInterrupt, even in cbreak mode.
    """

    def __init__(self):
        self._old = None
        self._win = os.name == "nt"
        if self._win:
            return
        self._fd = sys.stdin.fileno()
        if not os.isatty(self._fd):
            self._win = None
            return
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def _readchar(self, timeout):
        if self._win in (True, None):
            if self._win and msvcrt.kbhit():
                return msvcrt.getwch()
            return ""
        if select.select([self._fd], [], [], timeout)[0]:
            return os.read(self._fd, 1).decode("utf-8", "replace")
        return ""

    def read_motion(self):
        if self._win is None:
            return 0.0, 0.0
        d_ra = 0.0
        d_dec = 0.0
        while True:
            ch = self._readchar(0.0)
            if not ch:
                break
            if ch in ("\x00", "\xe0") and self._win:
                ext = self._readchar(0.0)
                if ext in ("K", "M"):
                    d_ra -= KEY_STEP_RA_H if ext == "K" else -KEY_STEP_RA_H
                elif ext in ("H", "P"):
                    d_dec += KEY_STEP_DEC_DEG if ext == "H" else -KEY_STEP_DEC_DEG
                continue
            if ch == "\x03":
                raise KeyboardInterrupt
            if ch != "\x1b":
                continue
            c1 = self._readchar(0.05)
            if c1 != "[":
                continue
            c2 = self._readchar(0.05)
            if c2 == "C":
                d_ra += KEY_STEP_RA_H
            elif c2 == "D":
                d_ra -= KEY_STEP_RA_H
            elif c2 == "A":
                d_dec += KEY_STEP_DEC_DEG
            elif c2 == "B":
                d_dec -= KEY_STEP_DEC_DEG
        return d_ra, d_dec

    def close(self):
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            self._old = None


class SimulatedScope:
    """Fake circle readers.

    Keyboard mode: arrows move the real position by KEY_STEP_RA_H and
    KEY_STEP_DEC_DEG per arrow received. Auto mode (--auto): drifts
    onto the target like a perfect pusher, for a hands-free demo.
    """

    def __init__(self, t_ra, t_dec, auto=False):
        self.t_ra = t_ra
        self.t_dec = t_dec
        self.auto = auto
        self.ha = None
        self.dec = t_dec - 8.0

    def __call__(self, lst, motion=(0.0, 0.0)):
        if self.ha is None:
            self.ha = ((lst - self.t_ra) + 3.0) % 24.0
        if self.auto:
            error_ha = ((lst - self.t_ra - self.ha + 12.0) % 24.0) - 12.0
            error_dec = self.t_dec - self.dec
            rate = 0.08
            self.ha = (self.ha + error_ha * rate) % 24.0
            self.dec += error_dec * rate
        else:
            d_ra, d_dec = motion
            self.ha = (self.ha - d_ra) % 24.0
            self.dec = max(-90.0, min(90.0, self.dec + d_dec))
        jitter_ha = 0.15 * HA_CODE_MINUTES / 60.0
        jitter_dec = 0.15 * DEC_CODE_DEG
        rha = (self.ha + ((time.time() * 7) % 1 - 0.5) * jitter_ha) % 24.0
        rdec = self.dec + ((time.time() * 11) % 1 - 0.5) * jitter_dec
        return rha, rdec


# ----------------------------------------------------------------------------
# Sound
# ----------------------------------------------------------------------------


def _wav_bytes(freq_hz, duration_s, volume=0.5):
    n = int(44100.0 * duration_s)
    fade = min(n // 10, 220)
    frames = bytearray()
    for i in range(n):
        amp = volume
        if i < fade:
            amp *= i / fade
        elif i > n - fade:
            amp *= (n - i) / fade
        sample = int(amp * 32767.0 * math.sin(2.0 * math.pi * freq_hz * i / 44100.0))
        frames += struct.pack("<h", sample)
    import io

    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(44100)
        w.writeframes(bytes(frames))
    return buf.getvalue()


_PLAYER = None
_PLAYER_RESOLVED = False


def _resolve_player():
    global _PLAYER, _PLAYER_RESOLVED
    if not _PLAYER_RESOLVED:
        _PLAYER_RESOLVED = True
        if os.name == "nt":
            _PLAYER = "winsound"
        else:
            for cmd in ("paplay", "aplay", "afplay", "ffplay"):
                if shutil.which(cmd):
                    _PLAYER = cmd
                    break
    return _PLAYER


def play_tone(freq_hz, duration_s=BEEP_SECONDS):
    """Play one sine tone; platform players, no third-party dependencies."""
    player = _resolve_player()
    if player is None:
        return
    try:
        if player == "winsound":
            import winsound

            winsound.Beep(int(freq_hz), int(duration_s * 1000))
            return
        path = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        path.write(_wav_bytes(freq_hz, duration_s))
        path.close()
        if player == "ffplay":
            cmd = ["ffplay", "-nodisp", "-loglevel", "quiet", "-autoexit", path.name]
        else:
            cmd = [player, path.name]
        subprocess.run(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True
        )
        os.unlink(path.name)
    except Exception:
        pass


# ----------------------------------------------------------------------------
# Display
# ----------------------------------------------------------------------------


def hm(hours):
    sign = "-" if hours < 0 else " "
    a = abs(hours)
    h = int(a)
    m = (a - h) * 60.0
    return "%s%02dh %05.2fm" % (sign, h, m)


def dms(deg):
    sign = "+" if deg >= 0 else "-"
    a = abs(deg)
    d = int(a)
    rest = (a - d) * 60.0
    m = int(rest)
    s = (rest - m) * 60.0
    return "%s%02dd %02dm %04.1fs" % (sign, d, m, s)


def draw_frame(lines):
    if ansi:
        sys.stdout.write("\x1b[s\x1b[H")
    for line in lines:
        sys.stdout.write(line + "\x1b[K\n")
    if ansi:
        sys.stdout.write("\x1b[J")
    sys.stdout.flush()


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------


def run(target_ra_hours, target_dec_deg, simulate, once, auto=False, keys=None):
    read = SimulatedScope(target_ra_hours, target_dec_deg, auto) if simulate else read_scope
    tolerance_deg = math.hypot(
        HA_TOLERANCE_MIN / 60.0 * math.cos(math.radians(target_dec_deg)),
        DEC_TOLERANCE_ARCMIN / 60.0,
    )
    in_tol_since = None
    first_frame = True

    while True:
        lst = lst_hours(time.time(), LONGITUDE_EAST_DEG)
        try:
            if simulate:
                rha, rdec = read(lst, keys.read_motion() if keys else (0.0, 0.0))
            else:
                rha, rdec = read()
        except Exception as exc:
            lines = [
                "telescope push-to console%s"
                % ("   [simulated circles]" if simulate else ""),
                "",
                "LST          %s" % hm(lst),
                "circle error: %s" % exc,
            ]
            if once:
                for line in lines:
                    print(line)
                return
            draw_frame(lines)
            time.sleep(1.0)
            first_frame = False
            continue

        rra = ha_to_ra(rha, lst)
        sep = separation_deg(ra1_hours=target_ra_hours, dec1_deg=target_dec_deg,
                             ra2_hours=rra, dec2_deg=rdec)
        closing = sep < CONVERGE_RANGE_DEG
        progress = max(0.0, 1.0 - sep / CONVERGE_RANGE_DEG)
        freq = TONE_MIN_HZ
        if closing:
            freq = TONE_MIN_HZ * (TONE_MAX_HZ / TONE_MIN_HZ) ** math.sqrt(progress)

        in_tol = sep <= tolerance_deg
        now = time.time()
        if in_tol and in_tol_since is None:
            in_tol_since = now
            play_tone(TONE_MAX_HZ, 0.12)
            time.sleep(0.08)
            play_tone(TONE_MAX_HZ, 0.12)
        elif in_tol and now - in_tol_since > TIMEOUT_SECONDS:
            draw_frame(["telescope push-to console", "target reached, done"])
            return
        elif not in_tol:
            in_tol_since = None
            play_tone(freq)

        lines = [
            "telescope push-to console%s"
            % ("   [simulated circles]" if simulate else ""),
            "",
            "LST          %s" % hm(lst),
            "",
            "target RA    %s    DEC %s" % (hm(target_ra_hours), dms(target_dec_deg)),
            "real   HA    %s    DEC %s" % (hm(rha), dms(rdec)),
            "real   RA    %s" % hm(rra),
            "",
            "error        %6.2f deg   (tolerance %.2f deg)"
            % (sep, tolerance_deg),
            "tone         %4.0f Hz" % freq,
            "status       %s"
            % (
                "ON TARGET - double beep"
                if in_tol
                else ("converging" if closing else "diverging - slew toward target")
            ),
        ]
        if simulate and not auto:
            lines.insert(
                2,
                "arrows: <-/-> RA %s, up/down DEC %s   Ctrl-C quit"
                % (hm(KEY_STEP_RA_H).strip(), dms(KEY_STEP_DEC_DEG)),
            )
        if once:
            for line in lines:
                print(line)
            return
        if first_frame and ansi:
            sys.stdout.write("\x1b[2J\x1b[?25l")
            first_frame = False
        draw_frame(lines)
        time.sleep(max(0.0, BEEP_PERIOD_SECONDS - BEEP_SECONDS))


def main():
    ap = argparse.ArgumentParser(description="Push-to console for the telescope")
    ap.add_argument("--ra", type=float, required=True, help="target RA in hours")
    ap.add_argument("--dec", type=float, required=True, help="target DEC in degrees")
    ap.add_argument(
        "--simulate", action="store_true", help="fake the circle readers"
    )
    ap.add_argument(
        "--auto",
        action="store_true",
        help="with --simulate: drift onto the target automatically",
    )
    ap.add_argument("--once", action="store_true", help="print one frame and exit")
    args = ap.parse_args()
    keys = KeyboardArrows() if args.simulate and not args.once else None
    try:
        run(args.ra, args.dec, args.simulate, args.once, args.auto, keys)
    except KeyboardInterrupt:
        pass
    finally:
        if keys:
            keys.close()
        if ansi:
            sys.stdout.write("\x1b[?25h\x1b[0m")
            sys.stdout.flush()


if __name__ == "__main__":
    main()
