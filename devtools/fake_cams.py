#!/usr/bin/env python3
"""Fake ESP32-CAM circle readers for hardware-free testing.

Serves the same HTTP interface as the real ESP32-CAM sketch on two
local ports so the whole host chain - push_to.py's HTTP read path or
the INDI driver through indiserver - runs with no hardware attached:

    python3 devtools/fake_cams.py                       # static values
    python3 devtools/fake_cams.py --drift --ra 5.588 --dec -5.39
    python3 devtools/fake_cams.py --fail                # band unreadable
    python3 devtools/fake_cams.py --ha 6.12 --dec 23.4  # park elsewhere

Then point the consumers at it, e.g.:

    python3 push_to.py --ra 5.588 --dec -5.39         (edit ESP32_*_URL to
                                                       http://127.0.0.1:8801
                                                       and :8802 first)
or in the INDI client: HA_URL http://127.0.0.1:8801/pos,
DEC_URL http://127.0.0.1:8802/pos.

--drift mimics a driven scope converging on the target: with tracking
on, the fake scope holds a pointing RA (HA comes from LST) and drifts
onto the target the way push_to.py's --auto does. Codes are derived
from the axis values exactly like the real decoders do (HA code =
minutes / 5, DEC code = (deg + 90) / 0.5), so CODE= is consistent with
VALUE= for calibration checks.
"""

import argparse
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HA_AXIS = "HA"
DEC_AXIS = "DEC"


def wrap24(x):
    r = math.fmod(x, 24.0)
    return r + 24.0 if r < 0.0 else r


def lst_hours(utc_epoch_s, lon_east_deg):
    jd = utc_epoch_s / 86400.0 + 2440587.5
    t = jd - 2451545.0
    gmst = (280.46061837 + 360.98564736629 * t) % 360.0
    return ((gmst + lon_east_deg) % 360.0) / 15.0


class AxisState:
    def __init__(self, axis, value, drift=None, jitter=0.0):
        self.axis = axis
        self.value = value
        self.drift = drift
        self.jitter = jitter
        self.point_ra = None
        self.lock = threading.Lock()

    def current(self, lon_east_deg):
        with self.lock:
            if self.drift is not None:
                t_ra, t_dec = self.drift
                lst = lst_hours(time.time(), lon_east_deg)
                if self.point_ra is None:
                    self.point_ra = wrap24(lst - self.value)
                error_ra = ((t_ra - self.point_ra + 12.0) % 24.0) - 12.0
                self.point_ra = wrap24(self.point_ra + error_ra * 0.25)
                if self.axis == HA_AXIS:
                    self.value = wrap24(lst - self.point_ra)
                else:
                    self.value += (t_dec - self.value) * 0.25
            jitter = 0.0
            if self.jitter:
                jitter = (math.sin(time.time() * 3.7) * 0.5 + 0.5) \
                    * self.jitter
            if self.axis == HA_AXIS:
                return self.axis, wrap24(self.value + jitter)
            return self.axis, self.value + jitter


def make_handler(axis_state, fail, lon_east_deg):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code, body):
            data = body.encode("ascii")
            self.send_response(code)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _value(self):
            axis, value = axis_state.current(lon_east_deg)
            if axis == HA_AXIS:
                code = int(round((value * 60.0) % 1440.0 / 5.0)) % 288
            else:
                value = max(-90.0, min(90.0, value))
                code = int(round((value + 90.0) / 0.5)) % 360
            return axis, code, value

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/pos":
                self._pos()
            elif path == "/raw":
                self._raw()
            else:
                self._reply(404, "ERROR=no such path\n")

        def _pos(self):
            if fail[0]:
                self._reply(503, "AXIS=%s\nERROR=no marker (fake fail)\n"
                            % axis_state.axis)
                return
            axis, code, value = self._value()
            self._reply(200, "AXIS=%s\nCODE=%d\nVALUE=%.4f\n"
                        % (axis, code, value))

        def _raw(self):
            if fail[0]:
                self._reply(503, "-1\n")
                return
            _, code, _ = self._value()
            self._reply(200, "%d\n" % code)

        def log_message(self, fmt, *args):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    ap.add_argument("--ha", type=float, default=6.25,
                    help="fake HA reading in hours (default 6.25)")
    ap.add_argument("--dec", type=float, default=20.0,
                    help="fake DEC reading in degrees (default 20)")
    ap.add_argument("--ha-port", type=int, default=8801)
    ap.add_argument("--dec-port", type=int, default=8802)
    ap.add_argument("--longitude", type=float, default=6.0,
                    help="site east longitude for --drift LST (default 6)")
    ap.add_argument("--drift", action="store_true",
                    help="converge onto --ra/--dec like a driven scope "
                         "(for GOTO tests with the helper)")
    ap.add_argument("--ra", type=float, default=5.588,
                    help="drift target RA in hours (default 5.588)")
    ap.add_argument("--dec-target", type=float, default=-5.39,
                    help="drift target DEC in degrees (default -5.39)")
    ap.add_argument("--jitter", type=float, default=0.0,
                    help="reading jitter: HA hours / DEC degrees")
    ap.add_argument("--fail", action="store_true",
                    help="answer 503 ERROR= like an unreadable band")
    args = ap.parse_args()

    fail = [args.fail]
    targets = (args.ra, args.dec_target) if args.drift else None
    states = (
        AxisState(HA_AXIS, args.ha, targets, args.jitter),
        AxisState(DEC_AXIS, args.dec, targets, args.jitter),
    )
    servers = []
    for state, port in zip(states, (args.ha_port, args.dec_port)):
        httpd = ThreadingHTTPServer(("127.0.0.1", port),
                                    make_handler(state, fail, args.longitude))
        servers.append(httpd)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()

    def print_status():
        while True:
            time.sleep(5.0)
            parts = []
            for state, port in zip(states, (args.ha_port, args.dec_port)):
                axis, value = state.current(args.longitude)
                parts.append("%s@%d VALUE=%.4f" % (axis, port, value))
            print("[fake_cams] %s  fail=%s" % (" | ".join(parts), fail[0]),
                  flush=True)

    threading.Thread(target=print_status, daemon=True).start()

    print("fake circle readers: "
          "http://127.0.0.1:%d/pos (HA), http://127.0.0.1:%d/pos (DEC)"
          % (args.ha_port, args.dec_port), flush=True)
    try:
        while True:
            time.sleep(3600.0)
    except KeyboardInterrupt:
        for httpd in servers:
            httpd.shutdown()


if __name__ == "__main__":
    main()
