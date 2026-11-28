#!/usr/bin/env python3
"""ESP32-CAM stand-in fed by a real INDI mount driver.

Instead of faking readings outright, this bridge snoops an INDI
telescope driver (e.g. "EQMod Mount" connected to a real mount) and
serves its sky position on the same HTTP interface as the circle
readers. Moving the real mount by hand then reads like moving the
University telescope's circles - the full chain (HTTP readers ->
push_to.py / indi_push_to_circle -> helper) runs against genuine,
continuously-changing sky coordinates:

    indiserver -v indi_eqmod_telescope   # your real-mount driver
    python3 devtools/mount_cams.py                 # snoops 127.0.0.1:7624
    python3 devtools/mount_cams.py --host 192.168.x.y --device "EQMod Mount"

On the consumer side, point PUSH_TO_CONFIG's HA_URL / DEC_URL at
http://127.0.0.1:8801/pos and http://127.0.0.1:8802/pos (or the
ESP32_*_URL constants in push_to.py). Like fake_cams.py, it serves
/pos (AXIS=/CODE=/VALUE=), /raw and - when no mount data has arrived
recently - HTTP 503 with ERROR=.

HA is computed as LST - RA at --longitude (west positive), the same
convention the mechanical circles and every other tool in this repo
use. Coordinates are taken from EQUATORIAL_EOD_COORD (epoch of now),
matching the push-to philosophy. Everything is stdlib-only: the INDI
client speaks the raw XML protocol over TCP, no pyindi-client needed,
and the bridge is read-only - it never sends anything to the mount
except the initial subscription.
"""

import argparse
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from push_to import lst_hours  # noqa: E402

COORD_PROP = "EQUATORIAL_EOD_COORD"


def wrap24(x):
    r = x % 24.0
    return r + 24.0 if r < 0.0 else r


class MountPosition:
    """Current RA/DEC of the snooped mount, with staleness tracking."""

    def __init__(self, stale_s):
        self.lock = threading.Lock()
        self.ra = 0.0
        self.dec = 0.0
        self.last_update = 0.0
        self.stale_s = stale_s

    def update(self, ra, dec):
        with self.lock:
            self.ra = ra
            self.dec = dec
            self.last_update = time.time()

    def ok(self):
        return time.time() - self.last_update < self.stale_s

    def get(self):
        with self.lock:
            return self.ra, self.dec, self.ok()


def code_from_value(axis, value):
    if axis == "HA":
        return int(round((value * 60.0) % 1440.0 / 5.0)) % 288
    value = max(-90.0, min(90.0, value))
    return int(round((value + 90.0) / 0.5)) % 360


class IndiSnoop(threading.Thread):
    """Minimal INDI client: subscribes and tracks <setNumberVector>."""

    def __init__(self, host, port, device, position):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.device = device
        self.position = position
        self.stop = threading.Event()

    def _source(self, sock):
        """Yield complete top-level XML elements from the socket stream.

        INDI sends a sequence of independent XML documents on one
        connection, so a single XMLPullParser chokes after the first
        one. Split documents by tag depth instead (attribute values
        and comments are stepped over verbatim).
        """
        sock_file = sock.makefile("rb")
        buf = b""
        depth = 0
        doc_start = 0
        pos = 0
        while not self.stop.is_set():
            chunk = sock_file.read1(4096)
            if not chunk:
                break
            buf += chunk
            while pos < len(buf):
                if buf[pos:pos + 1] != b"<":
                    pos += 1
                    continue
                gt = buf.find(b">", pos)
                if gt < 0:
                    break
                tag = buf[pos:gt + 1]
                if tag.startswith(b"<!--"):
                    end = buf.find(b"-->", pos)
                    if end < 0:
                        break
                    pos = end + 3
                    continue
                if tag.startswith(b"<?") or tag.startswith(b"<!"):
                    if depth == 0:
                        doc_start = gt + 1
                elif tag.startswith(b"</"):
                    depth -= 1
                    if depth == 0:
                        yield ET.fromstring(buf[doc_start:gt + 1])
                        doc_start = gt + 1
                elif tag.endswith(b"/>"):
                    if depth == 0:
                        yield ET.fromstring(buf[doc_start:gt + 1])
                        doc_start = gt + 1
                else:
                    if depth == 0:
                        doc_start = pos
                    depth += 1
                pos = gt + 1
            if depth > 0:
                buf = buf[doc_start:]
                pos = 0
                doc_start = 0
            elif pos > 0:
                buf = buf[pos:]
                pos = 0
                doc_start = 0

    def _process(self, elem):
        if elem.tag not in ("setNumberVector", "defNumberVector"):
            return
        if self.device and elem.get("device") != self.device:
            return
        if elem.get("name") != COORD_PROP:
            return
        ra = dec = None
        for one in elem:
            if one.tag != "oneNumber":
                continue
            if one.get("name") == "RA":
                try:
                    ra = float(one.text)
                except ValueError:
                    pass
            elif one.get("name") == "DEC":
                try:
                    dec = float(one.text)
                except ValueError:
                    pass
        if ra is not None and dec is not None:
            self.position.update(ra, dec)

    def run(self):
        while not self.stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), 5.0)
                print("[mount_cams] connected to indiserver %s:%d"
                      % (self.host, self.port), flush=True)
            except OSError as exc:
                print("[mount_cams] %s:%d unreachable (%s), retrying in 5 s"
                      % (self.host, self.port, exc), flush=True)
                time.sleep(5.0)
                continue
            try:
                sock.sendall(b'<getProperties version="1.7"/>\n')
                for elem in self._source(sock):
                    self._process(elem)
            except OSError:
                pass
            finally:
                sock.close()
            if not self.stop.is_set():
                print("[mount_cams] indiserver connection lost, retrying",
                      flush=True)
                time.sleep(5.0)


def make_handler(position, axis, lon_east_deg):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code, body):
            data = body.encode("ascii")
            self.send_response(code)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path not in ("/pos", "/raw"):
                self._reply(404, "ERROR=no such path\n")
                return
            ra, dec, ok = position.get()
            if not ok:
                self._reply(503, "AXIS=%s\nERROR=no mount data\n" % axis)
                return
            if axis == "HA":
                value = wrap24(lst_hours(time.time(), lon_east_deg) - ra)
            else:
                value = dec
            if path == "/raw":
                self._reply(200, "%d\n" % code_from_value(axis, value))
            else:
                self._reply(200, "AXIS=%s\nCODE=%d\nVALUE=%.4f\n"
                            % (axis, code_from_value(axis, value), value))

        def log_message(self, fmt, *args):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1",
                    help="indiserver host (default 127.0.0.1)")
    ap.add_argument("--port", type=int, default=7624,
                    help="indiserver TCP port (default 7624)")
    ap.add_argument("--device", default="",
                    help="device name of the mount driver (default: any "
                         "driver that defines EQUATORIAL_EOD_COORD)")
    ap.add_argument("--longitude", type=float, default=6.0,
                    help="site east longitude for LST (default 6.0)")
    ap.add_argument("--stale", type=float, default=10.0,
                    help="serves 503 if no mount update for this many "
                         "seconds (default 10)")
    ap.add_argument("--ha-port", type=int, default=8801)
    ap.add_argument("--dec-port", type=int, default=8802)
    args = ap.parse_args()

    position = MountPosition(args.stale)
    IndiSnoop(args.host, args.port, args.device, position).start()

    for axis, port in (("HA", args.ha_port), ("DEC", args.dec_port)):
        httpd = ThreadingHTTPServer(("127.0.0.1", port),
                                    make_handler(position, axis,
                                                 args.longitude))
        threading.Thread(target=httpd.serve_forever, daemon=True).start()

    def print_status():
        while True:
            time.sleep(5.0)
            ra, dec, ok = position.get()
            if not ok:
                print("[mount_cams] no mount data yet (indiserver up? mount "
                      "connected?)", flush=True)
                continue
            age = time.time() - position.last_update
            ha = wrap24(lst_hours(time.time(), args.longitude) - ra)
            print("[mount_cams] RA=%.4f h DEC=%.4f deg -> HA=%.4f h "
                  "(age %.1f s)" % (ra, dec, ha, age), flush=True)

    threading.Thread(target=print_status, daemon=True).start()

    print("circle stand-in on http://127.0.0.1:%d/pos (HA) and "
          "http://127.0.0.1:%d/pos (DEC); snooping %s:%d for %s"
          % (args.ha_port, args.dec_port, args.host, args.port,
             args.device or "any telescope driver"), flush=True)
    try:
        while True:
            time.sleep(3600.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
