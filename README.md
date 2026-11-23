# Push-to system for the University telescope

Two ESP32-CAMs read the existing graduated circles (one per axis) and
report coarse HA / DEC on demand. A Python console computes LST from the
observatory coordinates and UTC, converts real HA to RA, drives a
convergence tone (rising pitch as the telescope nears the target, double
beep inside tolerance) and shows target/real coordinates plus LST on a
text-only terminal display. Fine resolution comes later from the
original graduation marks (the long marks of each code cell), with the
short marks providing sub-cell interpolation.

```
        HA circle (288 codes, 5 min/code)          DEC circle (360 codes, 0.5 deg/code)
        ESP32-CAM #1 ---HTTP /pos--.               ESP32-CAM #2 ---HTTP /pos--.
                                   v                                          v
        +--------------------------------------------------------------+
        | push_to.py: LST(site, UTC) -> rRA = LST - rHA                |
        |   tone pitch ~ angular error   double beep on tolerance      |
        |   terminal: target RA/DEC, real HA/DEC/RA, LST, error, tone  |
        +--------------------------------------------------------------+
```

## Code band layout (both axes)

One code block in the camera view (left to right):

```
[white lead-in][marker: 12 dark cells][white gap: 1 cell][9 data cells][white gap]
```

- Data cells carry a 9-bit Gray code, MSB first, dark cell = 1.
  - HA axis: 288 codes, each worth 5 min of hour angle (0..287).
  - DEC axis: 360 codes, each worth 0.5 deg, code 0 = -90 deg (0..359).
- The marker is 12 cells of dark so it is always the widest dark run:
  the longest possible *merged* dark data run is 9 cells (all-ones),
  and the 1-cell gaps keep the marker from merging with dark data.
- The gap cell after the marker means consecutive dark cells can never
  touch the marker, so marker width -> cell pitch is exact:
  `pitch_px = marker_width_px / 12`.
- Decode per frame: average the band rows into a 1-D profile, Otsu
  threshold, find the widest dark run (the marker), sample 9 fixed
  windows after the gap, compare each window mean to the midpoint of
  the dark/white levels, Gray-decode with shifted XORs. No two-width
  classification, no digit tables, no start/stop matching.

This was validated by exhaustive simulation of all 288 and 360 codes
(checkout `esp32cam_circle` tests below): 100% correct decode, clean and
with +/-40 and +/-80 grey noise per pixel, with adjacent blocks in view
and with partial (cut) markers rejected; wrong-magnification views are
rejected rather than mis-decoded.

## ESP32-CAM sketch

`esp32cam_circle/esp32cam_circle.ino`, AI-Thinker ESP32-CAM, Arduino IDE
or arduino-cli with the esp32 core.

1. Set `WIFI_SSID` / `WIFI_PASS`.
2. Set `CIRCLE_AXIS` to `AXIS_HA` for the hour-angle circle reader,
   `AXIS_DEC` for the declination circle reader; flash the identical
   sketch to both cameras.
3. Point the camera squarely at the code band so the band crosses the
   frame horizontally.

Endpoints:

- `GET /pos` -> `AXIS=HA|DEC`, `CODE=<decoded index>`, `VALUE=<HA hours | DEC degrees>`
  (HTTP 503 with `ERROR=...` when the band is not read: cut marker,
  out-of-range code, lost focus).
- `GET /raw` -> just the decoded index, for calibration.

Calibration knobs at the top of the sketch:

- `CAL_CELL_PX`: expected width of one data cell in pixels at your
  camera-to-circle distance. The marker-width guard is
  11..13 x `CAL_CELL_PX`; set this from `/raw` experiments or a photo.
- `BAND_ROW_START` / `BAND_ROW_END`: rows averaged into the profile;
  pick rows that cross the band and avoid the circle's graduation ticks
  for now (fine-reading work will use them deliberately).
- Exposure/gain are fixed in `setup()`: with the studio lights the
  levels must sit far apart (dark well under ~100, white well over
  ~180); adjust `s->set_exposure()` until `+/-40` of noise still decodes.

The sketch intentionally serves fresh captures on request (no streaming).
Averages over the band rows give roughly log2(rows) bits of noise
immunity per pixel.

## Host console

`push_to.py`, Python 3, standard library only.

Edit the site and system block at the top:

- `LATITUDE_DEG`, `LONGITUDE_EAST_DEG` (east positive)
- `ESP32_HA_URL`, `ESP32_DEC_URL` (IPs of the two cameras)
- `HA_TOLERANCE_MIN`, `DEC_TOLERANCE_ARCMIN`
- tone constants if you want a different pitch range

Run:

```
python3 push_to.py --ra 5.588 --dec -5.39      # real readers
python3 push_to.py --ra 5.588 --dec -5.39 --simulate   # push-to by hand with arrow keys
python3 push_to.py --ra 5.588 --dec -5.39 --simulate --auto   # auto-converge demo
python3 push_to.py --ra 5.588 --dec -5.39 --once       # single frame
```

### Simulation with arrow keys

`--simulate` fakes the circle readers and lets the operator slew the
simulated telescope with the arrow keys, which is the real operating
experience: move onto the target until the tone climbs and the double
beep fires.

- Right arrow: RA +5 min of time (real HA -5 min)
- Left arrow: RA -5 min of time (real HA +5 min)
- Up arrow: DEC +0.5 deg ; Down arrow: DEC -0.5 deg
- Holding a key auto-repeats; steps accumulate while buffered
- Ctrl-C quits (works in raw mode); terminals only (pipe-friendly
  degradation: keys are ignored, `--auto` gives the same screen)

The step sizes match one code cell on each circle, so the keyboard
simulator moves the same quantum the push-to tone propagates. The
simulated reading carries a small jitter (15% of a cell) so the display
behaves like a real readout.

The simulated telescope has sidereal tracking on, like the real mount
when driven: the pointing RA is held between keystrokes while the HA
circle drifts at the sidereal rate (~15 arcsec/s, one 5 min code cell
per ~5 minutes), so the real HA readout creeps forward the way it does
on the actual telescope with the clock drive running.

Behaviour:

- LST each frame from UTC and east longitude
  (GMST = 280.46061837 + 360.98564736629 x (JD - 2451545.0), plus
  longitude; verified against the 2024-03-20 00:00 UTC epoch,
  GMST = 11h 52m).
- Real RA = LST - real HA (mod 24h), the same convention the telescope
  circles use.
- One short tone per update period; frequency rises exponentially from
  400 Hz (10 deg away) to 2000 Hz (on target), 400 Hz while outside the
  converge range. Double beep on entering the tolerance box, done after
  60 s inside it.
- Terminal display: target RA/DEC, real HA/DEC, real RA, LST, angular
  error, current tone, status. Rewrites in place on terminals.

## Next steps

- Fine reading: overlay detection of the original long/short
  graduation marks inside the code cell currently under the crosshair,
  weighted centroid -> sub-cell interpolation (5 min -> arcseconds-ish).
- Band artwork generator (SVG/PDF at ring radius, one marker + 9 cells
  per block, 288/360 blocks) is a natural companion script.
- Optional: ESP32-CAM soft-AP fallback and NTP on the host side.
