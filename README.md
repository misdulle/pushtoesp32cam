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

## Field-of-view layout (both axes)

As on the real instrument: the mobile part (the graduated circle)
moves VERTICALLY through the camera FOV -- the tangential direction is
the camera's Y axis, the radial direction is X. The graduated marks
and the code strips are horizontal bands, and the code cells are laid
out ALONG THE RADIUS (left to right in the frame). Each code strip is

```
[2-cell white lead-in][marker: 12 dark cells][white gap: 1 cell][9 data cells][3-cell white tail]
```

27 cells wide (along X), one cell tall (along Y). The FOV has two
radial bands side by side, plus the fixed pointer:

- LEFT radial band: the code strips. They stack vertically, one per
  5-min graduation, each ALIGNED with its long (5-min) mark: same
  height -- the codes only put a number on that mark.
- RIGHT radial band: the graduated marks, anchored at the rim: long
  (5-min, at each strip's height) and short (1-min, at the fifths
  between).
- Fixed part: a single thin pointer line, HORIZONTAL (parallel to the
  marks), at a fixed height near mid-FOV. It starts at the rim -- the
  marks' anchored right ends -- and extends rightward to the FOV's
  edge; it never crosses the code band. The full accuracy comes from
  interpolating the pointer height between the graduated lines; the
  code just identifies the 5-min line it crosses.

- Data cells carry a 9-bit Gray code, MSB first (the MSB data cell is
  the one nearest the marker, i.e. the strip's left/inner end), dark
  cell = 1.
  - HA axis: 288 codes, each worth 5 min of hour angle (0..287).
  - DEC axis: 360 codes, each worth 0.5 deg, code 0 = -90 deg (0..359).
- The marker is 12 cells of dark so it is always the widest dark run:
  the longest possible *merged* dark data run is 9 cells (all-ones),
  and the 1-cell gap keeps the marker from merging with dark data.
- The gap cell after the marker means consecutive dark cells can never
  touch the marker, so marker length -> cell pitch is exact:
  `pitch_px = marker_length_px / 12` (measured along X).
- Decode per frame (radial = X, tangential = Y): Otsu over the strip
  columns -> dark coverage per row over those columns -> the strip is
  the coverage run with >=30% of the columns dark and a height of at
  least 0.7 cells (thin marks and the pointer line are rejected), the
  run CLOSEST TO MID-FOV HEIGHT is chosen (that is where the pointer
  sits) -> radial profile averaged over the strip's rows -> Otsu,
  widest dark run along X (the marker), 11..13-cell guard, 9 fixed
  windows after the gap, each window mean vs the midpoint of the
  dark/white levels, Gray-decode with shifted XORs. No two-width
  classification, no digit tables, no start/stop matching. When two
  strips are in view the decoder reads the one nearest mid-height; if
  none is in view, `/pos` answers 503.

This was validated by exhaustive simulation of all 288 and 360 codes
(checkout `esp32cam_circle` tests below): 100% correct decode, clean and
with +/-40 and +/-80 grey noise per pixel, with adjacent strips in view
and with partial (cut) markers rejected; wrong-magnification views are
rejected rather than mis-decoded.

## ESP32-CAM sketch

`esp32cam_circle/esp32cam_circle.ino`, AI-Thinker ESP32-CAM, Arduino IDE
or arduino-cli with the esp32 core. Verified to compile with
arduino-cli 1.6.0 against esp32 core 4.0.0 (defaults: 34% flash, 19%
RAM):

```
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32cam esp32cam_circle          # HA camera
arduino-cli compile --fqbn esp32:esp32:esp32cam \
    --build-property "compiler.cpp.extra_flags=-DCIRCLE_AXIS=1" \
    esp32cam_circle                                                       # DEC camera
```

1. Set `WIFI_SSID` / `WIFI_PASS`.
2. Pick the axis by editing `CIRCLE_AXIS`, or leave the file as-is and
   use the `-DCIRCLE_AXIS=1` override above for the DEC camera.
3. Set the per-camera `STATIC_IP` (plus gateway/mask) before flashing
   each unit so the two cameras don't collide; empty strings fall back
   to DHCP.
4. Mount the camera so its long (X) axis runs ALONG the radius
   (perpendicular to the graduated lines, which are horizontal), with
   the code strips as horizontal bands in the LEFT of the frame, the
   graduated marks to their RIGHT, and the fixed pointer line at
   mid-height. The mobile circle part then moves VERTICALLY through
   the FOV as the telescope slews.

Watching the serial console:

```
arduino-cli monitor -p /dev/ttyUSB2 -c baudrate=115200,dtr=off,rts=off
```

`dtr=off,rts=off` is important: the default DTR/RTS assert on attach
can reset the board mid-boot, truncating the ROM output (a plain
monitor looks like the sketch dies after `boot:0x13`).

Endpoints:

- `GET /pos` -> `AXIS=HA|DEC`, `CODE=<decoded index>`, `VALUE=<HA hours | DEC degrees>`
  (HTTP 503 with `ERROR=...` when no strip is in view: cut marker,
  out-of-range code, lost focus, camera not ready).
- `GET /raw` -> just the decoded index, for calibration.
- `GET /debug` -> diagnostics: camera init result/error code, init
  tries, detected sensor (expect 0x3660 = OV3660, 0x2640 = OV2640),
  PSRAM size/free, free heap, grabbed frame size, WiFi IP/RSSI, and
  min/max/mean levels over the calibrated strip rows (exposure
  check: dark cells well under ~100, white well over ~180).
- `GET /jpg` -> the current grayscale frame converted with
  `frame2jpg()` (~15-60 kB, ~0.3 s) - the fast way to aim and focus;
  the sensor stays in grayscale, so `/pos` is unaffected.
- `GET /bmp` -> the current frame as a full-resolution grayscale BMP
  (~301 kB at VGA, slower to transfer) - pixel-exact inspection.
- `GET /bench` -> timing: sensor frame wait vs decode time (decode is
  a two-pass scan of the strip-column region, so `decode_ms` is a few
  times larger than the old single-profile pass -- expect single-digit
  milliseconds at SVGA; verify with `/bench`). With
  `?fs=<qqvga|qvga|vga|svga|xga|hd|sxga|uxga>` measures at that size
  (restores the default afterwards; add `&keep=1` to stay there),
  with `?all=1` sweeps the whole ladder. `fps_max` is the continuous
  rate; the working default is SVGA (800x600, ~5.6 fps measured on the
  OV3660 - the biggest image meeting the 5 fps target).
- `GET /init` -> retry camera init without rebooting (the sketch never
  restarts on camera failure: WiFi and the diagnostics endpoints stay
  up so you can iterate over HTTP alone).

WiFi comes up before the camera is initialized: unless the AP itself
is down, `/debug` answers even with the camera fully failed. Camera
buffer: uses PSRAM (VGA) when `psramFound()`, PSRAM-less modules fall
back to QVGA + DRAM automatically and the decode scales the strip
geometry (VGA-calibrated `CAL_CELL_PX`/`STRIP_COL_*` apply to either).

Calibration knobs at the top of the sketch:

- `CAL_CELL_PX`: expected radial length of one data cell in pixels at
  640 columns (scaled by the actual frame width). The marker-length
  guard is 11..13 x `CAL_CELL_PX`; measure the marker's length along X
  on a `/jpg` or `/bmp` and divide by 12. All 27 cells must fit in the
  strip region, so the ceiling is ~21 px at VGA, ~27 px at SVGA.
- `STRIP_COL_START` / `STRIP_COL_END`: the columns the strips' radial
  extent covers (40..460 at 640 columns). Aim so the code band fills
  this region; the decoder's coverage scan runs over these columns
  only, so the graduated marks (in their own band to the right) never
  enter the scan. `/debug`'s level stats cover the same columns (all
  rows) -- use them for the exposure check (dark cells well under
  ~100, white well over ~180).
- `CIRCLE_STRIP_FLIP`: set to 1 if the data cells read opposite to
  the painted order along X (window 0 = MSB = the data cell nearest
  the marker, i.e. the strip's left/inner end, by default; check by
  stepping one code with `/raw` and seeing whether the value
  increments or wraps backwards).
- Camera orientation: X (the 800 px dimension at SVGA) is radial --
  perpendicular to the graduated lines, spanning the code band and the
  tic band; Y is tangential -- the direction the circle moves as the
  telescope slews. The fixed pointer line must sit at mid-height, and
  the decoder reports the strip nearest to it (`/pos` returns 503 when
  no strip is in view; the crossed strip under the pointer is the
  realistic reading case).
- Exposure/gain are fixed in `setup()`: AEC/AGC off, manual exposure
  via `s->set_aec_value(s, 300)` (OV2640 range 0..1200). With the
  studio lights the levels must sit far apart (dark well under ~100,
  white well over ~180); raise/lower the value until +/-40 of noise
  still decodes.

The sketch intentionally serves fresh captures on request (no streaming).
Averaging over the strip's columns gives roughly log2(cols) bits of
noise immunity per pixel.

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
python3 push_to.py --ra 5.588 --dec -5.39 --quiet      # tones only, no display (INDI helper)
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
- `--quiet` skips the display entirely and keeps the tones: this is the
  mode launched by the INDI driver, so the beeps come out of the Pi's
  audio while the coordinates are shown by the INDI client instead.

## INDI telescope driver (Pi 5)

`indi_push_to/indi_push_to_circle.cpp` is a driver-only telescope
(no motor control). It reads the two ESP32-CAM circle readers over HTTP
every polling cycle, computes LST from the site location and UTC, and
reports RA/DEC like a normal telescope, so SkyChart, Stellarium, KStars
or Ekos simply show where the telescope is pointing in real time.

Selecting a target / GOTO in the client launches the push-to helper

```
push_to.py --ra <ra hours> --dec <dec degrees> --quiet
```

so the operator hand-slews while the tone rises and the double beep
marks the tolerance box. The helper exits itself; the driver's slew
state (SLEWING -> TRACKING when within 0.2 deg -> IDLE on helper exit)
follows along. The driver runs on any INDI build; it compiles against
both libindi 1.9.9 (Debian bookworm) and 2.x (trixie).

Build on the Pi 5:

```
sudo apt install build-essential cmake libindi-dev
cd indi_push_to
cmake -B build . && cmake --build build
sudo cmake --install build        # installs /usr/local/bin/indi_push_to_circle
sudo cp ../push_to.py /usr/local/bin/push_to.py && sudo chmod +x /usr/local/bin/push_to.py
```

Run it:

```
indiserver -v indi_push_to_circle &
```

then connect from SkyChart/Stellarium/KStars (INDI driver
`Push-To Circle Reader`). Driver properties:

- `PUSH_TO_CONFIG` (Options tab): `HA_URL` / `DEC_URL` (the two
  `http://<cam>/pos` endpoints) and `HELPER` (path of push_to.py).
- `TELESCOPE_LOCATION`: site latitude / longitude (east positive) /
  elevation, used for the LST computation. Saved to the driver config
  on change.
- Sync (Framing tab): stores an offset applied to the reported
  coordinates only, for small circle misalignments - the raw circle
  readings stay untouched.
- Goto: starts the helper; Abort/Disconnect kill it.

Nothing moves at any point; the driver is strictly a coordinate reader
plus the audio push-to guide.

## Test band image generator

`devtools/make_test_image.py` renders a synthetic field of view in the
style of the reference images, with the final geometry: the mobile
part moves vertically (tangential = Y, radial = X), the code strips
are horizontal bands (27 cells along X: 2-cell lead-in, 12-cell
marker, 1-cell gap, 9 Gray cells, 3-cell tail) stacked one per 5-min
line in a LEFT radial band, the graduated marks (long 5-min at each
strip's height, short 1-min at the fifths, anchored at the rim) sit in
a separate band to their RIGHT, and the fixed pointer is a single thin
horizontal line at mid-height, starting at the rim (the marks'
anchored right ends) and extending rightward to the FOV edge -- it
never crosses the code band -- the fine reading is the pointer
height interpolated between graduated lines. Use it as a print/screen
reference for laying out the real circle bands, or as a decode test
target:

```
python3 devtools/make_test_image.py --codes 25 150          # HA codes
python3 devtools/make_test_image.py --codes 90 --axis dec
python3 devtools/make_test_image.py --codes 0 287 --noise 60  # robustness
```

Output is an 8-bit grayscale BMP (same format as `GET /bmp`). Every
image is self-checked before it is written: a Python mirror of the
intended decode pipeline for this layout (locate the strip's row
window by dark coverage per row over the code-band columns with a
run-height filter that rejects the thin marks and the pointer, strip
chosen nearest mid-height -> radial profile along X -> Otsu -> widest
dark run -> 11..13-cell guard -> 9 windows -> Gray) must recover the
requested codes, including the two-strips-in-view case (the full-frame
decode returns the strip nearest mid-height, where the pointer is
drawn; with that strip cropped away the decoder must lock onto the
other one). Validated: all 288 HA + 360 DEC codes decode at +/-40
luma noise, plus 120 random two-strip frames (cell sizes 18..24,
noise 0/20/40, both decodes per frame = 240/240 checks).
`example_test_band.bmp` is a pre-generated example (codes 25 and
150; the full-frame decode returns 25, the strip under the pointer,
and 150 after that strip is cropped away).

## Next steps

- Fine reading: overlay detection of the original long/short
  graduation marks inside the code cell currently under the crosshair,
  weighted centroid -> sub-cell interpolation (5 min -> arcseconds-ish).
- Band artwork generator (SVG/PDF at ring radius, one marker + 9 cells
  per block, 288/360 blocks) is a natural companion script.
- Optional: ESP32-CAM soft-AP fallback and NTP on the host side.
