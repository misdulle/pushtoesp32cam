#!/usr/bin/env python3
"""Generate a synthetic code-band test image (graduated-circle style).

Measured on the real instrument (and confirmed): the mobile part moves
VERTICALLY through the field of view -- the tangential direction is the
camera's Y axis, the radial direction is X. The FOV has two parts:

- Mobile part (the circle), in two radial bands side by side:
  - LEFT: the code strips. Each strip is 27 cells laid out ALONG THE
    RADIUS (left to right: [2-cell lead-in][12-cell dark marker]
    [1-cell gap][9 Gray data cells, MSB nearest the marker][3-cell
    tail]), one cell tall. Strips stack vertically, one per 5-min
    graduation, each ALIGNED with its long (5-min) mark: same height,
    codes only put a number on that mark.
  - RIGHT: the graduated marks, anchored at the rim: long (5-min,
    70% of the band, at each strip's height) and short (1-min, 40%,
    at the fifths between).
- Fixed part: a single thin pointer line, HORIZONTAL (parallel to the
  marks), at a fixed height (mid-FOV here). It starts at the rim --
  the marks' anchored right ends -- and extends rightward to the FOV
  edge; it never crosses the code band. The fine reading is the
  pointer height interpolated between graduated lines.

The image is written as an 8-bit grayscale BMP (same format as the
camera's GET /bmp), and every generated image is self-checked by a
Python mirror of the intended ESP32 decode pipeline for this layout:
per-row dark coverage over the code-band columns (width-filtered so
the thin marks are rejected; the pointer never enters the code band),
strip chosen nearest mid-height (where the pointer sits), radial
profile along the strip's rows, then
Otsu -> widest dark run (12-cell marker) -> guard -> 9 windows -> Gray.
Two strips in one frame exercise the "adjacent strips in view" case:
the full-frame decode returns the strip nearest mid-height (the one
at the pointer's height); with that strip cropped away the decoder
must lock onto the other one.

Examples:
  python3 make_test_image.py                        # codes 25 and 150
  python3 make_test_image.py --codes 0 287 --cell 24
  python3 make_test_image.py --codes 90 --axis dec  # prints DEC value
  python3 make_test_image.py --noise 40             # robustness check
"""

import argparse
import random
import struct
import sys
import os

MARKER_CELLS = 12
GAP_CELLS = 1
LEAD_CELLS = 2
TAIL_CELLS = 3
N_BITS = 9
GROUP_CELLS = LEAD_CELLS + MARKER_CELLS + GAP_CELLS + N_BITS + TAIL_CELLS

# Geometry: tangential direction = vertical (rows), radial = columns.
# LEFT radial band = code strips, RIGHT radial band = graduated marks.
TOP_MARGIN = 40
BOTTOM_MARGIN = 40
LEFT_MARGIN_CELLS = 2
GAP_TO_TICS_CELLS = 2
RIGHT_MARGIN_CELLS = 2
TIC_BAND_CELLS = 12       # radial width of the graduated-marks band
SECTOR_CELLS = 8          # tangential sector pitch (1 code = 5 min)
MARK_WIDTH = 3
POINTER_WIDTH = 3
MARK_FRAC_SHORT = 0.4     # 1-min marks: 40% of the tic band
MARK_FRAC_LONG = 0.7      # 5-min marks (these carry the codes)


def gray_from_code(n):
    """Code index -> 9-bit Gray word (binary reflected Gray code)."""
    return (n ^ (n >> 1)) & ((1 << N_BITS) - 1)


def code_value(n, axis):
    """Code index -> displayed axis value, as angleFromCode() in the sketch."""
    if axis == "ha":
        h = n * 5 / 60.0  # 288 codes x 5 min-of-time
        return "%.4f h" % h
    d = -90.0 + n * 0.5  # 360 codes x 0.5 deg
    return "%.4f deg" % d


def render(codes, cell, cal_cell_px, noise=0, seed=1):
    """Return (pix, w, h, strip_rows, band_cols).

    strip_rows[k] = (y0, y1) of code k's strip; band_cols = (x0, x1) of
    the code band (the strips' radial extent).
    """
    left = LEFT_MARGIN_CELLS * cell
    band_x0 = left
    band_x1 = left + GROUP_CELLS * cell
    tics_x1 = band_x1 + GAP_TO_TICS_CELLS * cell + TIC_BAND_CELLS * cell
    w = tics_x1 + RIGHT_MARGIN_CELLS * cell
    sector = SECTOR_CELLS * cell
    h = TOP_MARGIN + len(codes) * sector + BOTTOM_MARGIN

    pix = [[255] * w for _ in range(h)]

    def hline(y0, x0, x1, width=MARK_WIDTH):
        for yy in range(max(0, y0), min(h, y0 + width)):
            row = pix[yy]
            for xx in range(max(0, x0), min(w, x1)):
                row[xx] = 0

    strip_rows = []
    band_w = TIC_BAND_CELLS * cell
    for k, n in enumerate(codes):
        yc = TOP_MARGIN + k * sector + sector // 2   # strip center = long mark
        y0 = yc - cell // 2
        y1 = y0 + cell
        strip_rows.append((y0, y1))

        # Code strip: GROUP_CELLS cells side by side along the radius.
        g = gray_from_code(n)
        pattern = [0] * LEAD_CELLS + [1] * MARKER_CELLS + [0] * GAP_CELLS
        pattern += [(g >> (N_BITS - 1 - i)) & 1 for i in range(N_BITS)]
        pattern += [0] * TAIL_CELLS
        for i, dark in enumerate(pattern):
            if dark:
                hline(y0, band_x0 + i * cell, band_x0 + (i + 1) * cell, cell)

        # Long (5-min) mark at the strip's height, short (1-min) marks
        # at the fifths below it (until the next strip's mark).
        hline(yc - MARK_WIDTH // 2, tics_x1 - int(MARK_FRAC_LONG * band_w),
              tics_x1)
        for j in range(1, 5):
            ys = yc + j * sector // 5
            hline(ys - MARK_WIDTH // 2,
                  tics_x1 - int(MARK_FRAC_SHORT * band_w), tics_x1)

    # Fixed part: one thin pointer line at mid-height, from the rim
    # (the marks' anchored right ends) rightward to the FOV edge --
    # parallel to the marks, and it never crosses the code band.
    yp = h // 2
    hline(yp - POINTER_WIDTH // 2, tics_x1, w, POINTER_WIDTH)

    if noise:
        rng = random.Random(seed)
        for yy in range(h):
            row = pix[yy]
            for xx in range(w):
                v = row[xx] + rng.randint(-noise, noise)
                row[xx] = 0 if v < 0 else (255 if v > 255 else v)

    return pix, w, h, strip_rows, (band_x0, band_x1)


# ---- Python mirror of the intended ESP32 decode pipeline (self-check) ----

def otsu(hist, total):
    sum_all = float(sum(i * hist[i] for i in range(256)))
    sum_b = 0.0
    w_b = 0
    w_f = total
    max_var = -1.0
    best = 127
    for t in range(256):
        w_b += hist[t]
        if w_b == 0:
            continue
        w_f = total - w_b
        if w_f == 0:
            break
        sum_b += t * hist[t]
        m_b = sum_b / w_b
        m_f = (sum_all - sum_b) / w_f
        var = w_b * w_f * (m_b - m_f) ** 2
        if var > max_var:
            max_var = var
            best = t
    return best


def gray_to_binary(g):
    g ^= g >> 8      # 9-bit word: propagate MSB first, then halve shifts
    g ^= g >> 4
    g ^= g >> 2
    g ^= g >> 1
    return g & ((1 << N_BITS) - 1)


def decode(pix, w, h, band_cols, cal_cell_px):
    """Radial-strip decode: locate the strip's row window (coverage per
    row over the code-band columns, run nearest mid-height), then run
    the 1-D pipeline along the radius. Returns the code index or None."""
    x0, x1 = band_cols
    ncols = x1 - x0

    hist = [0] * 256
    for y in range(h):
        row = pix[y]
        for x in range(x0, x1):
            hist[row[x]] += 1
    thr = otsu(hist, ncols * h)

    # Strip rows: dark coverage per row within the code band. The
    # marker alone (code 0, no dark data cells) gives 12/27 = 0.44 of
    # the strip; the thin pointer line is rejected by the height filter.
    min_cov = 0.30 * ncols
    min_h = max(1, int(0.7 * cal_cell_px))
    rowcov = [0] * h
    for y in range(h):
        row = pix[y]
        c = 0
        for x in range(x0, x1):
            if row[x] <= thr:
                c += 1
        rowcov[y] = c

    ys = ye = -1
    best_dist = -1
    rs = -1
    for y in range(h + 1):
        good = y < h and rowcov[y] >= min_cov
        if good and rs < 0:
            rs = y
        elif not good and rs >= 0:
            if y - rs >= min_h:
                dist = abs((rs + y) // 2 - h // 2)
                if best_dist < 0 or dist < best_dist:
                    best_dist = dist
                    ys, ye = rs, y
            rs = -1
    if ys < 0:
        return None

    # Radial profile along the strip's rows.
    prof = []
    sr = ye - ys
    for x in range(x0, x1):
        acc = 0
        for y in range(ys, ye):
            acc += pix[y][x]
        prof.append(acc // sr)

    hist = [0] * 256
    for v in prof:
        hist[v] += 1
    thr = otsu(hist, len(prof))
    dark = [v for v in prof if v <= thr]
    white = [v for v in prof if v > thr]
    bit_thr = thr
    if dark and white:
        bit_thr = 0.5 * (sum(dark) / len(dark) + sum(white) / len(white))

    best_start, best_w, cur_start = -1, 0, -1
    for i in range(len(prof) + 1):
        is_dark = i < len(prof) and prof[i] <= thr
        if is_dark and cur_start < 0:
            cur_start = i
        elif not is_dark and cur_start >= 0:
            if i - cur_start > best_w:
                best_w, best_start = i - cur_start, cur_start
            cur_start = -1
    if best_start < 0 or best_w < 11 * cal_cell_px or best_w > 13 * cal_cell_px:
        return None
    pitch = best_w / MARKER_CELLS
    band_start = best_start + best_w + int(GAP_CELLS * pitch + 0.5)
    gray = 0
    for i in range(N_BITS):
        xa = band_start + int(i * pitch + 0.5)
        xb = band_start + int((i + 1) * pitch + 0.5)
        if xa < 0 or xb <= xa or xb > len(prof):
            return None
        avg = sum(prof[xa:xb]) / (xb - xa)
        if avg <= bit_thr:
            gray |= 1 << (N_BITS - 1 - i)
    return gray_to_binary(gray)


def write_bmp(path, pix, w, h):
    row_size = (w + 3) & ~3
    data_off = 54 + 1024
    file_size = data_off + row_size * h
    with open(path, "wb") as f:
        f.write(b"BM")
        f.write(struct.pack("<IHHI", file_size, 0, 0, data_off))
        f.write(struct.pack("<IiiHHIIiiII", 40, w, h, 1, 8, 0,
                            row_size * h, 0, 0, 256, 0))
        for i in range(256):
            f.write(bytes((i, i, i, 0)))
        pad = b"\0" * (row_size - w)
        for y in range(h - 1, -1, -1):     # bottom-up rows
            f.write(bytes(pix[y]))
            f.write(pad)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--codes", nargs="+", type=int, default=[25, 150],
                    help="code indexes to render (default: 25 150)")
    ap.add_argument("--axis", choices=("ha", "dec"), default="ha",
                    help="axis for the printed value mapping")
    ap.add_argument("--cell", type=int, default=24,
                    help="cell size in pixels (default 24)")
    ap.add_argument("--cal-cell-px", type=int, default=24,
                    help="CAL_CELL_PX assumed by the decoder self-check")
    ap.add_argument("--noise", type=int, default=0,
                    help="add +/-N luma noise (robustness check)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="test_band.bmp")
    args = ap.parse_args()

    for n in args.codes:
        if not 0 <= n < (288 if args.axis == "ha" else 360):
            ap.error("code %d out of range for axis %s" % (n, args.axis))

    pix, w, h, strip_rows, band_cols = render(
        args.codes, args.cell, args.cal_cell_px, args.noise, args.seed)

    # The fixed pointer sits at mid-height; the decoder reads the strip
    # nearest to it. Compute which rendered strip that is.
    centers = [(y0 + y1) / 2.0 for (y0, y1) in strip_rows]
    mid = h // 2
    near = min(range(len(centers)), key=lambda k: abs(centers[k] - mid))

    # Self-check 1: full frame decodes to the strip nearest mid-height.
    got = decode(pix, w, h, band_cols, args.cal_cell_px)
    ok1 = got == args.codes[near]
    # Self-check 2: with the nearest strip cropped away, the decoder
    # must lock onto the other one (adjacent strips in view).
    ok2 = True
    got2 = None
    if len(args.codes) > 1:
        other = 1 - near if len(args.codes) == 2 else (0 if near else 1)
        if centers[near] > centers[other]:
            cut = strip_rows[near][0]       # nearest strip at the bottom
            sub = pix[:cut]
            h2 = cut
        else:
            cut = strip_rows[near][1]       # nearest strip at the top
            sub = pix[cut:]
            h2 = h - cut
        got2 = decode(sub, w, h2, band_cols, args.cal_cell_px)
        ok2 = got2 == args.codes[other]

    status = "ok" if (ok1 and ok2) else "FAILED"
    print("image %dx%d, self-check: %s" % (w, h, status))
    print("  full frame decode -> %s (strip %d, want %d) %s"
          % (got, near, args.codes[near], "ok" if ok1 else "FAIL"))
    if len(args.codes) > 1:
        print("  cropped decode    -> %s (strip %d, want %d) %s"
              % (got2, other, args.codes[other], "ok" if ok2 else "FAIL"))
    for n in args.codes:
        print("  code %3d = %s (%s axis)" % (n, code_value(n, args.axis),
                                              args.axis.upper()))
    if status != "ok":
        sys.exit(1)
    write_bmp(args.out, pix, w, h)
    print("wrote %s (%d bytes)" % (args.out, os.path.getsize(args.out)))


if __name__ == "__main__":
    main()
