#!/usr/bin/env python3
"""Generate a synthetic code-band test image (graduated-circle style).

Renders, per requested code, the band layout the ESP32-CAM decoder
expects, laid out on a white strip with a graduation tick ruler beneath
it -- in the style of the reference images from the original prompt:

  [2-cell white lead-in][12-cell dark marker][1-cell white gap]
  [9 Gray data cells, MSB first, dark=1][>=2-cell white gap]

The image is written as an 8-bit grayscale BMP (same format as the
camera's GET /bmp), and every generated image is self-checked: a Python
mirror of the ESP32 decode pipeline (row profile -> Otsu -> widest dark
run -> guard -> 9 windows -> Gray) must recover the requested codes
before the file is written. Two codes side by side also exercise the
"adjacent dark runs in view" case: the full-strip decode must return
the first code, and the strip after the first group must yield the
second.

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

# Vertical layout in rows (mimics the camera framing: a thin band strip
# around 45-55% of frame height, ruler ticks below).
TOP_MARGIN = 60
BAND_ROWS = 40          # matches the calibrated BAND_ROW_START..END width
RULER_GAP = 20
RULER_ROWS = 40
BOTTOM_MARGIN = 60


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
    """Return (pixels, w, h) of the test strip; pixels = list of rows."""
    # Two groups per image: each needs lead+marker+gap+9+tail cells.
    group_cells = LEAD_CELLS + MARKER_CELLS + GAP_CELLS + N_BITS + TAIL_CELLS
    groups = len(codes)
    inter_gap = 4 * cell          # white separation between the two groups
    left_margin = 2 * cell
    w = left_margin + groups * group_cells * cell + (groups - 1) * inter_gap \
        + 2 * cell
    h = TOP_MARGIN + BAND_ROWS + RULER_GAP + RULER_ROWS + BOTTOM_MARGIN
    band_top = TOP_MARGIN
    band_bot = band_top + BAND_ROWS

    pix = [[255] * w for _ in range(h)]

    x = left_margin
    group_spans = []
    for n in codes:
        g = gray_from_code(n)
        start = x
        # cells: lead (white), marker (dark), gap (white), data, tail
        pattern = [0] * LEAD_CELLS + [1] * MARKER_CELLS + [0] * GAP_CELLS
        pattern += [(g >> (N_BITS - 1 - i)) & 1 for i in range(N_BITS)]
        pattern += [0] * TAIL_CELLS
        for i, dark in enumerate(pattern):
            if dark:
                for xx in range(i * cell, (i + 1) * cell):
                    for yy in range(band_top, band_bot):
                        pix[yy][start + xx] = 0
        x += group_cells * cell + inter_gap
        group_spans.append((start - cell, start + group_cells * cell + cell))

    # Graduation tick ruler below the band: baseline + ticks, minor every
    # cell pitch, major every 5 cells, aligned with group boundaries.
    ruler_top = band_bot + RULER_GAP
    for xx in range(0, w):
        pix[ruler_top][xx] = 0
        pix[ruler_top + 1][xx] = 0
    for i in range(0, (w - left_margin) // cell + 2):
        xx = left_margin + i * cell
        if xx >= w - 1:
            break
        tick = 22 if i % 5 == 0 else 10
        for yy in range(ruler_top, min(ruler_top + tick, h)):
            pix[yy][xx] = 0
            if xx + 1 < w:
                pix[yy][xx + 1] = 0

    if noise:
        rng = random.Random(seed)
        for yy in range(h):
            row = pix[yy]
            for xx in range(w):
                v = row[xx] + rng.randint(-noise, noise)
                row[xx] = 0 if v < 0 else (255 if v > 255 else v)

    return pix, w, h, group_spans, band_top, band_bot


# ---- Python mirror of the ESP32 decode pipeline (self-check) -------------

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


def decode(pix, w, rows, cal_cell_px):
    """Mirror of decodeCircle(): band row profile -> code index, or None."""
    r_start, r_end = rows
    prof = []
    for x in range(w):
        acc = sum(pix[y][x] for y in range(r_start, r_end))
        prof.append(acc // (r_end - r_start))
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
        x0 = band_start + int(i * pitch + 0.5)
        x1 = band_start + int((i + 1) * pitch + 0.5)
        if x0 < 0 or x1 <= x0 or x1 > len(prof):
            return None
        avg = sum(prof[x0:x1]) / (x1 - x0)
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

    pix, w, h, spans, band_top, band_bot = render(
        args.codes, args.cell, args.cal_cell_px, args.noise, args.seed)

    # Self-check 1: full strip decodes to the FIRST code (both markers are
    # equally wide; the decoder keeps the first widest run).
    got = decode(pix, w, (band_top, band_bot), args.cal_cell_px)
    ok1 = got == args.codes[0]
    # Self-check 2: the strip after the first group decodes to the second.
    ok2 = True
    if len(args.codes) > 1:
        cut = spans[0][1]
        sub = [row[cut:] for row in pix]
        got2 = decode(sub, w - cut, (band_top, band_bot), args.cal_cell_px)
        ok2 = got2 == args.codes[1]

    status = "ok" if (ok1 and ok2) else "FAILED"
    print("image %dx%d, self-check: %s" % (w, h, status))
    print("  full strip decode  -> %s (want %d) %s"
          % (got, args.codes[0], "ok" if ok1 else "FAIL"))
    if len(args.codes) > 1:
        print("  cropped decode     -> %s (want %d) %s"
              % (got2, args.codes[1], "ok" if ok2 else "FAIL"))
    for n in args.codes:
        print("  code %3d = %s (%s axis)" % (n, code_value(n, args.axis),
                                              args.axis.upper()))
    if status != "ok":
        sys.exit(1)
    write_bmp(args.out, pix, w, h)
    print("wrote %s (%d bytes)" % (args.out, os.path.getsize(args.out)))


if __name__ == "__main__":
    main()
