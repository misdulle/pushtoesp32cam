#!/usr/bin/env python3
"""Generate a synthetic code-band test image (graduated-circle style).

The field of view has two parts, as on the real instrument:

- Mobile part (the circle): radial graduated marks -- long (hourly),
  medium (5 min: one code each) and short (1 min) lines -- plus the
  code strips. Every code strip is a radial band running PARALLEL to
  the graduated bars: its cells are stacked one after another ALONG
  the radius ([lead-in][12-cell marker][gap][9 Gray cells][tail]),
  one cell wide tangentially. The codes only put a number on the
  5-min medium marks (each strip sits next to its mark).
- Fixed part: a single black pointer line, drawn centered in the FOV
  at mid-X (the decoder reads the strip nearest to it) -- the fine
  reading comes from interpolating the pointer position between
  graduated lines.

The image is written as an 8-bit grayscale BMP (same format as the
camera's GET /bmp), and every generated image is self-checked by a
Python mirror of the intended ESP32 decode pipeline for this layout:
find the strip's column window (dark coverage per column, width
filtered so thin marks/pointer are rejected, strip chosen nearest
the FOV mid-line where the fixed pointer sits), build the radial
profile, then Otsu -> widest dark run (12-cell marker) -> guard ->
9 windows -> Gray. Two strips in one frame exercise the "adjacent
strips in view" case: the full-frame decode returns the strip
nearest mid-X (where the pointer is drawn); with that strip
cropped away the decoder must lock onto the other one.

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

# Geometry: radial direction = vertical (rows). Everything on the mobile
# part is a radial strip; the fixed pointer is a radial line too.
TOP_MARGIN = 40
BOTTOM_MARGIN = 40
SECTOR_CELLS = 8           # tangential sector pitch (1 code = 5 min)
STRIP_OFFSET_CELLS = 3     # strip position inside its sector
POINTER_SECTOR_FRAC = 1.0  # fixed pointer: centered in the FOV (mid-X)
MARK_WIDTH = 3
POINTER_WIDTH = 3
MARK_FRAC_SHORT = 0.4      # 1-min marks: 40% of the radial band
MARK_FRAC_MEDIUM = 0.7     # 5-min marks (these carry the codes)
SECTORS_PER_LONG = 12      # hourly long mark (12 x 5 min = 1 h)


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
    """Return (pixels, w, h, strip_spans, strip_top, strip_bot)."""
    groups = len(codes)
    sector = SECTOR_CELLS * cell
    left = 2 * cell
    w = left + groups * sector + 2 * cell
    h = TOP_MARGIN + GROUP_CELLS * cell + BOTTOM_MARGIN
    top = TOP_MARGIN
    bot = top + GROUP_CELLS * cell

    pix = [[255] * w for _ in range(h)]

    strip_ranges = []
    for k in range(groups):
        xs = left + k * sector + STRIP_OFFSET_CELLS * cell
        strip_ranges.append((xs, xs + cell))

    def vline(x0, y0, y1, width=MARK_WIDTH):
        for xx in range(max(0, x0), min(w, x0 + width)):
            for yy in range(max(0, y0), min(h, y1)):
                pix[yy][xx] = 0

    def overlaps_strip(x0, x1):
        return any(not (x1 <= a or x0 >= b) for a, b in strip_ranges)

    # Mobile part, graduated marks: thin radial lines, parallel to the
    # strips. Medium (5-min) at each sector start, short (1-min) at the
    # fifths, long (hourly) every SECTORS_PER_LONG sectors.
    n_sect = (w - left) // sector + 1
    for k in range(n_sect):
        xm = left + k * sector
        vline(xm, top, top + int(MARK_FRAC_MEDIUM * (bot - top)))
        for j in range(1, 5):
            xs = xm + j * sector // 5
            if not overlaps_strip(xs, xs + MARK_WIDTH):
                vline(xs, top, top + int(MARK_FRAC_SHORT * (bot - top)))
        if k % SECTORS_PER_LONG == 0:
            vline(xm, top, bot)

    # Mobile part, code strips: GROUP_CELLS cells stacked along the
    # radius, one cell wide, next to their 5-min mark.
    spans = []
    for idx, n in enumerate(codes):
        g = gray_from_code(n)
        pattern = [0] * LEAD_CELLS + [1] * MARKER_CELLS + [0] * GAP_CELLS
        pattern += [(g >> (N_BITS - 1 - i)) & 1 for i in range(N_BITS)]
        pattern += [0] * TAIL_CELLS
        xs = strip_ranges[idx][0]
        for i, dark in enumerate(pattern):
            if dark:
                for yy in range(i * cell, (i + 1) * cell):
                    row = pix[top + yy]
                    for xx in range(xs, xs + cell):
                        row[xx] = 0
        spans.append((xs - cell, xs + 2 * cell))

    # Fixed part: one pointer line centered in the FOV (mid-X, where
    # the decoder looks for the current strip), extending past the
    # mobile band at both ends. Being MARK_WIDTH-thin it is rejected
    # by the decoder's strip-width filter when between strips.
    xp = left + int(POINTER_SECTOR_FRAC * sector)
    vline(xp, TOP_MARGIN // 2, bot + BOTTOM_MARGIN // 2, POINTER_WIDTH)

    if noise:
        rng = random.Random(seed)
        for yy in range(h):
            row = pix[yy]
            for xx in range(w):
                v = row[xx] + rng.randint(-noise, noise)
                row[xx] = 0 if v < 0 else (255 if v > 255 else v)

    return pix, w, h, spans, top, bot


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


def decode(pix, w, rows, cal_cell_px):
    """Radial-strip decode: locate the strip's column window, then run
    the 1-D pipeline along the radius. Returns the code index or None."""
    r0, r1 = rows
    nrows = r1 - r0

    hist = [0] * 256
    for y in range(r0, r1):
        for v in pix[y]:
            hist[v] += 1
    thr = otsu(hist, nrows * w)

    # Strip columns: dark coverage per column within the band rows.
    # The marker alone (code 0, no dark data cells) gives 12/27 = 0.44;
    # thin marks and the pointer are rejected by the width filter.
    min_cov = 0.40 * nrows
    min_w = max(1, int(0.7 * cal_cell_px))
    colcov = [0] * w
    for y in range(r0, r1):
        row = pix[y]
        for x in range(w):
            if row[x] <= thr:
                colcov[x] += 1

    xs = xe = -1
    best_dist = -1
    cs = -1
    for x in range(w + 1):
        good = x < w and colcov[x] >= min_cov
        if good and cs < 0:
            cs = x
        elif not good and cs >= 0:
            if x - cs >= min_w:
                dist = abs((cs + x) // 2 - w // 2)
                if best_dist < 0 or dist < best_dist:
                    best_dist = dist
                    xs, xe = cs, x
            cs = -1
    if xs < 0:
        return None

    # Radial profile across the strip's columns.
    prof = []
    for y in range(r0, r1):
        acc = 0
        for x in range(xs, xe):
            acc += pix[y][x]
        prof.append(acc // (xe - xs))

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
        y0 = band_start + int(i * pitch + 0.5)
        y1 = band_start + int((i + 1) * pitch + 0.5)
        if y0 < 0 or y1 <= y0 or y1 > len(prof):
            return None
        avg = sum(prof[y0:y1]) / (y1 - y0)
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

    pix, w, h, spans, top, bot = render(
        args.codes, args.cell, args.cal_cell_px, args.noise, args.seed)

    # The fixed pointer sits at mid-X; the decoder reads the strip
    # nearest to it. Compute which rendered strip that is.
    centers = [spans[k][0] + 1.5 * args.cell for k in range(len(args.codes))]
    mid = w // 2
    near = min(range(len(centers)), key=lambda k: abs(centers[k] - mid))

    # Self-check 1: full frame decodes to the strip nearest mid-X.
    got = decode(pix, w, (top, bot), args.cal_cell_px)
    ok1 = got == args.codes[near]
    # Self-check 2: with the nearest strip cropped away, the decoder
    # must lock onto the other one (adjacent strips in view).
    ok2 = True
    got2 = None
    if len(args.codes) > 1:
        other = 1 - near if len(args.codes) == 2 else (0 if near else 1)
        if centers[near] > centers[other]:
            cut = spans[near][0]            # nearest strip on the right
            sub = [row[:cut] for row in pix]
            w2 = cut
        else:
            cut = spans[near][1]            # nearest strip on the left
            sub = [row[cut:] for row in pix]
            w2 = w - cut
        got2 = decode(sub, w2, (top, bot), args.cal_cell_px)
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
