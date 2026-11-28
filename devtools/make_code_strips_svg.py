#!/usr/bin/env python3
"""Generate printable code-strip sheets (SVG, A4) for the graduated circles.

Modelled on the user's i25 sheet (final_backup.svg): codes are printed
on long strips so that ~20 strips are glued around the circle instead
of 288 individual codes.

Strip anatomy (width = radial direction, as the camera sees it):

  [1.5 mm inner margin][27-cell pattern][0.5 mm outer margin]

  pattern = [2 lead][12-cell marker][1 gap][9 Gray data, MSB first][3 tail]
  The marker end is the INNER end (toward the circle centre); the MSB
  data cell is the one nearest the marker. Lengthwise a strip is
  exactly n x SECTOR mm with row centres at (k+1/2) x SECTOR, so
  butt-joined strips keep the exact pitch across the joints. A 6 mm
  label tab (cut off before gluing) carries the strip number and its
  code range. The 0.5 mm outer margin must stay clear of the existing
  graduation marks when gluing.

Standard reference: code pitch = SECTOR mm (7.4 mm = 5 min on the HA
circle; pass --sector-mm for DEC). Print at 100 % (no scaling); each
sheet carries a 74 mm check bar (10 pitches).

Examples:
  python3 make_code_strips_svg.py                    # HA, sketch cell 0.5265 mm
  python3 make_code_strips_svg.py --cell-mm 0.40     # compact (rotated camera)
  python3 make_code_strips_svg.py --axis dec --strips 30
"""

import argparse
import math
import os
import sys

BITS = 9
MARKER_CELLS = 12
GAP_CELLS = 1
LEAD_CELLS = 2
TAIL_CELLS = 3
GROUP_CELLS = LEAD_CELLS + MARKER_CELLS + GAP_CELLS + BITS + TAIL_CELLS  # 27

# Sheet layout (mm)
PAGE_W, PAGE_H = 210.0, 297.0
COLS = 10                     # strips per page row
COL_W = 20.0                  # column pitch
INNER_MARGIN = 1.5            # radial, marker end
OUTER_MARGIN = 0.5            # radial, rim end (keep clear of the marks)
TAB_H = 6.0                   # label tab, cut off before gluing
STRIP_HEIGHT_MM = 2.046       # tangential height of one code row (geometry.svg)
ROW_LABEL_FONT = 1.0
TOP_Y = 10.0                  # first cut line
ROW_GAP = 6.0                 # between page rows

# geometry.svg references
CELL_MM_STD = 0.526467        # sketch: space_for_code width / 27
SECTOR_MM_STD = 7.4           # 5-min spacing on the HA circle


def gray_from_code(n):
    return (n ^ (n >> 1)) & ((1 << BITS) - 1)


def pattern_runs(n):
    """Code index -> list of (start_cell, n_cells) dark runs in the
    27-cell pattern (cells merged into runs for crisp printing)."""
    g = gray_from_code(n)
    cells = [0] * LEAD_CELLS + [1] * MARKER_CELLS + [0] * GAP_CELLS
    cells += [(g >> (BITS - 1 - i)) & 1 for i in range(BITS)]
    cells += [0] * TAIL_CELLS
    runs = []
    start = None
    for i, d in enumerate(cells + [0]):
        if d and start is None:
            start = i
        elif not d and start is not None:
            runs.append((start, i - start))
            start = None
    return runs


def distribute(n_codes, n_strips):
    """Codes per strip, as even as possible (first strips get the extra)."""
    base, rem = divmod(n_codes, n_strips)
    return [base + 1 if i < rem else base for i in range(n_strips)]


class Sheet:
    """One A4 page holding up to ROWS x COLS strips."""

    def __init__(self):
        self.body = []
        self.strip_rects = []   # (strip_idx, code, x, y, w, h) dark cells, mm
        self.strip_geo = {}     # strip_idx -> (x0, y0, pattern_w, n)

    def add(self, markup):
        self.body.append(markup)

    def svg(self, title):
        return ('\n'.join([
            '<?xml version="1.0" encoding="UTF-8"?>',
            '<svg xmlns="http://www.w3.org/2000/svg" '
            'width="%.0fmm" height="%.0fmm" viewBox="0 0 %g %g">' % (
                PAGE_W, PAGE_H, PAGE_W, PAGE_H),
            '<rect x="0" y="0" width="%g" height="%g" fill="white"/>' % (
                PAGE_W, PAGE_H),
            '<!-- %s -->' % title,
        ]) + '\n' + '\n'.join(self.body) + '\n</svg>\n')


def build_sheets(args):
    per_strip = distribute(args.codes, args.strips)
    cell = args.cell_mm
    pattern_w = GROUP_CELLS * cell
    strip_w = INNER_MARGIN + pattern_w + OUTER_MARGIN
    if strip_w > COL_W - 1.0:
        raise SystemExit('strip width %.2f mm does not fit the %.1f mm column'
                         % (strip_w, COL_W))
    left0 = (PAGE_W - COLS * COL_W) / 2.0

    sheets = []
    n_pages = int(math.ceil(args.strips / float(COLS * 2)))
    strip_idx = 0
    for page in range(n_pages):
        sh = Sheet()
        page_rows = []          # (tab_y, strips_y, row_h) per non-empty row
        for r in range(2):
            row_strips = []
            for c in range(COLS):
                if strip_idx >= args.strips:
                    break
                row_strips.append((c, strip_idx))
                strip_idx += 1
            if not row_strips:
                continue
            row_h = max(per_strip[si] for (_, si) in row_strips) \
                * args.sector_mm
            if page_rows:
                prev = page_rows[-1]
                tab_y = prev[1] + prev[2] + ROW_GAP
            else:
                tab_y = TOP_Y
            strips_y = tab_y + TAB_H
            page_rows.append((tab_y, strips_y, row_h))
            for (c, si) in row_strips:
                n = per_strip[si]
                x0 = left0 + c * COL_W + (COL_W - strip_w) / 2.0
                y0 = strips_y
                sh.strip_geo[si] = (x0, y0, pattern_w, n)
                # tab: cut line under it, label, IN arrow
                sh.add('<line x1="%g" y1="%g" x2="%g" y2="%g" '
                       'stroke="black" stroke-width="0.25"/>' % (
                           x0 - 1, y0 + TAB_H, x0 + strip_w + 1, y0 + TAB_H))
                c0 = sum(per_strip[:si])   # first code index on this strip
                c1 = c0 + n - 1
                sh.add('<text x="%g" y="%g" font-size="2.2" '
                       'font-family="Helvetica, Arial, sans-serif" '
                       'text-anchor="start" fill="black">S%02d  codes %03d-%03d'
                       '</text>' % (x0, y0 + TAB_H - 1.8, si + 1, c0, c1))
                # IN arrow (triangle pointing to the inner/marker end)
                ay = y0 + TAB_H - 1.2
                sh.add('<polygon points="%g,%g %g,%g %g,%g" fill="black"/>' % (
                    x0 + 11.5, ay - 1.1, x0 + 11.5, ay + 1.1, x0 + 10.0, ay))
                sh.add('<text x="%g" y="%g" font-size="1.6" '
                       'font-family="Helvetica, Arial, sans-serif" '
                       'text-anchor="start" fill="black">IN</text>' % (
                           x0 + 12.0, ay + 0.55))
                # code rows
                for k in range(n):
                    yc = y0 + (k + 0.5) * args.sector_mm
                    for (cs, cn) in pattern_runs(c0 + k):
                        rx = x0 + INNER_MARGIN + cs * cell
                        rw = cn * cell
                        sh.add('<rect x="%g" y="%g" width="%g" height="%g" '
                               'fill="black"/>' % (
                                   rx, yc - STRIP_HEIGHT_MM / 2.0, rw,
                                   STRIP_HEIGHT_MM))
                        sh.strip_rects.append(
                            (si, c0 + k, rx, yc - STRIP_HEIGHT_MM / 2.0, rw,
                             STRIP_HEIGHT_MM))
                    # tiny row label + centre ticks in the margins
                    sh.add('<text x="%g" y="%g" font-size="%g" '
                           'font-family="Helvetica, Arial, sans-serif" '
                           'text-anchor="middle" fill="#999999" '
                           'transform="rotate(-90 %g %g)">%d</text>' % (
                               x0 + INNER_MARGIN / 2.0, yc + 0.35,
                               ROW_LABEL_FONT, x0 + INNER_MARGIN / 2.0,
                               yc + 0.35, c0 + k))
                    for tx in (x0 + INNER_MARGIN - 0.7,
                               x0 + INNER_MARGIN + pattern_w + 0.25):
                        sh.add('<rect x="%g" y="%g" width="0.15" '
                               'height="0.7" fill="black"/>' % (
                                   tx, yc - 0.35))
            # horizontal cut line below the strip rows of this page row
            sh.add('<line x1="%g" y1="%g" x2="%g" y2="%g" '
                   'stroke="black" stroke-width="0.25"/>' % (
                       left0 - 2, strips_y + row_h, PAGE_W - left0 + 2,
                       strips_y + row_h))
        # vertical cut lines across both page rows
        y_top = page_rows[0][0]
        y_bot = page_rows[-1][1] + page_rows[-1][2]
        for c in range(COLS + 1):
            x = left0 + c * COL_W
            sh.add('<line x1="%g" y1="%g" x2="%g" y2="%g" '
                   'stroke="black" stroke-width="0.25"/>' % (x, y_top, x, y_bot))
        # scale check bar: 10 pitches
        bar_y = y_bot + 10.0
        sh.add('<rect x="%g" y="%g" width="%g" height="0.6" fill="black"/>' % (
            left0, bar_y, 10 * args.sector_mm))
        for i in range(11):
            sh.add('<rect x="%g" y="%g" width="0.3" height="2" '
                   'fill="black"/>' % (left0 + i * args.sector_mm, bar_y - 0.7))
        sh.add('<text x="%g" y="%g" font-size="2.5" '
               'font-family="Helvetica, Arial, sans-serif" fill="black">'
               '%.2f mm = 10 code pitches - verify print scale (100 %%). '
               'cell %.4f mm, %d bits Gray, %d codes on %d strips.</text>' % (
                   left0, bar_y + 6.0, 10 * args.sector_mm, args.cell_mm,
                   BITS, args.codes, args.strips))
        sheets.append(sh)
    return per_strip, sheets


def raster_selftest(per_strip, sheets, args):
    """Rasterize sampled code rows and decode them with the Python
    mirror of the ESP32 pipeline (from make_test_image.py)."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from make_test_image import decode
    cell_px = 19
    ppm = cell_px / args.cell_mm
    pattern_w = GROUP_CELLS * args.cell_mm
    win_w = int(round((pattern_w + 2.0) * ppm))
    win_h = int(round(2.4 * args.sector_mm * ppm))
    all_rects = [r for sh in sheets for r in sh.strip_rects]
    codes = sorted(set(code for (_, code, *_ ) in all_rects))
    sample = codes[::max(1, len(codes) // 24)] + [codes[-1]]
    fails = []
    for want in sample:
        # anchor the window on the marker run's LEFT edge: pattern starts
        # LEAD_CELLS cells before it, so the band columns below map 1:1
        # onto the 27 pattern cells
        mr = next(r for r in all_rects
                  if r[1] == want and r[4] > MARKER_CELLS * args.cell_mm * 0.9)
        _, _, mx, my, mw, mh = mr
        cx, cy = mx + mw / 2.0, my + mh / 2.0
        x0 = mx - LEAD_CELLS * args.cell_mm - 1.0   # mm, window left edge
        y0 = cy - win_h / 2.0 / ppm                 # mm
        pix = [[255] * win_w for _ in range(win_h)]
        for (_, code, rx, ry, rw, rh) in all_rects:
            if code != want:
                continue
            px0 = int(round((rx - x0) * ppm))
            py0 = int(round((ry - y0) * ppm))
            px1 = int(round((rx + rw - x0) * ppm))
            py1 = int(round((ry + rh - y0) * ppm))
            for yy in range(max(0, py0), min(win_h, py1)):
                row = pix[yy]
                for xx in range(max(0, px0), min(win_w, px1)):
                    row[xx] = 0
        band = (int(round(1.0 * ppm)), int(round((1.0 + pattern_w) * ppm)))
        got = decode(pix, win_w, win_h, band, cell_px)
        if got != want:
            fails.append((want, got))
    print('raster selftest: %d rows sampled (cell %d px), failures: %d'
          % (len(sample), cell_px, len(fails)))
    for f in fails[:10]:
        print('   code want=%d got=%s' % (f[0], f[1]))
    return not fails


def main():
    global BITS
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--axis', choices=('ha', 'dec'), default='ha')
    ap.add_argument('--codes', type=int, default=None,
                    help='code count (default 288 ha / 360 dec)')
    ap.add_argument('--bits', type=int, default=BITS)
    ap.add_argument('--cell-mm', type=float, default=CELL_MM_STD,
                    help='radial cell pitch (default %.6f = geometry.svg)'
                         % CELL_MM_STD)
    ap.add_argument('--sector-mm', type=float, default=SECTOR_MM_STD,
                    help='code pitch = standard reference (default %.2f mm '
                         '= 5 min on the HA circle)' % SECTOR_MM_STD)
    ap.add_argument('--strips', type=int, default=20)
    ap.add_argument('--out', default=None,
                    help='output SVG path (default: '
                         '<axis>_code_strips_<cell>.svg in devtools/)')
    ap.add_argument('--no-selftest', action='store_true')
    args = ap.parse_args()
    BITS = args.bits
    if args.codes is None:
        args.codes = 288 if args.axis == 'ha' else 360

    per_strip, sheets = build_sheets(args)

    # geometry sanity checks
    assert sum(per_strip) == args.codes
    assert min(per_strip) > 0
    pattern_w = GROUP_CELLS * args.cell_mm
    page_h_needed = TOP_Y + 2 * TAB_H + 2 * max(per_strip) * args.sector_mm \
        + ROW_GAP + 20
    assert page_h_needed < PAGE_H, 'page overflow: %.1f mm' % page_h_needed

    out = args.out
    if out is None:
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '%s_code_strips_%.4fmm.svg'
                           % (args.axis, args.cell_mm))
    if len(sheets) == 1:
        paths = [(out, sheets[0])]
    else:
        base, ext = os.path.splitext(out)
        paths = [('%s_p%d%s' % (base, i + 1, ext), sh)
                 for i, sh in enumerate(sheets)]
    for path, sh in paths:
        with open(path, 'w') as f:
            f.write(sh.svg('code strips: %d codes, cell %.4f mm, pitch '
                           '%.2f mm, %d strips' % (
                               args.codes, args.cell_mm, args.sector_mm,
                               args.strips)))
        print('wrote %s (%d bytes)'
              % (path, os.path.getsize(path)))

    ok = True
    if not args.no_selftest:
        ok = raster_selftest(per_strip, sheets, args)
    if not ok:
        sys.exit(1)


if __name__ == '__main__':
    main()
