#!/usr/bin/env python3
"""Generate glass_basic.pgm — static occupancy grid for Nav2.

Map covers 5.20 m x 3.20 m (slightly larger than the 5x3 glass plane to
include the frame walls as occupied cells). Origin is the bottom-left
corner at world (-2.6, -1.6, 0).

Frame walls (occupied):
  bottom: world y in [-1.55, -1.50]
  top:    world y in [ 1.50,  1.55]
  left:   world x in [-2.55, -2.50]
  right:  world x in [ 2.50,  2.55]

Glass interior (free): world x in [-2.50, 2.50], y in [-1.50, 1.50].
"""

RES = 0.05  # m / pixel
W_M = 5.20
H_M = 3.20
ORIGIN_X = -2.6
ORIGIN_Y = -1.6

W = int(round(W_M / RES))   # 104
H = int(round(H_M / RES))   # 64

FREE = 254     # white-ish; map_server treats >occupied_thresh as occupied
OCCUPIED = 0   # black


def world_to_pixel(x_w, y_w):
    """Convert world (x, y) in metres to pixel (col, row) in the PGM.

    PGM rows are top-down; world y is bottom-up. We flip so that pixel
    (0, 0) is the top-left corner with maximum y.
    """
    col = int(round((x_w - ORIGIN_X) / RES))
    # Bottom-up world -> top-down image: row = (H - 1) - row_from_bottom
    row_from_bottom = int(round((y_w - ORIGIN_Y) / RES))
    row = (H - 1) - row_from_bottom
    return col, row


def fill_rect(grid, x_min, x_max, y_min, y_max, value):
    c0, r1 = world_to_pixel(x_min, y_min)  # bottom-left -> col_min, row_max (top-down)
    c1, r0 = world_to_pixel(x_max, y_max)  # top-right   -> col_max, row_min
    c_lo, c_hi = sorted([c0, c1])
    r_lo, r_hi = sorted([r0, r1])
    c_lo = max(0, c_lo)
    c_hi = min(W - 1, c_hi)
    r_lo = max(0, r_lo)
    r_hi = min(H - 1, r_hi)
    for r in range(r_lo, r_hi + 1):
        for c in range(c_lo, c_hi + 1):
            grid[r][c] = value


# Initialise as fully free
grid = [[FREE for _ in range(W)] for _ in range(H)]

# Frame walls: paint as occupied
fill_rect(grid, -2.55, 2.55, -1.55, -1.50, OCCUPIED)  # bottom
fill_rect(grid, -2.55, 2.55,  1.50,  1.55, OCCUPIED)  # top
fill_rect(grid, -2.55, -2.50, -1.55, 1.55, OCCUPIED)  # left
fill_rect(grid,  2.50,  2.55, -1.55, 1.55, OCCUPIED)  # right

# Write ASCII PGM (P2) next to this script
import os
out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'glass_basic.pgm')
with open(out_path, 'w') as f:
    f.write(f'P2\n')
    f.write(f'# glass_basic.pgm — regenerate with: python3 gen_map.py\n')
    f.write(f'# resolution = {RES} m/px, origin = ({ORIGIN_X}, {ORIGIN_Y}, 0)\n')
    f.write(f'{W} {H}\n')
    f.write('255\n')
    for row in grid:
        f.write(' '.join(str(v) for v in row) + '\n')

print(f'Wrote {W}x{H} PGM with frame walls.')
