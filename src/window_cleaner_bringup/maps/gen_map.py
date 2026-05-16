#!/usr/bin/env python3
"""Generate static Nav2 occupancy grids (.pgm + .yaml) for each world.

Phase 4: refactored from the original single-world script into a
parameterised generator so per-world maps can be produced. The frame-wall
rectangles are now *derived* from the glass dimensions instead of being
literals, and the `.yaml` sidecar (previously hand-maintained for the basic
world) is emitted alongside the `.pgm` to prevent drift.

Regression invariant: the occupancy-grid content for `glass_basic`
(everything from the dimensions line onward) is byte-identical to the
previous output — only the line-2 provenance comment is normalised, and the
grid the planner/Nav2 actually consumes is unchanged.

Map convention (unchanged): a strip of free margin + the frame walls as
occupied cells around a free glass interior. The map covers
``glass + 2*thickness + 2*margin`` so the outer wall face plus a small
border is inside the grid; the origin is the bottom-left world corner.

    python3 gen_map.py            # regenerate every world map
    python3 gen_map.py --list     # show the world table without writing
"""

from __future__ import annotations

import argparse
import os

RES = 0.05            # m / pixel
WALL_THICKNESS = 0.05  # frame wall thickness (m)
MAP_MARGIN = 0.05     # free border beyond the outer wall face (m)

FREE = 254     # white-ish; map_server treats >occupied_thresh as occupied
OCCUPIED = 0   # black

# world name -> glass surface dimensions (full width x height, metres).
# `glass_large` is geometrically identical to `glass_basic` and reuses the
# basic map at runtime (nav2.launch.py map:=glass_basic.yaml), so it is not
# generated separately. `glass_obstacles` is geometrically the basic 5x3
# surface; its interior mullions are NOT painted here (Phase-3 documented
# limitation — the planner-side detour is future work, out of Phase-4
# conservative scope), so it also reuses the basic map.
WORLDS = {
    "glass_basic": (5.0, 3.0),
    "glass_small": (2.0, 1.0),
}


def wall_rects(glass_w, glass_h, thickness=WALL_THICKNESS):
    """Four frame-wall AABBs (x_min, x_max, y_min, y_max) in world metres,
    derived from the glass half-extents. Order matches the original script
    (bottom, top, left, right); all are OCCUPIED so order is immaterial to
    the resulting grid."""
    hw = glass_w / 2.0
    hh = glass_h / 2.0
    t = thickness
    return [
        (-hw - t,  hw + t, -hh - t, -hh),     # bottom
        (-hw - t,  hw + t,  hh,      hh + t),  # top
        (-hw - t, -hw,     -hh - t,  hh + t),  # left
        (hw,       hw + t, -hh - t,  hh + t),  # right
    ]


def _world_to_pixel(x_w, y_w, origin_x, origin_y, h_px):
    """World (x, y) metres -> pixel (col, row). PGM rows are top-down while
    world y is bottom-up, so the row is flipped."""
    col = int(round((x_w - origin_x) / RES))
    row_from_bottom = int(round((y_w - origin_y) / RES))
    row = (h_px - 1) - row_from_bottom
    return col, row


def _fill_rect(grid, w_px, h_px, origin_x, origin_y,
               x_min, x_max, y_min, y_max, value):
    c0, r1 = _world_to_pixel(x_min, y_min, origin_x, origin_y, h_px)
    c1, r0 = _world_to_pixel(x_max, y_max, origin_x, origin_y, h_px)
    c_lo, c_hi = sorted([c0, c1])
    r_lo, r_hi = sorted([r0, r1])
    c_lo = max(0, c_lo)
    c_hi = min(w_px - 1, c_hi)
    r_lo = max(0, r_lo)
    r_hi = min(h_px - 1, r_hi)
    for r in range(r_lo, r_hi + 1):
        for c in range(c_lo, c_hi + 1):
            grid[r][c] = value


def generate_map(name, glass_w, glass_h, out_dir):
    """Write ``<name>.pgm`` and ``<name>.yaml`` into ``out_dir``."""
    w_m = glass_w + 2 * WALL_THICKNESS + 2 * MAP_MARGIN
    h_m = glass_h + 2 * WALL_THICKNESS + 2 * MAP_MARGIN
    # Round away float noise (e.g. -5.2/2 -> -2.5999999999999996). The origin
    # is always a multiple of RES, so 3 dp is exact and keeps the .yaml /
    # comment identical to the hand-written glass_basic.yaml.
    origin_x = round(-w_m / 2.0, 3)
    origin_y = round(-h_m / 2.0, 3)
    w_px = int(round(w_m / RES))
    h_px = int(round(h_m / RES))

    grid = [[FREE for _ in range(w_px)] for _ in range(h_px)]
    for (xa, xb, ya, yb) in wall_rects(glass_w, glass_h):
        _fill_rect(grid, w_px, h_px, origin_x, origin_y,
                   xa, xb, ya, yb, OCCUPIED)

    pgm_path = os.path.join(out_dir, f"{name}.pgm")
    with open(pgm_path, "w") as f:
        f.write("P2\n")
        f.write(f"# {name}.pgm — regenerate with: python3 gen_map.py\n")
        f.write(f"# resolution = {RES} m/px, "
                f"origin = ({origin_x}, {origin_y}, 0)\n")
        f.write(f"{w_px} {h_px}\n")
        f.write("255\n")
        for row in grid:
            f.write(" ".join(str(v) for v in row) + "\n")

    yaml_path = os.path.join(out_dir, f"{name}.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"image: {name}.pgm\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {RES}\n")
        f.write(f"origin: [{origin_x}, {origin_y}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")

    print(f"Wrote {w_px}x{h_px} {name}.pgm (+ .yaml) "
          f"origin=({origin_x}, {origin_y})")


def gen_all_maps(out_dir):
    for name, (gw, gh) in WORLDS.items():
        generate_map(name, gw, gh, out_dir)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Generate Nav2 occupancy maps.")
    ap.add_argument("--list", action="store_true",
                    help="print the world table and exit")
    args = ap.parse_args(argv)
    if args.list:
        for name, (gw, gh) in WORLDS.items():
            print(f"{name}: glass {gw} x {gh} m")
        return
    out_dir = os.path.dirname(os.path.abspath(__file__))
    gen_all_maps(out_dir)


if __name__ == "__main__":
    main()
