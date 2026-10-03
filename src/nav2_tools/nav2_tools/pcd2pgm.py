#!/usr/bin/env python3
"""Convert FAST-LIO PCD → Nav2 PGM/YAML with RANSAC ground leveling.

Aligns the map so the fitted ground plane is z=0 (same idea as wheeltec pcd2pgm),
then projects the obstacle height band [z_min, z_max] relative to that ground.

Usage:
  python3 pcd2pgm.py scans.pcd /path/to/maps/name \\
      [--pitch-down 0.785] [--roll 0] [--lidar-z 0] \\
      [--resolution 0.05] [--z-min 0.15] [--z-max 1.4] \\
      [--occ-min 0.12] [--occ-pts 4] [--despeckle 2]
"""
from __future__ import annotations

import argparse
import math
import os
import random
import struct
import sys


def read_pcd(path, voxel=0.0):
    """Load PCD xyz. If voxel>0, keep ≤1 point per voxel while streaming (saves RAM on huge maps)."""
    with open(path, 'rb') as f:
        # Parse header without slurping a multi-GB body into one bytes blob
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError('truncated PCD header')
            header_lines.append(line)
            if line.startswith(b'DATA') or line.upper().startswith(b'DATA'):
                break
        header = b''.join(header_lines).decode('ascii', 'replace')
        fields, points, width, height, datatype = [], None, 0, 1, 'ascii'
        for ln in header.split('\n'):
            ln = ln.strip()
            if ln.startswith('FIELDS'):
                fields = ln.split()[1:]
            elif ln.startswith('WIDTH'):
                width = int(ln.split()[1])
            elif ln.startswith('HEIGHT'):
                height = int(ln.split()[1])
            elif ln.startswith('POINTS'):
                points = int(ln.split()[1])
            elif ln.startswith('DATA'):
                datatype = ln.split()[1].lower()
        if points is None:
            points = width * height

        inv = (1.0 / voxel) if voxel and voxel > 1e-6 else 0.0
        # voxel → one kept point; plain list if no voxel
        kept = {} if inv else None
        pts = [] if not inv else None

        def push(x, y, z):
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                return
            if inv:
                key = (int(math.floor(x * inv)), int(math.floor(y * inv)),
                       int(math.floor(z * inv)))
                if key not in kept:
                    kept[key] = (x, y, z)
            else:
                pts.append((x, y, z))

        if datatype == 'ascii':
            for line in f:
                toks = line.decode('ascii', 'replace').split()
                if len(toks) >= 3:
                    try:
                        push(float(toks[0]), float(toks[1]), float(toks[2]))
                    except ValueError:
                        pass
        elif datatype == 'binary':
            # Prefer SIZE/TYPE from header if present; default xyz float32
            stride = 12
            if len(fields) >= 3:
                # common FAST-LIO: x y z intensity → 16 bytes
                stride = max(12, len(fields) * 4)
            n = 0
            while n < points:
                buf = f.read(stride)
                if len(buf) < 12:
                    break
                x, y, z = struct.unpack_from('<fff', buf, 0)
                push(x, y, z)
                n += 1
        else:
            raise ValueError(f'unsupported DATA type: {datatype}')

    out = list(kept.values()) if kept is not None else pts
    if inv:
        print(f'PCD streamed voxel={voxel}m → {len(out)} pts (from POINTS={points})')
    return out


def nominal_up(pitch_down: float, roll: float):
    """Ground normal in camera_init for a forward-tilted lidar (pitch_down>0).

    Matches wheeltec / v2: up = (-sinθ, sinφ·cosθ, cosφ·cosθ).
    """
    sp, cp = math.sin(pitch_down), math.cos(pitch_down)
    sr, cr = math.sin(roll), math.cos(roll)
    return (-sp, sr * cp, cr * cp)


def _normalize(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def ransac_plane(pts, up, dist_thr=0.05, eps_deg=20.0, iters=800):
    if len(pts) < 100:
        return None
    cos_eps = math.cos(math.radians(eps_deg))
    best = None
    best_cnt = 0
    n = len(pts)
    rng = random.Random(0)
    for _ in range(iters):
        i0, i1, i2 = rng.randrange(n), rng.randrange(n), rng.randrange(n)
        if i0 == i1 or i0 == i2 or i1 == i2:
            continue
        ax, ay, az = pts[i0]
        b = (pts[i1][0] - ax, pts[i1][1] - ay, pts[i1][2] - az)
        c = (pts[i2][0] - ax, pts[i2][1] - ay, pts[i2][2] - az)
        nx = b[1] * c[2] - b[2] * c[1]
        ny = b[2] * c[0] - b[0] * c[2]
        nz = b[0] * c[1] - b[1] * c[0]
        nl = math.sqrt(nx * nx + ny * ny + nz * nz)
        if nl < 1e-9:
            continue
        nx, ny, nz = nx / nl, ny / nl, nz / nl
        dot = nx * up[0] + ny * up[1] + nz * up[2]
        if dot < 0:
            nx, ny, nz, dot = -nx, -ny, -nz, -dot
        if dot < cos_eps:
            continue
        d = -(nx * ax + ny * ay + nz * az)
        cnt = 0
        for p in pts:
            if abs(nx * p[0] + ny * p[1] + nz * p[2] + d) <= dist_thr:
                cnt += 1
        if cnt > best_cnt:
            best_cnt = cnt
            best = (nx, ny, nz, d, cnt)
    if not best or best_cnt < max(50, n // 12):
        return None
    # refine d from inlier centroid
    sx = sy = sz = 0.0
    c = 0
    nx, ny, nz, d, _ = best
    for p in pts:
        if abs(nx * p[0] + ny * p[1] + nz * p[2] + d) <= dist_thr:
            sx += p[0]
            sy += p[1]
            sz += p[2]
            c += 1
    if c > 10:
        d = -(nx * sx / c + ny * sy / c + nz * sz / c)
    return nx, ny, nz, d, c


def level_mat_from_normal(n):
    """Rodrigues: rotate n -> +Z. Returns 3x3 row-major."""
    u = _normalize(n)
    c = u[2]
    if c > 0.999999:
        return ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    if c < -0.999999:
        return ((1, 0, 0), (0, -1, 0), (0, 0, -1))
    vx, vy = u[1], -u[0]
    k = 1.0 / (1.0 + c)
    return (
        (1 - vy * vy * k, vx * vy * k, vy),
        (vx * vy * k, 1 - vx * vx * k, -vx),
        (-vy, vx, 1 - (vx * vx + vy * vy) * k),
    )


def apply_level(pts, M, zoff):
    out = []
    for x, y, z in pts:
        qx = M[0][0] * x + M[0][1] * y + M[0][2] * z
        qy = M[1][0] * x + M[1][1] * y + M[1][2] * z
        qz = M[2][0] * x + M[2][1] * y + M[2][2] * z + zoff
        out.append((qx, qy, qz))
    return out


def build_grid(pts, resolution, occ_min, z_min, z_max, occ_pts=4,
               free_pts=1, ground_min_z=-0.15, free_dilate=8, free_erode=2,
               fill_enclosed=True):
    """障碍带 [z_min,z_max] → occupied；近地带 [ground_min_z, z_min) → free。

    光滑地面回波稀疏时自由区会成麻点：先大半径膨胀再小半径腐蚀把地板连片，
    再填充被自由/障碍围住的空洞。障碍永远优先。
    """
    band = [p for p in pts if ground_min_z <= p[2] <= z_max]
    if not band:
        return None
    xs = [p[0] for p in band]
    ys = [p[1] for p in band]
    pad = 1.0
    x_min, x_max = min(xs) - pad, max(xs) + pad
    y_min, y_max = min(ys) - pad, max(ys) + pad
    w = max(1, int(math.ceil((x_max - x_min) / resolution)))
    h = max(1, int(math.ceil((y_max - y_min) / resolution)))
    if w * h > 80_000_000:
        return None

    occ_cnt = [[0] * w for _ in range(h)]
    free_cnt = [[0] * w for _ in range(h)]
    zmin_c = [[0.0] * w for _ in range(h)]
    zmax_c = [[0.0] * w for _ in range(h)]
    for x, y, z in band:
        ix = min(w - 1, max(0, int((x - x_min) / resolution)))
        iy = min(h - 1, max(0, int((y - y_min) / resolution)))
        if z >= z_min:
            c = occ_cnt[iy][ix]
            if c == 0:
                zmin_c[iy][ix] = zmax_c[iy][ix] = z
            else:
                if z < zmin_c[iy][ix]:
                    zmin_c[iy][ix] = z
                if z > zmax_c[iy][ix]:
                    zmax_c[iy][ix] = z
            if c < 65535:
                occ_cnt[iy][ix] = c + 1
        else:
            if free_cnt[iy][ix] < 65535:
                free_cnt[iy][ix] += 1

    clear_h = z_min + max(0.04, occ_min * 0.5)
    occm = [[0] * w for _ in range(h)]
    freem = [[0] * w for _ in range(h)]
    for iy in range(h):
        for ix in range(w):
            c = occ_cnt[iy][ix]
            if c >= occ_pts:
                spread = zmax_c[iy][ix] - zmin_c[iy][ix]
                if spread >= occ_min or zmax_c[iy][ix] >= clear_h:
                    occm[iy][ix] = 1
            # 障碍带里点数不够高的格子也当「看见过的地面/矮物」→ 自由种子
            elif c > 0:
                freem[iy][ix] = 1
            if free_cnt[iy][ix] >= free_pts:
                freem[iy][ix] = 1

    free_raw = sum(1 for row in freem for v in row if v)

    def morph(src, r, dilate):
        if r <= 0:
            return [row[:] for row in src]
        dst = [[0] * w for _ in range(h)]
        for iy in range(h):
            for ix in range(w):
                v = 0 if dilate else 1
                done = False
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        ny, nx = iy + dy, ix + dx
                        s = 0 if (nx < 0 or nx >= w or ny < 0 or ny >= h) else src[ny][nx]
                        if dilate:
                            if s:
                                v = 1
                                done = True
                                break
                        else:
                            if not s:
                                v = 0
                                done = True
                                break
                    if done:
                        break
                dst[iy][ix] = v
        return dst

    # 大膨胀把地板麻点连成片，再小腐蚀修边；障碍格上的自由会被清掉
    if free_dilate > 0:
        freem = morph(freem, free_dilate, True)
    for iy in range(h):
        for ix in range(w):
            if occm[iy][ix]:
                freem[iy][ix] = 0
    if free_erode > 0:
        freem = morph(freem, free_erode, False)
        for iy in range(h):
            for ix in range(w):
                if occm[iy][ix]:
                    freem[iy][ix] = 0
    free_closed = sum(1 for row in freem for v in row if v)

    filled = 0
    if fill_enclosed:
        reach = [[0] * w for _ in range(h)]
        stack = []

        def push(x, y):
            if x < 0 or x >= w or y < 0 or y >= h:
                return
            if reach[y][x] or freem[y][x] or occm[y][x]:
                return
            reach[y][x] = 1
            stack.append((x, y))

        for x in range(w):
            push(x, 0)
            push(x, h - 1)
        for y in range(h):
            push(0, y)
            push(w - 1, y)
        while stack:
            x, y = stack.pop()
            push(x + 1, y)
            push(x - 1, y)
            push(x, y + 1)
            push(x, y - 1)
        for iy in range(h):
            for ix in range(w):
                if not freem[iy][ix] and not occm[iy][ix] and not reach[iy][ix]:
                    freem[iy][ix] = 1
                    filled += 1

    grid = [[-1] * w for _ in range(h)]
    for iy in range(h):
        for ix in range(w):
            if occm[iy][ix]:
                grid[iy][ix] = 100
            elif freem[iy][ix]:
                grid[iy][ix] = 0
    print(f'free: raw={free_raw} grown={free_closed} filled_holes={filled} '
          f'(dilate={free_dilate} erode={free_erode})')
    return grid, x_min, y_min, w, h


def despeckle(grid, min_neighbors=2, passes=2):
    """Drop isolated occupied cells (8-neighborhood) that look like noise."""
    h, w = len(grid), len(grid[0])
    for _ in range(max(1, passes)):
        kill = []
        for iy in range(h):
            for ix in range(w):
                if grid[iy][ix] != 100:
                    continue
                n = 0
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        y, x = iy + dy, ix + dx
                        if 0 <= y < h and 0 <= x < w and grid[y][x] == 100:
                            n += 1
                if n < min_neighbors:
                    kill.append((iy, ix))
        for iy, ix in kill:
            # 孤立噪点变未知，不要误标成可通行缝
            grid[iy][ix] = -1
    return grid


def write_pgm(grid, path):
    """Write PGM for nav2 map_server.

    OccupancyGrid: row0 = origin = low world-Y.
    PGM/image: row0 = top of image. map_server flips on load so that
    image-top → high world-Y. Therefore we must write high-Y rows first,
    otherwise saved maps are Y-mirrored vs the PCD / live /scan.
    """
    h, w = len(grid), len(grid[0])
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        for iy in range(h - 1, -1, -1):  # high world-Y first (= image top)
            row = bytearray()
            for ix in range(w):
                v = grid[iy][ix]
                row.append(205 if v < 0 else (254 if v == 0 else 0))
            f.write(bytes(row))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input')
    ap.add_argument('output')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--occ-min', type=float, default=0.12,
                    help='min vertical spread (m) inside a cell to count as obstacle')
    ap.add_argument('--occ-pts', type=int, default=4,
                    help='min point hits per cell before it can become occupied')
    ap.add_argument('--free-pts', type=int, default=1,
                    help='min ground-band hits to seed a free cell')
    ap.add_argument('--ground-min-z', type=float, default=-0.15,
                    help='near-ground band lower (leveled); [ground_min_z, z_min) → free')
    ap.add_argument('--free-dilate', type=int, default=8,
                    help='dilate free seeds (cells) to grow traversable floor')
    ap.add_argument('--free-erode', type=int, default=2,
                    help='erode after dilate to clean edges')
    ap.add_argument('--free-close', type=int, default=-1,
                    help='legacy: if >=0, use as both dilate and erode radius')
    ap.add_argument('--no-fill-enclosed', action='store_true',
                    help='disable filling holes enclosed by free/occ')
    ap.add_argument('--z-min', type=float, default=0.05,
                    help='obstacle band lower (leveled z); sync config/obstacle_height_band.yaml / stage2')
    ap.add_argument('--z-max', type=float, default=0.85,
                    help='obstacle band upper; sync with stage2 max_height / Go2W')
    ap.add_argument('--despeckle', type=int, default=2,
                    help='isolated occupied-cell removal passes (0=off)')
    ap.add_argument('--pitch-down', type=float, default=0.342586,
                    help='forward tilt rad (positive = looking down), Mid360 on dog head ~19.6°')
    ap.add_argument('--roll', type=float, default=0.0)
    ap.add_argument('--lidar-z', type=float, default=0.0,
                    help='if >0, use as ground height instead of RANSAC d')
    ap.add_argument('--voxel', type=float, default=0.05,
                    help='stream-downsample PCD by this voxel (m); 0=keep all (may OOM)')
    ap.add_argument('--no-align', action='store_true')
    ap.add_argument('--ground-z', type=float, default=0.03)  # legacy alias ignored
    args = ap.parse_args()

    # Auto voxel on huge files if user left default
    voxel = float(args.voxel)
    try:
        sz = os.path.getsize(args.input)
    except OSError:
        sz = 0
    if voxel > 0 and sz > 400 * 1024 * 1024:
        print(f'large PCD ({sz/1e9:.2f} GB) — voxel downsample {voxel}m to avoid OOM')
    elif voxel <= 0 and sz > 400 * 1024 * 1024:
        voxel = 0.05
        print(f'large PCD ({sz/1e9:.2f} GB) — forcing voxel=0.05m (was 0)')

    pts = read_pcd(args.input, voxel=voxel)
    print(f'PCD loaded: {len(pts)} points')
    if not pts:
        print('no points')
        sys.exit(1)

    up = nominal_up(args.pitch_down, args.roll)
    print(f'expected up (pitch_down={args.pitch_down:.3f}rad='
          f'{math.degrees(args.pitch_down):.1f}°): '
          f'({up[0]:.3f},{up[1]:.3f},{up[2]:.3f})')

    if not args.no_align:
        low = []
        for p in pts:
            h = up[0] * p[0] + up[1] * p[1] + up[2] * p[2]
            if -3.0 < h < 0.35:
                low.append(p)
        print(f'low candidates: {len(low)} / {len(pts)}')
        fit = ransac_plane(low, up) if len(low) > 100 else None
        if fit:
            nx, ny, nz, d, cnt = fit
            fitted_h = abs(d)
            resid = math.degrees(math.acos(max(-1.0, min(1.0,
                nx * up[0] + ny * up[1] + nz * up[2]))))
            zoff = args.lidar_z if args.lidar_z > 1e-3 else fitted_h
            print(f'RANSAC ground: inliers={cnt} ({100*cnt/max(1,len(low)):.0f}% of low), '
                  f'resid vs nominal {resid:.1f}°, fitted_h={fitted_h:.3f}m, use_z={zoff:.3f}m')
            M = level_mat_from_normal((nx, ny, nz))
            pts = apply_level(pts, M, zoff)
        else:
            print('RANSAC failed — fall back to nominal pitch leveling')
            M = level_mat_from_normal(up)
            zoff = args.lidar_z if args.lidar_z > 1e-3 else 0.45
            pts = apply_level(pts, M, zoff)

    dilate = int(args.free_dilate)
    erode = int(args.free_erode)
    if int(args.free_close) >= 0:
        dilate = erode = int(args.free_close)
    res = build_grid(
        pts, args.resolution, args.occ_min, args.z_min, args.z_max,
        occ_pts=max(1, int(args.occ_pts)),
        free_pts=max(1, int(args.free_pts)),
        ground_min_z=float(args.ground_min_z),
        free_dilate=max(0, dilate),
        free_erode=max(0, erode),
        fill_enclosed=not args.no_fill_enclosed)
    if res is None:
        print('no points in height band')
        sys.exit(1)
    grid, x_min, y_min, w, h = res
    if args.despeckle > 0:
        before = sum(1 for row in grid for v in row if v == 100)
        despeckle(grid, min_neighbors=2, passes=int(args.despeckle))
        after = sum(1 for row in grid for v in row if v == 100)
        print(f'despeckle×{args.despeckle}: occupied {before} → {after}')
    pgm = args.output + '.pgm'
    write_pgm(grid, pgm)
    yaml = args.output + '.yaml'
    with open(yaml, 'w') as f:
        f.write('image: %s\n' % os.path.basename(pgm))
        f.write('resolution: %f\n' % args.resolution)
        f.write('origin: [%f, %f, 0.0]\n' % (x_min, y_min))
        f.write('negate: 0\n')
        # free_thresh 必须严格小于 (255-205)/255≈0.196，与 wheeltec pcd2pgm 一致用 0.196
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')
    occ = sum(1 for row in grid for v in row if v == 100)
    free = sum(1 for row in grid for v in row if v == 0)
    unk = sum(1 for row in grid for v in row if v == -1)
    tot = max(1, w * h)
    print(f'grid {w}x{h} @ {args.resolution}m: '
          f'occ={occ} ({100*occ/tot:.1f}%) free={free} ({100*free/tot:.1f}%) '
          f'unk={unk} ({100*unk/tot:.1f}%)')
    print(f'wrote {pgm} + {yaml}')


if __name__ == '__main__':
    main()
