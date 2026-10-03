#!/usr/bin/env python3
"""Bake Unitree Go2 (mujoco_menagerie) meshes into a standing-pose Three.js buffer.

Source: https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_go2
License: BSD-3-Clause (see LICENSE alongside)

Output: go2_stand.json  — positions + colors, Y-up, +X forward, feet on y=0.
"""
from __future__ import annotations

import json
import math
import os
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parent
# 源 OBJ 放在仓库 third_party，避免被 http.server 直接扫到
ASSETS = Path("/home/linaro/robot_ws/third_party/unitree_go2/assets")
if not ASSETS.is_dir():
    ASSETS = ROOT / "assets"

# home keyframe joint angles (abduction, thigh, calf) × 4 legs FL FR RL RR
HOME = [0.0, 0.9, -1.8] * 4

# Skip ultra-heavy cosmetic shell; keep silhouette parts
SKIP_MESH = {"base_4"}

COLORS = {
    "black": (0.12, 0.13, 0.15),
    "white": (0.92, 0.93, 0.95),
    "gray": (0.55, 0.58, 0.64),
    "metal": (0.72, 0.76, 0.78),
}


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_from_axis_angle(axis, ang):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    s = math.sin(ang * 0.5)
    return (math.cos(ang * 0.5), x * s, y * s, z * s)


def quat_rotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    # v + w*t + cross(q.xyz, t)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def parse_obj(path: Path):
    verts = []
    faces = []
    with path.open("r", errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z, *rest = line.split()
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                idx = []
                for tok in line.split()[1:]:
                    idx.append(int(tok.split("/")[0]) - 1)
                for i in range(1, len(idx) - 1):
                    faces.append((idx[0], idx[i], idx[i + 1]))
    return verts, faces


def load_mesh(name: str):
    p = ASSETS / f"{name}.obj"
    if not p.exists():
        raise FileNotFoundError(p)
    return parse_obj(p)


# Visual geom list: (mesh_name, material, local_pos, local_quat_wxyz, body_chain)
# body_chain ends at the body this geom is attached to.
# Built from go2.xml standing hierarchy.

def make_scene():
    """Return list of (mesh, color_rgb, world_pos, world_quat) after home pose FK."""
    # Body tree with joints
    # Each body: name, parent, pos_in_parent, joint_axis or None, joint_idx or None, geoms
    # geoms: (mesh, material, pos, quat_wxyz)
    hip_q = (4.63268e-05, 1, 0, 0)  # ≈ 180° about X for FR
    hip_q_rl = (4.63268e-05, 0, 1, 0)
    hip_q_rr = (2.14617e-09, 4.63268e-05, 4.63268e-05, -1)

    bodies = {
        "base": {
            "parent": None,
            "pos": (0.0, 0.0, 0.27),  # home freejoint height
            "joint_axis": None,
            "joint_i": None,
            "geoms": [
                ("base_0", "black", (0, 0, 0), (1, 0, 0, 0)),
                ("base_1", "black", (0, 0, 0), (1, 0, 0, 0)),
                ("base_2", "black", (0, 0, 0), (1, 0, 0, 0)),
                ("base_3", "white", (0, 0, 0), (1, 0, 0, 0)),
                # base_4 skipped (huge)
            ],
        },
        "FL_hip": {
            "parent": "base",
            "pos": (0.1934, 0.0465, 0),
            "joint_axis": (1, 0, 0),
            "joint_i": 0,
            "geoms": [
                ("hip_0", "metal", (0, 0, 0), (1, 0, 0, 0)),
                ("hip_1", "gray", (0, 0, 0), (1, 0, 0, 0)),
            ],
        },
        "FL_thigh": {
            "parent": "FL_hip",
            "pos": (0, 0.0955, 0),
            "joint_axis": (0, 1, 0),
            "joint_i": 1,
            "geoms": [
                ("thigh_0", "metal", (0, 0, 0), (1, 0, 0, 0)),
                ("thigh_1", "gray", (0, 0, 0), (1, 0, 0, 0)),
            ],
        },
        "FL_calf": {
            "parent": "FL_thigh",
            "pos": (0, 0, -0.213),
            "joint_axis": (0, 1, 0),
            "joint_i": 2,
            "geoms": [
                ("calf_0", "gray", (0, 0, 0), (1, 0, 0, 0)),
                ("calf_1", "black", (0, 0, 0), (1, 0, 0, 0)),
                ("foot", "black", (0, 0, -0.213), (1, 0, 0, 0)),
            ],
        },
        "FR_hip": {
            "parent": "base",
            "pos": (0.1934, -0.0465, 0),
            "joint_axis": (1, 0, 0),
            "joint_i": 3,
            "geoms": [
                ("hip_0", "metal", (0, 0, 0), hip_q),
                ("hip_1", "gray", (0, 0, 0), hip_q),
            ],
        },
        "FR_thigh": {
            "parent": "FR_hip",
            "pos": (0, -0.0955, 0),
            "joint_axis": (0, 1, 0),
            "joint_i": 4,
            "geoms": [
                ("thigh_mirror_0", "metal", (0, 0, 0), (1, 0, 0, 0)),
                ("thigh_mirror_1", "gray", (0, 0, 0), (1, 0, 0, 0)),
            ],
        },
        "FR_calf": {
            "parent": "FR_thigh",
            "pos": (0, 0, -0.213),
            "joint_axis": (0, 1, 0),
            "joint_i": 5,
            "geoms": [
                ("calf_mirror_0", "gray", (0, 0, 0), (1, 0, 0, 0)),
                ("calf_mirror_1", "black", (0, 0, 0), (1, 0, 0, 0)),
                ("foot", "black", (0, 0, -0.213), (1, 0, 0, 0)),
            ],
        },
        "RL_hip": {
            "parent": "base",
            "pos": (-0.1934, 0.0465, 0),
            "joint_axis": (1, 0, 0),
            "joint_i": 6,
            "geoms": [
                ("hip_0", "metal", (0, 0, 0), hip_q_rl),
                ("hip_1", "gray", (0, 0, 0), hip_q_rl),
            ],
        },
        "RL_thigh": {
            "parent": "RL_hip",
            "pos": (0, 0.0955, 0),
            "joint_axis": (0, 1, 0),
            "joint_i": 7,
            "geoms": [
                ("thigh_0", "metal", (0, 0, 0), (1, 0, 0, 0)),
                ("thigh_1", "gray", (0, 0, 0), (1, 0, 0, 0)),
            ],
        },
        "RL_calf": {
            "parent": "RL_thigh",
            "pos": (0, 0, -0.213),
            "joint_axis": (0, 1, 0),
            "joint_i": 8,
            "geoms": [
                ("calf_0", "gray", (0, 0, 0), (1, 0, 0, 0)),
                ("calf_1", "black", (0, 0, 0), (1, 0, 0, 0)),
                ("foot", "black", (0, 0, -0.213), (1, 0, 0, 0)),
            ],
        },
        "RR_hip": {
            "parent": "base",
            "pos": (-0.1934, -0.0465, 0),
            "joint_axis": (1, 0, 0),
            "joint_i": 9,
            "geoms": [
                ("hip_0", "metal", (0, 0, 0), hip_q_rr),
                ("hip_1", "gray", (0, 0, 0), hip_q_rr),
            ],
        },
        "RR_thigh": {
            "parent": "RR_hip",
            "pos": (0, -0.0955, 0),
            "joint_axis": (0, 1, 0),
            "joint_i": 10,
            "geoms": [
                ("thigh_mirror_0", "metal", (0, 0, 0), (1, 0, 0, 0)),
                ("thigh_mirror_1", "gray", (0, 0, 0), (1, 0, 0, 0)),
            ],
        },
        "RR_calf": {
            "parent": "RR_thigh",
            "pos": (0, 0, -0.213),
            "joint_axis": (0, 1, 0),
            "joint_i": 11,
            "geoms": [
                ("calf_mirror_0", "gray", (0, 0, 0), (1, 0, 0, 0)),
                ("calf_mirror_1", "black", (0, 0, 0), (1, 0, 0, 0)),
                ("foot", "black", (0, 0, -0.213), (1, 0, 0, 0)),
            ],
        },
    }

    # FK: world pose for each body
    order = [
        "base",
        "FL_hip", "FL_thigh", "FL_calf",
        "FR_hip", "FR_thigh", "FR_calf",
        "RL_hip", "RL_thigh", "RL_calf",
        "RR_hip", "RR_thigh", "RR_calf",
    ]
    world = {}
    for name in order:
        b = bodies[name]
        if b["parent"] is None:
            wp, wq = b["pos"], (1.0, 0.0, 0.0, 0.0)
        else:
            pp, pq = world[b["parent"]]
            lp = b["pos"]
            # joint rotation in parent
            jq = (1.0, 0.0, 0.0, 0.0)
            if b["joint_axis"] is not None and b["joint_i"] is not None:
                jq = quat_from_axis_angle(b["joint_axis"], HOME[b["joint_i"]])
            # child origin = parent_rot * local_pos + parent_pos
            rp = quat_rotate(pq, lp)
            wp = (pp[0] + rp[0], pp[1] + rp[1], pp[2] + rp[2])
            wq = quat_mul(pq, jq)
        world[name] = (wp, wq)

    placed = []
    for name in order:
        b = bodies[name]
        wp, wq = world[name]
        for mesh, mat, gpos, gquat in b["geoms"]:
            if mesh in SKIP_MESH:
                continue
            # geom local → body → world
            gq = quat_mul(wq, gquat)
            gp = quat_rotate(wq, gpos)
            gp = (wp[0] + gp[0], wp[1] + gp[1], wp[2] + gp[2])
            placed.append((mesh, COLORS[mat], gp, gq))
    return placed


def mj_to_three(p):
    """MuJoCo Z-up X-forward → Three.js Y-up X-forward (Z = -Y)."""
    x, y, z = p
    return (x, z, -y)


def main():
    cache = {}
    pos_out = []
    col_out = []
    stride = 10  # web-friendly density (~1.5MB bin)

    for mesh, rgb, gp, gq in make_scene():
        if mesh not in cache:
            print("load", mesh)
            cache[mesh] = load_mesh(mesh)
        verts, faces = cache[mesh]
        for fi, (i0, i1, i2) in enumerate(faces):
            if fi % stride:
                continue
            for ii in (i0, i1, i2):
                lx, ly, lz = verts[ii]
                wx = quat_rotate(gq, (lx, ly, lz))
                w = (gp[0] + wx[0], gp[1] + wx[1], gp[2] + wx[2])
                tx, ty, tz = mj_to_three(w)
                pos_out.extend((tx, ty, tz))
                col_out.extend(rgb)

    ys = pos_out[1::3]
    y0 = min(ys) if ys else 0.0
    for i in range(1, len(pos_out), 3):
        pos_out[i] -= y0

    n_tri = len(pos_out) // 9
    bin_path = ROOT / "go2_stand.bin"
    with bin_path.open("wb") as f:
        f.write(b"GO2S")
        f.write(struct.pack("<I", n_tri))
        f.write(struct.pack("<%df" % len(pos_out), *pos_out))
        f.write(bytes(max(0, min(255, int(c * 255 + 0.5))) for c in col_out))
    meta = {
        "source": "google-deepmind/mujoco_menagerie unitree_go2 (BSD-3-Clause)",
        "pose": "home standing",
        "triangles": n_tri,
        "file": "go2_stand.bin",
        "format": "GO2S",
    }
    (ROOT / "go2_stand.meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(f"wrote {bin_path}  tris={n_tri}  size={bin_path.stat().st_size/1024:.0f}KB")


if __name__ == "__main__":
    main()
