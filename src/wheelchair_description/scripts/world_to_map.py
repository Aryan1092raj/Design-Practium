#!/usr/bin/env python3
"""Rasterise a Gazebo world's collision geometry into a Nav2 occupancy map.

A cell is occupied if a collision shape reaches into 0.05-1.0 m, or a visual shape sits at
the lidar plane. The map frame equals the world frame, so a robot spawned at the world
origin has map pose (0, 0).

Usage: world_to_map.py WORLD.world MODELS_DIR OUT_PREFIX
Writes OUT_PREFIX.pgm and OUT_PREFIX.yaml.
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import label

RES = 0.02
LIDAR_Z = 0.29  # world height of the sim lidar plane: base_link z -0.03 + 0.2 - 0.13 (urdf) + 0.25 (wc_gazebo.xacro)


def tag(e):
    return e.tag.split("}")[-1]


def pose_of(e):
    """4x4 matrix from a child <pose>x y z roll pitch yaw</pose>; identity if absent."""
    p = e.find("pose")
    v = ([float(s) for s in p.text.split()] + [0.0] * 6)[:6] if p is not None and p.text else [0.0] * 6
    x, y, z, r, pt, yw = v
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(pt), math.sin(pt),
                              math.cos(yw), math.sin(yw))
    m = np.eye(4)
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = [x, y, z]
    return m


def dae_triangles(path):
    """Triangles (n,3,3) of a COLLADA file in metres, node transforms applied."""
    root = ET.parse(path).getroot()
    unit = 1.0
    for e in root.iter():
        if tag(e) == "unit":
            unit = float(e.get("meter", 1))
        if tag(e) == "up_axis" and e.text.strip() != "Z_UP":
            print(f"warning: {path} is {e.text.strip()}, not handled")
    geoms = {}
    for g in root.iter():
        if tag(g) != "geometry":
            continue
        mesh = next(c for c in g if tag(c) == "mesh")
        src = {s.get("id"): np.array(next(c for c in s if tag(c) == "float_array").text.split(),
                                     float).reshape(-1, int(next(a for a in s.iter()
                                                                 if tag(a) == "accessor").get("stride", 1)))
               for s in mesh if tag(s) == "source"}
        verts = {v.get("id"): next(i for i in v if tag(i) == "input"
                                   and i.get("semantic") == "POSITION").get("source")[1:]
                 for v in mesh if tag(v) == "vertices"}
        tris = []
        for prim in mesh:
            if tag(prim) not in ("triangles", "polylist"):
                continue
            inputs = [i for i in prim if tag(i) == "input"]
            stride = max(int(i.get("offset")) for i in inputs) + 1
            vi = next(i for i in inputs if i.get("semantic") == "VERTEX")
            pos = src[verts[vi.get("source")[1:]]]
            ptxt = next(c for c in prim if tag(c) == "p").text
            if not ptxt:
                continue
            p = np.array(ptxt.split(), int)
            p = p.reshape(-1, stride)[:, int(vi.get("offset"))]
            if tag(prim) == "triangles":
                counts = [3] * (len(p) // 3)
            else:
                counts = [int(s) for s in next(c for c in prim if tag(c) == "vcount").text.split()]
            k = 0
            for c in counts:
                for j in range(1, c - 1):
                    tris.append(pos[[p[k], p[k + j], p[k + j + 1]]])
                k += c
        geoms[g.get("id")] = np.array(tris)

    out = []

    def walk(node, m):
        for c in node:
            t = tag(c)
            if t == "matrix":
                m = m @ np.array(c.text.split(), float).reshape(4, 4)
            elif t == "translate":
                tm = np.eye(4)
                tm[:3, 3] = [float(s) for s in c.text.split()]
                m = m @ tm
            elif t == "scale":
                m = m @ np.diag([float(s) for s in c.text.split()] + [1.0])
        for c in node:
            if tag(c) == "instance_geometry":
                out.append((geoms[c.get("url")[1:]], m))
            elif tag(c) == "node":
                walk(c, m)

    for scene in root.iter():
        if tag(scene) == "visual_scene":
            for n in scene:
                if tag(n) == "node":
                    walk(n, np.eye(4))
    u = np.diag([unit, unit, unit, 1.0])
    return [(t, u @ m) for t, m in out]


def box_triangles(size):
    sx, sy, sz = (s / 2 for s in size)
    c = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    faces = [(0, 1, 3), (0, 3, 2), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 3, 7), (1, 7, 5)]
    return np.array([c[list(f)] for f in faces])


def collisions(model, t_model, models_dir, kind="collision"):
    """Yield world-frame triangles for every <kind> shape (collision or visual) of an SDF <model>."""
    for link in model.iter("link"):
        t_link = t_model @ pose_of(link)
        for col in link.iter(kind):
            t = t_link @ pose_of(col)
            geo = col.find("geometry")
            if geo is None:
                continue
            mesh, box = geo.find("mesh"), geo.find("box")
            if mesh is not None:
                path = models_dir / mesh.find("uri").text.replace("model://", "")
                s = mesh.find("scale")
                sc = np.diag([float(v) for v in s.text.split()] + [1.0]) if s is not None else np.eye(4)
                for tris, m in dae_triangles(path):
                    yield transform(tris, t @ sc @ m)
            elif box is not None:
                yield transform(box_triangles([float(v) for v in box.find("size").text.split()]), t)


def transform(tris, m):
    h = np.concatenate([tris, np.ones(tris.shape[:2] + (1,))], axis=2)
    return (h @ m.T)[..., :3]


def main():
    world_path, models_dir, out = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
    world = ET.parse(world_path).getroot().find("world")

    def gather(kind, zmin, zmax):
        shapes = []

        def add_include(inc, t):
            name = inc.find("uri").text.replace("model://", "")
            sdf = ET.parse(models_dir / name / "model.sdf").getroot().find("model")
            shapes.extend(collisions(sdf, t @ pose_of(inc), models_dir, kind))

        for el in world:
            if tag(el) == "include":
                add_include(el, np.eye(4))
            elif tag(el) == "model":
                t = pose_of(el)
                for inc in el.findall("include"):
                    add_include(inc, t)
                shapes.extend(collisions(el, t, models_dir, kind))
        tris = np.concatenate([s for s in shapes if len(s)])
        tris = tris[(tris[..., 2].max(axis=1) >= zmin) & (tris[..., 2].min(axis=1) <= zmax)]
        print(f"{len(tris)} {kind} triangles in band z {zmin}..{zmax}")
        return tris

    # Safety layer: whole-body collision shapes. Lidar layer: what the simulated lidar
    # actually sees - gpu_lidar renders visuals at the sensor height (~0.04 m above floor),
    # so AMCL needs those cells occupied to match the scans. Map = union of both.
    tris = np.concatenate([gather("collision", 0.05, 1.0), gather("visual", LIDAR_Z - 0.01, LIDAR_Z + 0.01)])

    xy = tris[..., :2]
    x0, y0 = xy[..., 0].min() - 1.0, xy[..., 1].min() - 1.0
    w = int((xy[..., 0].max() + 1.0 - x0) / RES) + 1
    h = int((xy[..., 1].max() + 1.0 - y0) / RES) + 1
    occ = np.zeros((h, w), bool)
    for a, b, c in xy:
        n = max(2, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a),
                                     np.linalg.norm(c - b)) / (RES / 2))))
        u, v = np.meshgrid(np.linspace(0, 1, n + 1), np.linspace(0, 1, n + 1))
        k = u + v <= 1
        pts = a + np.outer(u[k], b - a) + np.outer(v[k], c - a)
        occ[((pts[:, 1] - y0) / RES).astype(int), ((pts[:, 0] - x0) / RES).astype(int)] = True

    img = np.where(occ, 0, 254).astype(np.uint8)
    lab, _ = label(~occ)
    ext = lab == lab[0, 0]
    sy_, sx_ = int((0 - y0) / RES), int((0 - x0) / RES)
    if ext[sy_, sx_]:
        print("warning: spawn cell connects to the outside (open door/window); no unknown fill")
    else:
        img[ext] = 205
    Image.fromarray(np.flipud(img)).save(out + ".pgm")
    name = Path(out).name
    Path(out + ".yaml").write_text(
        f"image: {name}.pgm\nmode: trinary\nresolution: {RES}\n"
        f"origin: [{x0:.3f}, {y0:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"wrote {out}.pgm {w}x{h} origin ({x0:.2f}, {y0:.2f})")


if __name__ == "__main__":
    main()
