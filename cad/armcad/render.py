"""Dependency-light renderer (numpy z-buffer) for documentation images."""
from __future__ import annotations

from typing import Dict, Sequence, Tuple

import cadquery as cq
import numpy as np
from PIL import Image

Color = Tuple[int, int, int]

PALETTE: Dict[str, Color] = {
    "printed": (240, 120, 30), "servo": (45, 48, 56), "horn": (200, 205, 212),
    "grid": (70, 74, 82), "tray": (60, 110, 160), "payload": (200, 40, 45),
    "dark": (55, 58, 66),
}


def color_for(name: str) -> Color:
    if name.startswith("servo"):
        return PALETTE["servo"]
    if name.startswith("horn"):
        return PALETTE["horn"]
    if name.startswith("grid"):
        return PALETTE["grid"]
    if name.startswith("electronics"):
        return PALETTE["tray"]
    if name.startswith("payload"):
        return PALETTE["payload"]
    if name.startswith(("finger", "turret_shim", "base")):
        return PALETTE["dark"]
    return PALETTE["printed"]


def _camera(view_dir: Sequence[float], up=(0.0, 0.0, 1.0)) -> np.ndarray:
    forward = np.asarray(view_dir, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    right /= np.linalg.norm(right)
    true_up = np.cross(right, forward)
    return np.stack([right, true_up, -forward])


def render(shapes: Dict[str, cq.Shape], path: str, view_dir=(1.0, 1.0, -0.7),
           size=(1400, 1000), background: Color = (245, 246, 248),
           tolerance: float = 0.25, colors: Dict[str, Color] = None,
           supersample: int = 2) -> None:
    """Orthographic flat-shaded render of named shapes to a PNG."""
    cam = _camera(view_dir)
    light = np.array([0.35, 0.45, 0.82])
    light /= np.linalg.norm(light)
    meshes = []
    for name, shape in shapes.items():
        verts, tris = shape.tessellate(tolerance, 0.3)
        v = np.array([(p.x, p.y, p.z) for p in verts])
        t = np.array(tris, dtype=int)
        if len(t):
            meshes.append((v, t, (colors or {}).get(name, color_for(name))))
    pts = np.concatenate([v @ cam.T for v, _, _ in meshes])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    w, h = size[0] * supersample, size[1] * supersample
    margin = 0.05
    scale = min(w * (1 - 2 * margin) / (hi[0] - lo[0]), h * (1 - 2 * margin) / (hi[1] - lo[1]))
    offset = np.array([w / 2 - scale * (lo[0] + hi[0]) / 2, h / 2 + scale * (lo[1] + hi[1]) / 2])

    image = np.empty((h, w, 3), dtype=np.uint8)
    image[:] = background
    depth = np.full((h, w), -np.inf)
    for v, t, color in meshes:
        p = v @ cam.T
        sx = p[:, 0] * scale + offset[0]
        sy = -p[:, 1] * scale + offset[1]
        tri_world = v[t]
        normals = np.cross(tri_world[:, 1] - tri_world[:, 0], tri_world[:, 2] - tri_world[:, 0])
        norm = np.linalg.norm(normals, axis=1)
        norm[norm == 0] = 1.0
        normals /= norm[:, None]
        facing = normals @ cam[2]
        # two-sided lighting: back-facing triangles are lit as if flipped
        shade = np.where(facing > 0, 0.38 + 0.62 * np.clip(normals @ light, 0, 1),
                         0.38 + 0.62 * np.clip(-(normals @ light), 0, 1))
        for (a, b, c), s in zip(t, shade):
            xs, ys, zs = sx[[a, b, c]], sy[[a, b, c]], p[[a, b, c], 2]
            x0, x1 = max(int(xs.min()), 0), min(int(xs.max()) + 1, w - 1)
            y0, y1 = max(int(ys.min()), 0), min(int(ys.max()) + 1, h - 1)
            if x0 > x1 or y0 > y1:
                continue
            den = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
            if abs(den) < 1e-9:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            w0 = ((ys[1] - ys[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gy - ys[2])) / den
            w1 = ((ys[2] - ys[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gy - ys[2])) / den
            w2 = 1.0 - w0 - w1
            z = w0 * zs[0] + w1 * zs[1] + w2 * zs[2]
            region = depth[y0:y1 + 1, x0:x1 + 1]
            mask = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6) & (z > region)
            if mask.any():
                region[mask] = z[mask]
                image[y0:y1 + 1, x0:x1 + 1][mask] = (np.array(color) * s).astype(np.uint8)
    # darken silhouette/depth edges so parts read clearly
    finite = np.where(np.isfinite(depth), depth, depth[np.isfinite(depth)].min() - 50)
    edge = (np.abs(np.diff(finite, axis=0, prepend=finite[:1])) +
            np.abs(np.diff(finite, axis=1, prepend=finite[:, :1]))) > 2.5
    image[edge] = (image[edge] * 0.45).astype(np.uint8)
    Image.fromarray(image).resize(size, Image.LANCZOS).save(path)
