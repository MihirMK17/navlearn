#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Occupancy grid from an SDF world's collision geometry, sliced at the sensor height.

Why this exists
    Every localization result in this paper is a claim about a filter's behaviour
    GIVEN a map, and a map of unstated provenance makes the claim unciteable. The maps
    shipped with the AWS RoboMaker worlds were built by SLAM with a different robot,
    whose LiDAR sits 2-3 cm higher than ours; whether that matters is a measurement,
    not an argument, and nothing in the repository could take the measurement.

    This takes it. The world's collision geometry IS the ground truth the simulator
    itself uses for ray intersection, so a horizontal slice of it at the LiDAR height
    is exactly what a noiseless scan would see. That gives two things a SLAM map cannot:
    a map with no sensor or drift error to confound the filter's own behaviour, and a
    reference against which an existing map can be checked.

What it is NOT
    Not a replacement for SLAM in deployment. A geometric slice contains geometry the
    robot could never observe (behind walls, inside closed rooms), so it is a map of the
    world rather than a map of what is observable. For evaluating a filter against a
    known world that is the stronger choice -- a critic cannot blame the map -- but it
    is a different object from what a robot builds, and the paper must say so.

Method
    1. `ign sdf -p` expands the world's <include> graph with poses resolved. This needs
       SDF_PATH, NOT GZ_SIM_RESOURCE_PATH: the standalone CLI uses libsdformat's file
       callback, which the gz-sim variable does not feed. With the wrong variable the
       command still exits successfully and emits an unexpanded 105-line stub, which
       would silently produce a map of an empty world.
    2. Every collision geometry becomes triangles in world coordinates. Meshes are read
       with pycollada and scaled by the document's own `unitmeter` -- the AWS meshes are
       authored in centimetres, and skipping that step yields a map 100x too large that
       still looks plausible at a glance.
    3. Triangles are intersected with the plane z = height, giving the cross-section as
       line segments.
    4. Segments are rasterized as occupied cells; free space is flood-filled from a seed
       that must itself be free; everything unreached stays unknown.

Usage
    Generate:
        python3 world_to_map.py generate --world <path.world> --models <dir> \
            --height 0.1538 --resolution 0.05 --out results/maps/small_warehouse
    Validate an existing map against the world geometry:
        python3 world_to_map.py validate --world <path.world> --models <dir> \
            --height 0.1538 --map src/bumperbot/bumperbot_mapping/maps/small_house/map.yaml \
            --out results/maps/small_house_validation.md
"""

import argparse
import collections
import math
import os
import pathlib
import re
import subprocess
import sys

import numpy as np

# The SDF is produced locally by `ign sdf -p` from vendored world files, so it is not
# untrusted input -- but a world file is still a document pulled from an upstream
# project, and the hardened parser costs nothing.
from defusedxml import ElementTree as ET

# ROS map_server pixel conventions with negate=0.
PGM_OCCUPIED = 0
PGM_FREE = 254
PGM_UNKNOWN = 205

# Cross-section segments are sampled at this fraction of a cell when rasterizing. A
# third of a cell cannot step over a cell diagonally, which a full-cell step can.
_RASTER_STEP_CELLS = 1.0 / 3.0


def expand_world(world_path, models_dir):
    """Run `ign sdf -p` with SDF_PATH set, returning the expanded SDF text.

    Refuses a result that did not expand. An unresolved world still exits zero and
    prints a short stub, and a stub rasterizes to an empty map -- a failure that looks
    exactly like a world with no obstacles in it.
    """
    env = dict(os.environ)
    env["SDF_PATH"] = str(models_dir)
    proc = subprocess.run(
        ["ign", "sdf", "-p", str(world_path)],
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"ign sdf -p failed: {proc.stderr[:400]}")
    if "Tried to use callback" in proc.stderr:
        raise RuntimeError(
            "ign sdf could not resolve model:// URIs. SDF_PATH must point at the "
            "directory holding the model folders; GZ_SIM_RESOURCE_PATH does not work "
            f"for the standalone CLI. Got SDF_PATH={models_dir}"
        )
    text = proc.stdout
    if "<include>" in text:
        raise RuntimeError("expanded SDF still contains <include>; expansion failed")
    return text


def parse_pose(text):
    """'x y z roll pitch yaw' -> 6-tuple. Missing components are zero."""
    if text is None:
        return (0.0,) * 6
    parts = [float(v) for v in text.split()]
    parts += [0.0] * (6 - len(parts))
    return tuple(parts[:6])


def pose_matrix(pose):
    """SDF pose -> 4x4 homogeneous transform, roll-pitch-yaw applied as Rz*Ry*Rx."""
    x, y, z, roll, pitch, yaw = pose
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = (x, y, z)
    return out


def box_triangles(sx, sy, sz):
    """Triangulated axis-aligned box centred on the origin."""
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    c = np.array(
        [
            [-hx, -hy, -hz],
            [hx, -hy, -hz],
            [hx, hy, -hz],
            [-hx, hy, -hz],
            [-hx, -hy, hz],
            [hx, -hy, hz],
            [hx, hy, hz],
            [-hx, hy, hz],
        ]
    )
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    return np.array([[c[i], c[j], c[k]] for i, j, k in faces])


def cylinder_triangles(radius, length, segments=32):
    """Triangulated cylinder about the z axis, centred on the origin."""
    hz = length / 2.0
    ang = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    ring = np.stack([radius * np.cos(ang), radius * np.sin(ang)], axis=1)
    tris = []
    for i in range(segments):
        j = (i + 1) % segments
        a = np.array([ring[i][0], ring[i][1], -hz])
        b = np.array([ring[j][0], ring[j][1], -hz])
        c = np.array([ring[j][0], ring[j][1], hz])
        d = np.array([ring[i][0], ring[i][1], hz])
        tris += [[a, b, c], [a, c, d]]
        tris += [[np.array([0, 0, -hz]), b, a], [np.array([0, 0, hz]), d, c]]
    return np.array(tris)


def sphere_triangles(radius, segments=16):
    """Triangulated UV sphere centred on the origin."""
    tris = []
    for i in range(segments // 2):
        p0 = math.pi * i / (segments // 2)
        p1 = math.pi * (i + 1) / (segments // 2)
        for j in range(segments):
            t0 = 2.0 * math.pi * j / segments
            t1 = 2.0 * math.pi * (j + 1) / segments

            def pt(phi, theta):
                return np.array(
                    [
                        radius * math.sin(phi) * math.cos(theta),
                        radius * math.sin(phi) * math.sin(theta),
                        radius * math.cos(phi),
                    ]
                )

            a, b, c, d = pt(p0, t0), pt(p0, t1), pt(p1, t1), pt(p1, t0)
            tris += [[a, b, c], [a, c, d]]
    return np.array(tris)


def load_mesh_triangles(mesh_path):
    """Triangles from a COLLADA file, in metres.

    The document's own `unitmeter` is applied. The AWS collision meshes declare
    centimetres; ignoring that produces a world 100x too large, which still rasterizes
    to a map-shaped image and is not obviously wrong until localization fails on it.
    A Y_UP document is rotated to Z_UP, since SDF is Z_UP and pycollada does not convert.
    """
    import collada

    # The installed pycollada (0.4.1) cannot load animations in this environment, for
    # two independent reasons: collada/animation.py references DaeError in an except
    # clause without importing it, and collada/source.py uses numpy.unicode_, removed
    # in NumPy 2.0. Either raises before any geometry is read. The small_house assets
    # are animated and the AWS ones are not, which is why this surfaced only on the
    # second world.
    #
    # Animations are irrelevant here: a collision cross-section is taken from the rest
    # pose, which is also what the simulator uses for the static world. So the animation
    # pass is disabled rather than repaired -- narrower than patching two library bugs,
    # and it cannot change any geometry that is read.
    if not getattr(collada.Collada, "_navlearn_animations_disabled", False):
        collada.Collada._loadAnimations = lambda self: None
        collada.Collada._navlearn_animations_disabled = True

    doc = collada.Collada(str(mesh_path))
    unit = doc.assetInfo.unitmeter or 1.0
    chunks = []
    for geom in doc.scene.objects("geometry"):
        for prim in geom.primitives():
            # A COLLADA primitive may be a triangle set, a polygon list, or a line set.
            # Polygon lists carry a triangleset() conversion; line sets have no faces
            # and cannot contribute a cross-section. Assuming triangles and reshaping
            # blindly raises on the first quad -- which is how the small_house assets
            # differ from the AWS ones.
            faces = prim
            if not hasattr(faces, "vertex_index") or faces.vertex_index is None:
                continue
            if hasattr(prim, "triangleset") and not isinstance(
                prim, collada.triangleset.TriangleSet
            ):
                try:
                    faces = prim.triangleset()
                except Exception:  # noqa: BLE001 - a non-triangulable primitive
                    continue
            idx = np.asarray(faces.vertex_index)
            if idx.ndim != 2 or idx.shape[-1] != 3:
                continue
            chunks.append(np.asarray(faces.vertex)[idx].reshape(-1, 3, 3))
    if not chunks:
        return np.zeros((0, 3, 3))
    tris = np.concatenate(chunks, axis=0) * unit
    if (doc.assetInfo.upaxis or "").upper().startswith("Y"):
        rot = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
        tris = tris @ rot.T
    return tris


def resolve_mesh_uri(uri, models_dir):
    """model://pkg/rest -> filesystem path under models_dir. None when unresolvable."""
    match = re.match(r"^model://(.+)$", uri.strip())
    rel = match.group(1) if match else uri.strip().replace("file://", "")
    candidate = pathlib.Path(models_dir) / rel
    if candidate.exists():
        return candidate
    # Case differences in the extension are common in these assets.
    parent, name = candidate.parent, candidate.name
    if parent.is_dir():
        for entry in parent.iterdir():
            if entry.name.lower() == name.lower():
                return entry
    return None


def geometry_triangles(geom_el, models_dir, warnings):
    """Triangles for one <geometry>, in its own frame."""
    mesh_el = geom_el.find("mesh")
    if mesh_el is not None:
        uri_el = mesh_el.find("uri")
        if uri_el is None:
            return np.zeros((0, 3, 3))
        path = resolve_mesh_uri(uri_el.text, models_dir)
        if path is None:
            warnings.append(f"unresolved mesh uri: {uri_el.text}")
            return np.zeros((0, 3, 3))
        tris = load_mesh_triangles(path)
        scale_el = mesh_el.find("scale")
        if scale_el is not None:
            tris = tris * np.array([float(v) for v in scale_el.text.split()])
        return tris

    box_el = geom_el.find("box")
    if box_el is not None:
        sx, sy, sz = (float(v) for v in box_el.find("size").text.split())
        return box_triangles(sx, sy, sz)

    cyl_el = geom_el.find("cylinder")
    if cyl_el is not None:
        return cylinder_triangles(
            float(cyl_el.find("radius").text), float(cyl_el.find("length").text)
        )

    sph_el = geom_el.find("sphere")
    if sph_el is not None:
        return sphere_triangles(float(sph_el.find("radius").text))

    if geom_el.find("plane") is not None:
        # The ground plane is not an obstacle to a horizontal scan and would otherwise
        # fill the entire grid at any height that grazes it.
        return np.zeros((0, 3, 3))

    warnings.append(f"unhandled geometry: {[c.tag for c in geom_el]}")
    return np.zeros((0, 3, 3))


def collect_world_triangles(sdf_text, models_dir):
    """All collision triangles in the world, in world coordinates.

    Walks world -> model -> link -> collision, composing poses down the chain. Nested
    models are followed, since the AWS worlds group furniture that way.
    """
    root = ET.fromstring(sdf_text)
    world = root.find("world")
    if world is None:
        raise RuntimeError("no <world> in the expanded SDF")

    warnings = []
    chunks = []
    counts = collections.Counter()

    def walk_model(model_el, parent_tf):
        model_tf = parent_tf @ pose_matrix(parse_pose(model_el.findtext("pose")))
        for link_el in model_el.findall("link"):
            link_tf = model_tf @ pose_matrix(parse_pose(link_el.findtext("pose")))
            for col_el in link_el.findall("collision"):
                col_tf = link_tf @ pose_matrix(parse_pose(col_el.findtext("pose")))
                geom_el = col_el.find("geometry")
                if geom_el is None:
                    continue
                tris = geometry_triangles(geom_el, models_dir, warnings)
                if len(tris) == 0:
                    continue
                flat = tris.reshape(-1, 3)
                homo = np.hstack([flat, np.ones((len(flat), 1))])
                chunks.append((homo @ col_tf.T)[:, :3].reshape(-1, 3, 3))
                counts["collisions"] += 1
        for nested in model_el.findall("model"):
            walk_model(nested, model_tf)

    for model_el in world.findall("model"):
        walk_model(model_el, np.eye(4))
        counts["models"] += 1

    if not chunks:
        raise RuntimeError("no collision geometry found in the world")
    return np.concatenate(chunks, axis=0), counts, warnings


def slice_triangles(tris, height):
    """Intersect triangles with the plane z = height, returning (M, 2, 2) segments.

    A triangle contributes a segment when its vertices straddle the plane. Triangles
    lying exactly in the plane are skipped: they are degenerate as a cross-section and
    in practice come from flat surfaces the scan would pass over, not intersect.
    """
    if len(tris) == 0:
        return np.zeros((0, 2, 2))
    segments = []
    z = tris[:, :, 2] - height
    straddles = ~(np.all(z > 0, axis=1) | np.all(z < 0, axis=1))
    for tri, dz in zip(tris[straddles], z[straddles]):
        pts = []
        for i in range(3):
            j = (i + 1) % 3
            a, b = dz[i], dz[j]
            if a == 0.0:
                pts.append(tri[i][:2])
            if (a > 0.0) != (b > 0.0) and (a - b) != 0.0:
                t = a / (a - b)
                pts.append(tri[i][:2] + t * (tri[j][:2] - tri[i][:2]))
        if len(pts) >= 2:
            segments.append([pts[0], pts[1]])
    return np.array(segments) if segments else np.zeros((0, 2, 2))


def rasterize(segments, resolution, origin, shape):
    """Boolean occupied grid of `shape` (rows, cols); origin is (min_x, min_y)."""
    grid = np.zeros(shape, dtype=bool)
    if len(segments) == 0:
        return grid
    rows, cols = shape
    step = resolution * _RASTER_STEP_CELLS
    for p0, p1 in segments:
        length = float(np.hypot(*(p1 - p0)))
        n = max(int(length / step) + 1, 2)
        for t in np.linspace(0.0, 1.0, n):
            x, y = p0 + t * (p1 - p0)
            col = int((x - origin[0]) / resolution)
            row = int((y - origin[1]) / resolution)
            if 0 <= row < rows and 0 <= col < cols:
                grid[row, col] = True
    return grid


def flood_free(occupied, seed_rc):
    """Free-space mask reachable from seed_rc without crossing an occupied cell."""
    rows, cols = occupied.shape
    free = np.zeros_like(occupied)
    sr, sc = seed_rc
    if not (0 <= sr < rows and 0 <= sc < cols):
        raise RuntimeError(f"flood seed {seed_rc} is outside the grid")
    if occupied[sr, sc]:
        raise RuntimeError(
            f"flood seed {seed_rc} is on an occupied cell; free space cannot be "
            "identified from inside an obstacle"
        )
    stack = [(sr, sc)]
    free[sr, sc] = True
    while stack:
        r, c = stack.pop()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if (
                0 <= nr < rows
                and 0 <= nc < cols
                and not free[nr, nc]
                and not occupied[nr, nc]
            ):
                free[nr, nc] = True
                stack.append((nr, nc))
    return free


def to_pgm_array(occupied, free):
    """ROS map_server pixel values, row 0 = top of the image."""
    img = np.full(occupied.shape, PGM_UNKNOWN, dtype=np.uint8)
    img[free] = PGM_FREE
    img[occupied] = PGM_OCCUPIED
    return np.flipud(img)


def write_pgm(path, img):
    with open(path, "wb") as handle:
        handle.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        handle.write(img.tobytes())


def read_pgm(path):
    """Minimal binary P5 reader; returns the pixel array with row 0 at the top."""
    with open(path, "rb") as handle:
        data = handle.read()
    if not data.startswith(b"P5"):
        raise RuntimeError(f"{path}: not a binary P5 PGM")
    fields, pos = [], 2
    while len(fields) < 3:
        while pos < len(data) and data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b"#":
            while data[pos : pos + 1] not in (b"\n", b""):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos : pos + 1].isspace():
            pos += 1
        fields.append(int(data[start:pos]))
    pos += 1
    width, height, _ = fields
    return np.frombuffer(data[pos : pos + width * height], dtype=np.uint8).reshape(
        height, width
    )


def read_map_yaml(path):
    """resolution / origin / image from a map_server YAML, without a YAML dependency."""
    out = {}
    for line in pathlib.Path(path).read_text().splitlines():
        if ":" not in line or line.strip().startswith("#"):
            continue
        key, _, value = line.partition(":")
        out[key.strip()] = value.strip()
    origin = [float(v) for v in out["origin"].strip("[]").split(",")]
    return {
        "image": (pathlib.Path(path).parent / out["image"]).resolve(),
        "resolution": float(out["resolution"]),
        "origin": origin,
    }


def build(world, models, height, resolution, margin=1.0, seed_xy=(0.0, 0.0)):
    """Everything from a world path to (occupied, free, origin, counts, warnings)."""
    sdf_text = expand_world(world, models)
    tris, counts, warnings = collect_world_triangles(sdf_text, models)
    segments = slice_triangles(tris, height)
    if len(segments) == 0:
        raise RuntimeError(
            f"the world has no collision geometry crossing z={height} m; either the "
            "height is above everything in the world or the units are wrong"
        )

    pts = segments.reshape(-1, 2)
    origin = (pts[:, 0].min() - margin, pts[:, 1].min() - margin)
    width = int(math.ceil((pts[:, 0].max() + margin - origin[0]) / resolution)) + 1
    heightc = int(math.ceil((pts[:, 1].max() + margin - origin[1]) / resolution)) + 1

    occupied = rasterize(segments, resolution, origin, (heightc, width))
    seed_rc = (
        int((seed_xy[1] - origin[1]) / resolution),
        int((seed_xy[0] - origin[0]) / resolution),
    )
    free = flood_free(occupied, seed_rc)
    counts["triangles"] = len(tris)
    counts["segments"] = len(segments)
    return occupied, free, origin, counts, warnings


def cmd_generate(args):
    occupied, free, origin, counts, warnings = build(
        args.world,
        args.models,
        args.height,
        args.resolution,
        seed_xy=(args.seed_x, args.seed_y),
    )
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    img = to_pgm_array(occupied, free)
    write_pgm(out / "map.pgm", img)
    (out / "map.yaml").write_text(
        "image: map.pgm\n"
        f"resolution: {args.resolution:.6f}\n"
        f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.000000]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    (out / "PROVENANCE.md").write_text(
        f"# {pathlib.Path(args.world).name} map provenance\n\n"
        f"Generated by world_to_map.py from the world's COLLISION geometry -- the same\n"
        f"geometry the simulator uses for ray intersection -- sliced at the robot's\n"
        f"LiDAR height. No SLAM, no sensor noise, no pose drift: map error is not a\n"
        f"confound in any result computed against this map.\n\n"
        f"| | |\n|---|---|\n"
        f"| world | `{args.world}` |\n"
        f"| slice height | {args.height} m above the ground plane |\n"
        f"| resolution | {args.resolution} m/cell |\n"
        f"| origin | {origin[0]:.4f}, {origin[1]:.4f} |\n"
        f"| models | {counts['models']} |\n"
        f"| collision geometries | {counts['collisions']} |\n"
        f"| triangles | {counts['triangles']} |\n"
        f"| cross-section segments | {counts['segments']} |\n"
        f"| occupied cells | {int(occupied.sum())} |\n"
        f"| free cells | {int(free.sum())} |\n\n"
        f"Contains geometry no robot could observe (behind walls, inside closed rooms),\n"
        f"so it is a map of the world rather than of what is observable from within it.\n"
        + ("\nWarnings:\n" + "\n".join(f"- {w}" for w in warnings) if warnings else "")
    )
    print(f"wrote {out}/map.pgm  ({occupied.sum()} occupied, {free.sum()} free cells)")
    for w in warnings:
        print(f"WARNING: {w}", file=sys.stderr)
    return 0


def compare(existing_occ, existing_res, existing_origin, gen_occ, gen_res, gen_origin):
    """Nearest-neighbour distance from each existing occupied cell to a generated one.

    Distances rather than an overlap score: two maps of the same world can disagree on
    wall THICKNESS while agreeing perfectly on wall POSITION, and an IoU would call that
    a large disagreement while a localization filter would not notice it at all.
    """

    def cells_to_xy(mask, res, origin):
        rows, cols = np.nonzero(mask)
        return np.stack(
            [origin[0] + (cols + 0.5) * res, origin[1] + (rows + 0.5) * res], axis=1
        )

    a = cells_to_xy(existing_occ, existing_res, existing_origin)
    b = cells_to_xy(gen_occ, gen_res, gen_origin)
    if len(a) == 0 or len(b) == 0:
        return None
    dists = []
    for start in range(0, len(a), 2000):
        chunk = a[start : start + 2000]
        d = np.sqrt(((chunk[:, None, :] - b[None, :, :]) ** 2).sum(axis=2))
        dists.append(d.min(axis=1))
    return np.concatenate(dists)


def cmd_validate(args):
    meta = read_map_yaml(args.map)
    img = read_pgm(meta["image"])
    existing_occ = np.flipud(img) < 90  # map_server: dark pixels are occupied

    occupied, _, origin, counts, warnings = build(
        args.world,
        args.models,
        args.height,
        meta["resolution"],
        seed_xy=(args.seed_x, args.seed_y),
    )

    d = compare(
        existing_occ,
        meta["resolution"],
        meta["origin"],
        occupied,
        meta["resolution"],
        origin,
    )
    if d is None:
        raise RuntimeError("one of the maps has no occupied cells")

    pct = {p: float(np.percentile(d, p)) for p in (50, 75, 90, 95, 99)}
    within = {t: float((d <= t).mean() * 100.0) for t in (0.05, 0.10, 0.20, 0.50)}
    lines = [
        f"# Map validation: {pathlib.Path(args.map).parent.name}",
        "",
        f"Existing map checked against the world's collision geometry sliced at "
        f"{args.height} m.",
        "",
        "Reported as distance from each occupied cell of the existing map to the "
        "nearest occupied cell of the geometric slice. Distance rather than overlap: "
        "two maps can disagree on wall thickness while agreeing on wall position, and "
        "an overlap score would call that a large disagreement where a particle filter "
        "would not notice it.",
        "",
        "| quantile | distance (m) |",
        "|---|---|",
    ]
    lines += [f"| p{p} | {v:.3f} |" for p, v in pct.items()]
    lines += ["", "| within | share of occupied cells |", "|---|---|"]
    lines += [f"| {t:.2f} m | {v:.1f}% |" for t, v in within.items()]
    lines += [
        "",
        f"- existing map occupied cells: {int(existing_occ.sum())}",
        f"- geometric slice occupied cells: {int(occupied.sum())}",
        f"- models {counts['models']}, collisions {counts['collisions']}, "
        f"triangles {counts['triangles']}, segments {counts['segments']}",
    ]
    if warnings:
        lines += ["", "Warnings:"] + [f"- {w}" for w in warnings]
    text = "\n".join(lines)
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    print(text)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    for name in ("generate", "validate"):
        p = sub.add_parser(name)
        p.add_argument("--world", required=True)
        p.add_argument(
            "--models",
            required=True,
            help="directory holding the model folders (becomes SDF_PATH)",
        )
        p.add_argument("--height", type=float, required=True)
        p.add_argument("--seed-x", type=float, default=0.0)
        p.add_argument("--seed-y", type=float, default=0.0)
        p.add_argument("--out", required=True)
        if name == "generate":
            p.add_argument("--resolution", type=float, default=0.05)
        else:
            p.add_argument("--map", required=True, help="existing map.yaml")
    args = parser.parse_args()
    return cmd_generate(args) if args.cmd == "generate" else cmd_validate(args)


if __name__ == "__main__":
    sys.exit(main())
