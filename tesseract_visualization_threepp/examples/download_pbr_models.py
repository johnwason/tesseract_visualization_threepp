#!/usr/bin/env python3
"""Download glTF 2.0 sample models for the PBR models example, and write the scene URDF for them.

The models come from the Khronos glTF-Sample-Assets repository at a pinned commit. They are not part of this
repository. Each model's license is listed in ATTRIBUTION.md, written next to the models.

usage: download_pbr_models.py [output_dir] [--include-noncommercial]

output_dir defaults to ./pbr_models. It receives the .glb files, ATTRIBUTION.md and pbr_models.urdf. The URDF
refers to the models by absolute path, which tesseract resolves on every platform; after moving the directory, run
this again to rewrite it. Models already downloaded are kept.
--include-noncommercial also downloads DamagedHelmet, which is partly licensed CC-BY-NC-4.0.

Only the Python standard library is used.
"""
import argparse
import itertools
import json
import math
import pathlib
import struct
import sys
import urllib.request
from xml.sax.saxutils import quoteattr

ASSETS_COMMIT = "8f492390ddc7a2ae221a1369ef560589cf8c70fb"
ASSETS_URL = "https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/{commit}/Models/{name}/glTF-Binary/{name}.glb"
ASSETS_REPO = "https://github.com/KhronosGroup/glTF-Sample-Assets"

# name, license, credit, display size in meters (height for turntable models, width for test boards)
TURNTABLE = [
    ("WaterBottle", "CC0-1.0", "", 0.8),
    ("BoomBox", "CC0-1.0", "", 0.6),
    ("Lantern", "CC0-1.0", "", 0.9),
    ("DamagedHelmet", "CC-BY-4.0 and CC-BY-NC-4.0", "ctxwing; earlier version by theblueturtle_", 0.7),
    ("Corset", "CC0-1.0", "", 0.8),
    ("Avocado", "CC0-1.0", "", 0.7),
]
BOARDS = [
    ("MetalRoughSpheres", "CC-BY-4.0", "Ed Mackey", 2.4),
    ("AlphaBlendModeTest", "CC-BY-4.0", "Ed Mackey", 2.4),
    ("EmissiveStrengthTest", "CC-BY-4.0", "Ed Mackey", 2.4),
    ("NormalTangentMirrorTest", "CC-BY-4.0", "Ed Mackey", 2.2),
]
NONCOMMERCIAL = {"DamagedHelmet"}

# glTF is +Y up and faces +Z; tesseract is +Z up. This rpy maps glTF y->Z, z->X and x->Y, so the models face +X,
# toward the viewers' default camera.
RPY = (math.pi / 2, 0.0, math.pi / 2)


def download(name, path):
    url = ASSETS_URL.format(commit=ASSETS_COMMIT, name=name)
    request = urllib.request.Request(url, headers={"User-Agent": "tesseract-pbr-example"})
    with urllib.request.urlopen(request, timeout=120) as response:
        data = response.read()
    if data[:4] != b"glTF":
        raise RuntimeError(f"{url} did not return a binary glTF file")
    tmp = path.with_suffix(".part")
    tmp.write_bytes(data)
    tmp.replace(path)


def read_gltf_json(path):
    with open(path, "rb") as f:
        magic, _, _ = struct.unpack("<III", f.read(12))
        length, chunk_type = struct.unpack("<II", f.read(8))
        if magic != 0x46546C67 or chunk_type != 0x4E4F534A:
            raise RuntimeError(f"{path} is not a binary glTF file")
        return json.loads(f.read(length))


def node_matrix(node):
    if "matrix" in node:
        m = node["matrix"]
        return [[m[c * 4 + r] for c in range(4)] for r in range(4)]
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    rot = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
           [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
           [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    t = node.get("translation", [0, 0, 0])
    s = node.get("scale", [1, 1, 1])
    return [[rot[r][c] * s[c] for c in range(3)] + [t[r]] for r in range(3)] + [[0, 0, 0, 1]]


def multiply(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def bounds(doc):
    """The axis aligned bounds of the default scene, with every node transform applied"""
    lo, hi = [float("inf")] * 3, [float("-inf")] * 3

    def walk(index, parent):
        node = doc["nodes"][index]
        m = multiply(parent, node_matrix(node))
        if "mesh" in node:
            for primitive in doc["meshes"][node["mesh"]]["primitives"]:
                accessor = doc["accessors"][primitive["attributes"]["POSITION"]]
                for corner in itertools.product(*zip(accessor["min"], accessor["max"])):
                    p = [sum(m[r][k] * corner[k] for k in range(3)) + m[r][3] for r in range(3)]
                    for k in range(3):
                        lo[k], hi[k] = min(lo[k], p[k]), max(hi[k], p[k])
        for child in node.get("children", []):
            walk(child, m)

    for root in scene_roots(doc):
        walk(root, IDENTITY)
    return lo, hi


def scene_roots(doc):
    return doc["scenes"][doc.get("scene", 0)]["nodes"]


def dropped_root_transform(doc, name):
    """The transform tesseract leaves out, as a uniform scale, a rotation and a translation.

    tesseract resets the transform of assimp's root node, as RViz does, to undo assimp's conversion of Z-up formats to
    Y-up. When a glTF scene has a single root node, assimp makes that node its root, so that node's transform is
    dropped. The URDF puts it back.
    """
    roots = scene_roots(doc)
    if len(roots) != 1:
        return 1.0, IDENTITY3, [0.0, 0.0, 0.0]
    m = node_matrix(doc["nodes"][roots[0]])
    columns = [[m[r][c] for r in range(3)] for c in range(3)]
    scales = [math.sqrt(sum(v * v for v in col)) for col in columns]
    rot = [[m[r][c] / scales[c] for c in range(3)] for r in range(3)]
    if max(scales) - min(scales) > 1e-6 * max(scales) or abs(det3(rot) - 1) > 1e-6:
        raise RuntimeError(f"{name}: the root node transform has a non-uniform scale or a reflection")
    return scales[0], rot, [m[r][3] for r in range(3)]


def det3(a):
    return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))


def rpy_to_matrix(roll, pitch, yaw):
    """URDF convention: R = Rz(yaw) Ry(pitch) Rx(roll)"""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_to_rpy(r):
    return (math.atan2(r[2][1], r[2][2]), math.asin(max(-1.0, min(1.0, -r[2][0]))), math.atan2(r[1][0], r[0][0]))


def mul3(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)] for r in range(3)]


IDENTITY = [[float(r == c) for c in range(4)] for r in range(4)]
IDENTITY3 = [[float(r == c) for c in range(3)] for r in range(3)]


def xml_escape(text):
    return quoteattr(text)[1:-1]


def visual(path, name, size, by_width):
    doc = read_gltf_json(path)
    lo, hi = bounds(doc)
    extent = (hi[0] - lo[0]) if by_width else (hi[1] - lo[1])
    s = size / extent
    c = [(a + b) / 2 for a, b in zip(lo, hi)]
    # A model point q, with every node transform applied, should land at B (s q) + xyz, where B is the RPY rotation.
    # That centers x and z on the joint and puts the bottom on the floor.
    xyz = [-c[2] * s, -c[0] * s, -lo[1] * s]
    # tesseract loads p, with q = k R p + t, where k, R and t are the dropped root transform. So the URDF gets the
    # rotation B R, the mesh scale s k, and the origin xyz + s B t.
    base = rpy_to_matrix(*RPY)
    k, rot, t = dropped_root_transform(doc, name)
    rpy = matrix_to_rpy(mul3(base, rot))
    xyz = [xyz[r] + s * sum(base[r][j] * t[j] for j in range(3)) for r in range(3)]
    ms = s * k
    return (f'<visual><origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" '
            f'rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>'
            f'<geometry><mesh filename="{xml_escape(str(path))}" scale="{ms:.6f} {ms:.6f} {ms:.6f}"/></geometry></visual>')


def write_urdf(out_dir, turntable, boards):
    lines = ['<robot name="pbr_models" tesseract:make_convex="false" xmlns:tesseract="http://tesseract">',
             '  <link name="world"/>']
    for i, (name, _, _, size) in enumerate(turntable):
        y = (i - (len(turntable) - 1) / 2) * 1.1
        lines.append(f'  <link name="{name}">{visual(out_dir / f"{name}.glb", name, size, False)}</link>')
        lines.append(f'  <joint name="{name}_turntable" type="continuous"><parent link="world"/><child link="{name}"/>'
                     f'<origin xyz="0 {y:.3f} 0"/><axis xyz="0 0 1"/></joint>')
    for i, (name, _, _, size) in enumerate(boards):
        y = (i - (len(boards) - 1) / 2) * 2.8
        lines.append(f'  <link name="{name}">{visual(out_dir / f"{name}.glb", name, size, True)}</link>')
        lines.append(f'  <joint name="{name}_joint" type="fixed"><parent link="world"/><child link="{name}"/>'
                     f'<origin xyz="-2.5 {y:.3f} 0"/></joint>')
    lines.append("</robot>")
    (out_dir / "pbr_models.urdf").write_text("\n".join(lines) + "\n")


def write_attribution(out_dir, models):
    rows = "\n".join(f"| {name} | {license} | {credit or 'see the model README in the repository'} |"
                     for name, license, credit, _ in models)
    (out_dir / "ATTRIBUTION.md").write_text(
        "# PBR example models\n\n"
        f"Downloaded unmodified from {ASSETS_REPO} at commit {ASSETS_COMMIT}\n"
        "(the glTF-Binary version of each model). Each model's folder in that repository has its README and license.\n\n"
        "| Model | License | Credit |\n| --- | --- | --- |\n" + rows + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("output_dir", nargs="?", default="pbr_models")
    parser.add_argument("--include-noncommercial", action="store_true",
                        help="also download DamagedHelmet, which is partly licensed CC-BY-NC-4.0")
    args = parser.parse_args()

    out_dir = pathlib.Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    turntable = [m for m in TURNTABLE if args.include_noncommercial or m[0] not in NONCOMMERCIAL]

    for name, license, _, _ in turntable + BOARDS:
        path = out_dir / f"{name}.glb"
        if path.exists():
            print(f"  have        {name}")
            continue
        print(f"  downloading {name} ({license})", flush=True)
        download(name, path)

    write_urdf(out_dir, turntable, BOARDS)
    write_attribution(out_dir, turntable + BOARDS)
    print(f"\nWrote {out_dir / 'pbr_models.urdf'}. Licenses are in {out_dir / 'ATTRIBUTION.md'}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
