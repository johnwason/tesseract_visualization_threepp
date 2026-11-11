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
RPY = "1.5708 0 1.5708"


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


def bounds(path):
    """The axis aligned bounds of the default scene, with node transforms applied"""
    doc = read_gltf_json(path)
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

    identity = [[float(r == c) for c in range(4)] for r in range(4)]
    for root in doc["scenes"][doc.get("scene", 0)]["nodes"]:
        walk(root, identity)
    return lo, hi


def xml_escape(text):
    return quoteattr(text)[1:-1]


def visual(path, size, by_width):
    lo, hi = bounds(path)
    extent = (hi[0] - lo[0]) if by_width else (hi[1] - lo[1])
    s = size / extent
    c = [(a + b) / 2 for a, b in zip(lo, hi)]
    # A mesh point p lands at R (s p) + xyz. Center x and z on the joint and put the bottom on the floor.
    xyz = (-c[2] * s, -c[0] * s, -lo[1] * s)
    return (f'<visual><origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{RPY}"/>'
            f'<geometry><mesh filename="{xml_escape(str(path))}" scale="{s:.6f} {s:.6f} {s:.6f}"/></geometry></visual>')


def write_urdf(out_dir, turntable, boards):
    lines = ['<robot name="pbr_models" tesseract:make_convex="false" xmlns:tesseract="http://tesseract">',
             '  <link name="world"/>']
    for i, (name, _, _, size) in enumerate(turntable):
        y = (i - (len(turntable) - 1) / 2) * 1.1
        lines.append(f'  <link name="{name}">{visual(out_dir / f"{name}.glb", size, False)}</link>')
        lines.append(f'  <joint name="{name}_turntable" type="continuous"><parent link="world"/><child link="{name}"/>'
                     f'<origin xyz="0 {y:.3f} 0"/><axis xyz="0 0 1"/></joint>')
    for i, (name, _, _, size) in enumerate(boards):
        y = (i - (len(boards) - 1) / 2) * 2.8
        lines.append(f'  <link name="{name}">{visual(out_dir / f"{name}.glb", size, True)}</link>')
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
