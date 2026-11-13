#!/usr/bin/env python3
"""Preserve the bundled STL facets and normals in MuJoCo's visual OBJ assets.

MuJoCo regenerates normals when loading STL, smoothing the original facets.
OBJ supports explicit per-face-corner normals, so no geometry changes are needed.
Run this script after updating the source visual STL files. No third-party
dependencies are required; collision meshes and source STL files are untouched.
"""

from pathlib import Path
import math
import struct


def convert(source: Path, destination: Path) -> int:
    data = source.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid binary STL: {source}")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Invalid binary STL size: {source}")

    vertices = {}
    normals = []
    faces = []
    for index in range(count):
        values = struct.unpack_from("<12f", data, 84 + 50 * index)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"Non-finite STL data: {source}, face {index}")
        normal = values[:3]
        if sum(value * value for value in normal) == 0:
            raise ValueError(f"Missing STL normal: {source}, face {index}")
        normals.append(normal)
        face = []
        for offset in (3, 6, 9):
            vertex = values[offset:offset + 3]
            face.append(vertices.setdefault(vertex, len(vertices) + 1))
        faces.append(face)

    with destination.open("w", encoding="ascii", newline="\n") as output:
        output.write(f"# Generated from mujoco/assets/visual/{source.name}\n")
        output.write("# Original triangles, winding and face normals; no decimation.\n")
        output.write("# Regenerate with tools/generate_visual_meshes.py\n")
        # Nine significant digits round-trip every original STL float32 value.
        for vertex in vertices:
            output.write("v " + " ".join(f"{value:.9g}" for value in vertex) + "\n")
        for normal in normals:
            output.write("vn " + " ".join(f"{value:.9g}" for value in normal) + "\n")
        for index, face in enumerate(faces, 1):
            output.write("f " + " ".join(f"{vertex}//{index}" for vertex in face) + "\n")
    return count


def main():
    package = Path(__file__).resolve().parents[1]
    destination = package / "mujoco/assets/visual"
    destination.mkdir(parents=True, exist_ok=True)
    for name in ("link_base", *(f"link{i}" for i in range(1, 7))):
        count = convert(destination / f"{name}.stl", destination / f"{name}.obj")
        print(f"{name}: preserved {count} triangles and their original face normals")


if __name__ == "__main__":
    main()
