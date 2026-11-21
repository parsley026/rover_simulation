"""
Generate the structure that is shaped like a safety indicator

Output: meshes/indicators/safety_indicator/rounded_triangle_prism.stl

The prism is normalised so the xacro macro can size it with the mesh
`scale` attribute (one mesh serves the clear shell, both caps and the PCB):
  - width (extent along y) = 1.0
  - height (z)             = 1.0, from z = 0 to z = 1
  - the vertical axis through the origin is the triangle's centroid, which
    is where the mounting pole runs through
  - one flat face points along +x (the LED side), a rounded corner along -x

Run: python3 generate_safety_indicator_mesh.py [--corner-radius 0.2] [--segments 16]
"""

import argparse
import math
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(
    HERE, '..', 'meshes', 'indicators', 'safety_indicator', 'rounded_triangle_prism.stl')


def outline(corner_radius: float, segments: int):
    """
    Counter-clockwise outline of an equilateral triangle with rounded corners.

    Corners sit at 60, 180 and 300 degrees, so the flat face between the
    60 and 300 degree corners faces +x. Corner circle centres are placed at
    distance rc_dist from the centroid such that the total width along y is 1:
        rc_dist * sin(60) + corner_radius = 0.5
    """
    rc_dist = (0.5 - corner_radius) / math.sin(math.radians(60))
    pts = []
    for corner_deg in (60.0, 180.0, 300.0):
        a = math.radians(corner_deg)
        cx, cy = rc_dist * math.cos(a), rc_dist * math.sin(a)
        for i in range(segments + 1):
            t = a - math.radians(60) + math.radians(120) * i / segments
            pts.append((cx + corner_radius * math.cos(t), cy + corner_radius * math.sin(t)))
    return pts


def triangles(pts):
    n = len(pts)
    tris = []
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        a, b, c, d = (x0, y0, 0.0), (x1, y1, 0.0), (x1, y1, 1.0), (x0, y0, 1.0)
        tris.append((a, b, c))
        tris.append((a, c, d))
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        tris.append(((0.0, 0.0, 1.0), (x0, y0, 1.0), (x1, y1, 1.0)))  # top, normal +z
        tris.append(((0.0, 0.0, 0.0), (x1, y1, 0.0), (x0, y0, 0.0)))  # bottom, normal -z
    return tris


def normal(a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / length, ny / length, nz / length


def write_binary_stl(path, tris):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(b'rover_description safety indicator rounded triangle prism'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            f.write(struct.pack('<3f', *normal(a, b, c)))
            for v in (a, b, c):
                f.write(struct.pack('<3f', *v))
            f.write(struct.pack('<H', 0))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--corner-radius', type=float, default=0.2,
                   help='corner radius as a fraction of the width (default 0.2)')
    p.add_argument('--segments', type=int, default=16, help='segments per rounded corner')
    p.add_argument('--out', default=DEFAULT_OUT)
    args = p.parse_args()
    if not 0.0 < args.corner_radius < 0.5:
        p.error('--corner-radius must be between 0 and 0.5')

    pts = outline(args.corner_radius, args.segments)
    tris = triangles(pts)
    write_binary_stl(args.out, tris)

    xs = [x for x, _ in pts]
    ys = [y for _, y in pts]
    print(f'wrote {os.path.normpath(args.out)}: {len(tris)} triangles')
    print(f'  x [{min(xs):+.4f}, {max(xs):+.4f}]  (flat LED face at +x)')
    print(f'  y [{min(ys):+.4f}, {max(ys):+.4f}]  z [0, 1]')


if __name__ == '__main__':
    main()
