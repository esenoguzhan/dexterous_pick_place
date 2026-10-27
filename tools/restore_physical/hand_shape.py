"""The open hand's underside from its URDF collision meshes, in right_hand_base_link.

hand_shape.py [OUT.npz]: every hand mesh vertex (at the measured joint angles, from TF) is binned
on an x/z grid of the hand frame; each cell keeps the largest y, i.e. how far the hand reaches out
of the palm side there. An object top under the palm must stay below that everywhere.
"""

import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import robot as rb

GRID = 0.005
X_RANGE, Z_RANGE = (-0.16, 0.10), (-0.06, 0.30)


def load_stl(path):
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
        return np.array(v, float)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(float)


def package_path(uri):
    pkg, rel = uri[len("package://"):].split("/", 1)
    return Path("/ws/install") / pkg / "share" / pkg / rel


def main(out):
    r = rb.start("restore_hand_shape")
    import subprocess
    xml = subprocess.run(["ros2", "param", "get", "/robot_state_publisher", "robot_description", "--hide-type"],
                         capture_output=True, text=True, timeout=30).stdout
    root = ET.fromstring(xml[xml.index("<"):])
    pts = []
    for link in root.findall("link"):
        name = link.get("name")
        if not name.startswith("right_") or "camera" in name or "holder" in name:
            continue
        for col in link.findall("collision"):
            mesh = col.find("geometry/mesh")
            if mesh is None:
                continue
            o = col.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            scale = [float(v) for v in mesh.get("scale", "1 1 1").split()]
            v = load_stl(package_path(mesh.get("filename"))) * scale
            v = v @ Rotation.from_euler("xyz", rpy).as_matrix().T + xyz
            T = r.tf(name, "right_hand_base_link")
            pts.append(v @ T[:3, :3].T + T[:3, 3])
    r.close()
    p = np.concatenate(pts)
    nx = int(round((X_RANGE[1] - X_RANGE[0]) / GRID))
    nz = int(round((Z_RANGE[1] - Z_RANGE[0]) / GRID))
    depth = np.full((nx, nz), -np.inf)
    ix = ((p[:, 0] - X_RANGE[0]) / GRID).astype(int)
    iz = ((p[:, 2] - Z_RANGE[0]) / GRID).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iz >= 0) & (iz < nz)
    np.maximum.at(depth, (ix[ok], iz[ok]), p[ok, 1])
    np.savez(out, depth=depth, x0=X_RANGE[0], z0=Z_RANGE[0], grid=GRID)
    print("vertices", len(p), "max y", round(float(p[:, 1].max()), 3))
    for z0 in np.arange(0.0, 0.26, 0.02):
        row = depth[:, int((z0 - Z_RANGE[0]) / GRID)]
        xs = np.arange(nx) * GRID + X_RANGE[0]
        occupied = row > -np.inf
        if occupied.any():
            print(f"z={z0:.2f}: deepest y {row[occupied].max():.3f} at x {xs[occupied][np.argmax(row[occupied])]:.3f}; "
                  f"x span {xs[occupied].min():.3f}..{xs[occupied].max():.3f}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/run_data/hand_shape.npz")
