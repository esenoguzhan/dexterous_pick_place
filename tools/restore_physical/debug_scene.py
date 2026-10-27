"""debug_scene.py [N]: raw pipeline candidates (before scene.py's filter) for N captures."""
import sys

import numpy as np

import scene as sc

s = sc.Scene()
for i in range(int(sys.argv[1]) if len(sys.argv) > 1 else 3):
    result, frame, snap = s.capture(f"debug_{i}")
    kept = {o["id"] for o in result["objects"]}
    print(f"capture {i}: table {result['table_height']:.4f}")
    for o in snap.objects:
        print("  ", o.id, "kept" if o.id in kept else "DROP", np.round(o.position, 3).tolist(),
              "size", np.round(o.size, 3).tolist(), "pts", len(o.points))
