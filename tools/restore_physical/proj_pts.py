"""proj_pts.py X,Y,Z ...: pixel of world points in the fixed camera."""
import sys

import numpy as np

import scene as sc

s = sc.Scene()
for a in sys.argv[1:]:
    p = np.array([float(v) for v in a.split(",")])
    print(a, "->", np.round(s.project(p), 1).tolist())
