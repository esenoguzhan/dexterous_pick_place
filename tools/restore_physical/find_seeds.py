"""find_seeds.py [TRIES]: random-seed IK for palm-down hover poses over the sheet; fills config/seeds.json."""
import math
import sys
import time

import numpy as np

import grasp as gr
import ik
import motion as mo
import robot as rb

tries = int(sys.argv[1]) if len(sys.argv) > 1 else 15
rng = np.random.default_rng(1)
r = rb.start("restore_find_seeds")
T_flange_hand = r.frame_pose("right_hand_base_link", "lara5_flange")
targets = [(0.15, 0.0), (0.35, 0.2), (0.35, -0.2), (0.1, 0.25), (0.1, -0.25)]
t0 = time.monotonic()
found = 0
for x, y in targets:
    top = np.array([x, y, 1.0])
    for offset in mo.HEADINGS_DEG[:4]:
        yaw = gr.radial_yaw(top) + math.radians(offset)
        T_flange = gr.flange_pose(gr.hand_pose(top, yaw, 0.15), T_flange_hand)
        seeds = mo.load_seeds()[:4] + [np.r_[rng.uniform(-math.pi, math.pi), rng.uniform(-1.5, 1.5), rng.uniform(-2.5, 2.5),
                                             rng.uniform(-math.pi, math.pi), rng.uniform(-2.5, 2.5), rng.uniform(-math.pi, math.pi)]
                                        for _ in range(tries)]
        for seed in seeds:
            q, ep, er = ik.solve(r, T_flange, seed, iters=60)
            if ep < 2e-3 and er < 0.01:
                mo.save_seed(q)
                found += 1
                print(f"({x},{y}) heading {offset:+.0f}: {np.round(q, 3).tolist()}  [{time.monotonic() - t0:.0f} s]", flush=True)
                break
        else:
            print(f"({x},{y}) heading {offset:+.0f}: none", flush=True)
print("found", found, "seeds total", len(mo.load_seeds()))
r.close()
