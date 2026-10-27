"""measure_hand.py [--up M]: compare the real hand (fixed-camera depth) with the model hand frame.

Points near the model hand are expressed in right_hand_base_link; their extent along +z (fingers)
and the palm side show how far the real hand sits from where the model puts it.
"""
import argparse

import numpy as np

import jetson
import robot as rb
import scene as sc


def measure(r, s, label=""):
    """Real minus model hand position, in the hand frame [m], from the fixed camera's depth."""
    frame = jetson.capture(jetson.FIXED, frames=10)
    H = r.frame_pose("right_hand_base_link", "world")
    F = r.frame_pose("lara5_flange", "world")
    K = frame.intrinsics.matrix()
    v, u = np.nonzero(frame.depth > 0)
    z = frame.depth[v, u]
    p = np.c_[(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z]
    Tc = s.cam.world_pose()
    pw = p @ Tc[:3, :3].T + Tc[:3, 3]
    ph = (pw - H[:3, 3]) @ H[:3, :3]          # hand frame
    box = (ph[:, 0] > -0.16) & (ph[:, 0] < 0.12) & (ph[:, 1] > -0.06) & (ph[:, 1] < 0.06) & (ph[:, 2] > -0.10) & (ph[:, 2] < 0.30)
    h = ph[box]
    print("hand base world", np.round(H[:3, 3], 3).tolist(), "fingers dir", np.round(H[:3, 2], 3).tolist(), "points", len(h))
    if len(h):
        for q in (50, 90, 98, 99.5):
            print(f"z percentile {q}: {np.percentile(h[:, 2], q):.3f}")
        far = h[h[:, 2] > np.percentile(h[:, 2], 97)]
        print("far end (tips) x,y mean", np.round(far[:, :2].mean(axis=0), 3).tolist())
        np.save(f"/run_data/hand_points{label}.npy", h)
        return far[:, :2].mean(axis=0), np.percentile(h[:, 2], 99.5)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--up", type=float, default=0.0)
    a = ap.parse_args()
    r = rb.start("restore_measure_hand")
    if a.up:
        T = r.frame_pose("lara5_flange", "world")
        T[2, 3] += a.up
        r.move_pose(T, duration=4.0)
    measure(r, sc.Scene())
    r.close()
