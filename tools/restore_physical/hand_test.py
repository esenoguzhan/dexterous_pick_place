"""hand_test.py: close the four fingers to 0.6 rad and open them again; thumb unchanged."""
import numpy as np

import robot as rb

r = rb.start("restore_hand_test")
h0 = r.hand_joints()
print("start", np.round(h0, 3).tolist())
target = h0.copy()
target[1:5] = 0.6
print("half closed", np.round(r.hand(target, 1.5), 3).tolist())
target[1:5] = 0.0
print("open", np.round(r.hand(target, 1.5), 3).tolist())
r.close()
