"""thumb_view.py: wrist-camera images with the thumb yaw at 0 and at 1.16 rad (hand in free air)."""
import cv2

import jetson
import robot as rb

r = rb.start("restore_thumb_view")
for yaw in (0.0, 1.16):
    h = r.hand([yaw, 0, 0, 0, 0, 0], 1.5)
    f = jetson.capture(jetson.WRIST, frames=1)
    cv2.imwrite(f"/run_data/thumb_yaw_{yaw:.2f}.jpg", f.color[..., ::-1])
    print("yaw", yaw, "measured", [round(v, 3) for v in h])
r.hand([0, 0, 0, 0, 0, 0], 1.5)
r.close()
