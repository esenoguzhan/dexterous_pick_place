"""EGL RGB-D and opt-in private-display viewer regression; no ROS or physics steps."""

import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import pytest

from lara5_mujoco_sim.camera import mask_depth


def scene():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <light pos="0 0 2"/>
      <geom type="plane" size="2 2 .1" rgba=".6 .6 .6 1"/>
      <body name="target" pos="0 0 .1"><geom type="box" size=".05 .05 .1" rgba="0 1 0 1"/></body>
      <camera name="external_camera" pos="0 -.5 .3" mode="targetbody" target="target"/>
      <camera name="wrist_camera" pos=".2 -.5 .3" mode="targetbody" target="target"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def render_camera(model, data, camera, frames):
    with mujoco.Renderer(model, width=160, height=120) as renderer:
        for _ in range(frames):
            renderer.update_scene(data, camera=camera)
            rgb = renderer.render()
            assert np.std(rgb) > 1, "Blank RGB image"
            renderer.enable_depth_rendering()
            depth = mask_depth(renderer.render(), {"depth_min_m": .2, "depth_max_m": 3.})
            renderer.disable_depth_rendering()
            assert np.count_nonzero(depth) > depth.size // 10
            assert np.isfinite(depth).all()


def test_two_offscreen_cameras_render_rgb_and_depth():
    model, data = scene()
    render_camera(model, data, "external_camera", 3)
    render_camera(model, data, "wrist_camera", 3)


@pytest.mark.skipif(os.environ.get("LARA5_CAMERA_VIEWER_TEST") != "1",
                    reason="Viewer test requires an explicitly enabled private X display")
def test_two_camera_threads_do_not_close_the_viewer():
    assert os.environ.get("MUJOCO_GL") == "egl", "Use the same EGL backend as simulation bringup"
    # Keep the GUI lifecycle in a fresh interpreter, as in normal simulator
    # startup. In-process headless tests may already own a different GL context.
    result = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--viewer-probe"],
                            capture_output=True, text=True, timeout=25)
    assert result.returncode == 0, result.stdout + result.stderr


def viewer_probe():
    model, data = scene()
    existing_threads = set(threading.enumerate())
    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    viewer_threads = set(threading.enumerate()) - existing_threads
    errors, finished = [], []

    def render(name):
        try:
            render_camera(model, data, name, 40)
            finished.append(name)
        except Exception as error:
            errors.append(error)

    threads = [threading.Thread(target=render, args=(name,), daemon=True)
               for name in ("external_camera", "wrist_camera")]
    try:
        for thread in threads:
            thread.start()
        deadline = time.monotonic() + 15
        while any(thread.is_alive() for thread in threads) and time.monotonic() < deadline:
            assert viewer.is_running(), "Viewer closed while camera rendering was active"
            viewer.sync()
            time.sleep(.02)
        for thread in threads:
            thread.join(timeout=1)
        assert not errors, errors
        assert len(finished) == 2
        assert viewer.is_running()
    finally:
        viewer.close()
        # The public close method requests shutdown asynchronously. Let the GUI
        # finish before Python tears down its graphics libraries.
        for thread in viewer_threads:
            thread.join(timeout=5)
        assert all(not thread.is_alive() for thread in viewer_threads), "Viewer shutdown timed out"


if __name__ == "__main__" and "--viewer-probe" in sys.argv:
    viewer_probe()
