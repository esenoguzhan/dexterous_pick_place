"""Rendering must never block the physics producer or supply feedback timestamps."""
import os
import time

import mujoco
import pytest

from lara5_mujoco_sim.viewer_process import ViewerProcess


def blocked_display(model, state, state_lock, stop, rate):
    # Simulate the display being descheduled while owning its IPC-buffer lock.
    with state_lock:
        state[0] = -123.
        stop.wait(10.)


def unresponsive_display(model, state, state_lock, stop, rate):
    time.sleep(60.)


def model_data():
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
        <body><joint name="joint"/><geom type="sphere" size=".1"/></body>
        <camera name="camera" pos="0 -1 1"/>
      </worldbody></mujoco>''')
    return model,mujoco.MjData(model)


def test_blocked_viewer_cannot_block_physics_or_snapshot_updates():
    model,data=model_data()
    viewer=ViewerProcess(model,data,worker=blocked_display)
    try:
        viewer.start()
        end=time.monotonic()+8
        while viewer.state[0] != -123. and time.monotonic()<end:
            time.sleep(.01)
        assert viewer.state[0] == -123.
        begin=time.monotonic()
        for _ in range(500):
            mujoco.mj_step(model,data)
            assert viewer.update(data) is False
        assert time.monotonic()-begin < .3
        assert data.time > .9  # Physics progressed while rendering could not.
    finally:
        viewer.close()
    assert not viewer.process.is_alive()


def test_unresponsive_renderer_has_bounded_shutdown():
    model,data=model_data()
    viewer=ViewerProcess(model,data,worker=unresponsive_display)
    viewer.start()
    begin=time.monotonic()
    viewer.close(timeout=.1)
    assert time.monotonic()-begin < 3
    assert not viewer.process.is_alive()


@pytest.mark.skipif(os.environ.get('LARA5_CAMERA_VIEWER_TEST')!='1',reason='Private display opt-in')
def test_real_viewer_process_runs_and_closes_cleanly():
    assert os.environ.get('DISPLAY')
    model,data=model_data()
    viewer=ViewerProcess(model,data)
    try:
        viewer.start()
        begin=time.monotonic()
        while time.monotonic()-begin < 3:
            assert viewer.is_running()
            mujoco.mj_step(model,data)
            viewer.update(data)
            time.sleep(.01)
    finally:
        viewer.close()
    assert viewer.process.exitcode == 0
