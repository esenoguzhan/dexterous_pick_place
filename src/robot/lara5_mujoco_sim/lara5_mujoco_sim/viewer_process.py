"""A latest-state viewer process that cannot block physics or feedback.

Only a compact integration-state snapshot crosses the process boundary. Rendering owns
its own model/data and GLFW/GIL; no viewer lock is taken by the physics thread.
The viewer is for observation. Mouse orbit/zoom are local to the display.
"""

import multiprocessing as mp
import signal
import threading
import time

import mujoco
import numpy as np

STATE = mujoco.mjtState.mjSTATE_INTEGRATION


def display(model, state, state_lock, stop, rate):
    import mujoco.viewer
    # The simulator owns Ctrl+C and closes this child through stop/close().
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    data = mujoco.MjData(model)
    with state_lock:
        initial = np.frombuffer(state).copy()
    mujoco.mj_setState(model, data, initial, STATE)
    mujoco.mj_forward(model, data)
    previous_threads = set(threading.enumerate())
    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    gui_threads = set(threading.enumerate())-previous_threads
    try:
        while not stop.is_set() and viewer.is_running():
            begin = time.monotonic()
            with state_lock:
                sample = np.frombuffer(state).copy()
            with viewer.lock():
                mujoco.mj_setState(model, data, sample, STATE)
                mujoco.mj_forward(model, data)
            viewer.sync()
            stop.wait(max(0., 1./rate-(time.monotonic()-begin)))
    finally:
        viewer.close()
        for thread in gui_threads:
            thread.join(timeout=2.)


class ViewerProcess:
    def __init__(self, model, data, rate=30., worker=display):
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError('viewer_rate must be finite and positive')
        # Forking an initialized ROS/EGL process is unsafe. Spawn creates fresh
        # interpreter/graphics state and pickles a separate MuJoCo model.
        self.context = mp.get_context('spawn')
        self.model = model
        self.state = self.context.RawArray('d', mujoco.mj_stateSize(model, STATE))
        self.state_lock = self.context.Lock()
        self.stop = self.context.Event()
        self.process = self.context.Process(target=worker,
            args=(model, self.state, self.state_lock, self.stop, rate),
            name='mujoco_viewer', daemon=True)
        self.update(data)

    def start(self):
        self.process.start()

    def update(self, data):
        # A paused or overloaded viewer may even be descheduled while holding
        # its shared-buffer lock. Drop that display update instead of waiting.
        if not self.state_lock.acquire(block=False):
            return False
        try:
            sample = np.frombuffer(self.state)
            mujoco.mj_getState(self.model, data, sample, STATE)
            return True
        finally:
            self.state_lock.release()

    def is_running(self):
        return self.process.is_alive()

    def close(self, timeout=3.):
        self.stop.set()
        if self.process.pid is None:
            return
        self.process.join(timeout=timeout)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=2.)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=2.)
