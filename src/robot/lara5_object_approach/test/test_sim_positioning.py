"""Opt-in actual MuJoCo positioning, hand open and no grasp command.

Use an isolated container without hardware access, ROS_LOCALHOST_ONLY=1 and
LARA5_SIM_POSITIONING_TEST=1. Starts a fresh headless simulation and executes the
known cylinder approach. Normal package tests skip all simulated motion.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from lara5_object_approach.node import ObjectApproach


@pytest.mark.skipif(os.environ.get('LARA5_SIM_POSITIONING_TEST') != '1', reason='Opt-in MuJoCo motion only')
def test_simulated_open_hand_two_phase_positioning(tmp_path):
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '42', '43')
    log = open(tmp_path/'simulation.log', 'w+')
    backend = subprocess.Popen(['ros2', 'launch', 'lara5_controller_manager', 'controller_manager.launch.py',
        'deployment_mode:=simulation', 'sim_headless:=true', 'rviz:=false', 'hands:=on',
        'safety:=on', 'sim_camera_backend:=egl'], stdout=log, stderr=log, start_new_session=True)
    rclpy.init()
    node = ObjectApproach()
    peer = rclpy.create_node('positioning_integration_observer')
    executor = SingleThreadedExecutor(); executor.add_node(node); executor.add_node(peer)
    events = []
    started = time.monotonic()
    def status(message):
        event = json.loads(message.data)
        event['elapsed_s'] = time.monotonic()-started
        events.append(event)
        print(json.dumps(event), flush=True)
    subscription = peer.create_subscription(String, '/arm/approach/status', status,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher = peer.create_publisher(PoseStamped, '/arm/approach/target', 1)
    def until(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate(): return
        raise AssertionError('MuJoCo integration timeout:\n'+(tmp_path/'simulation.log').read_text()[-5000:])
    try:
        until(lambda: len(node.planner.samples)==12 and node.planner.motion.server_is_ready()
              and all(c.service_is_ready() for c in (node.planner.plan_client,
                  node.planner.check_client, node.planner.fk_client))
              and publisher.get_subscription_count()==1, 40)
        start = node.planner.measured().copy()
        pose = PoseStamped(); pose.header.frame_id = 'world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = .25, -.2, .95
        pose.pose.orientation.z, pose.pose.orientation.w = -.7071067811865476, .7071067811865476
        publisher.publish(pose)
        until(lambda: any(e['state'] in ('succeeded', 'failed', 'error', 'rejected_state', 'unavailable')
                          for e in events), 180)
        assert events[-1]['state'] == 'succeeded', events[-1]
        assert events[-1]['ready_for_closure']
        assert events[-1]['measured_flange_xyz'] == pytest.approx([.1, -.225, 1.035], abs=.003)
        assert max(abs(node.planner.measured()-start)) > .2
        assert events[-1]['grasp_error_m_rad'][0] <= .003
        states = [e['state'] for e in events]
        assert states.index('verifying_pregrasp') < states.index('moving_to_grasp') < states.index('succeeded')
        # measured() enforces the open-hand requirement at completion as well.
        node.planner.measured()
    finally:
        output = Path(os.environ.get('LARA5_SIM_POSITIONING_REPORT', str(tmp_path/'result.json')))
        output.write_text(json.dumps(events, indent=2))
        node.cancel_active()
        deadline = time.monotonic()+5
        while node.active_request is not None and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
        executor.shutdown(); node.destroy_node(); peer.destroy_node(); rclpy.shutdown()
        os.killpg(backend.pid, signal.SIGINT)
        try: backend.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(backend.pid, signal.SIGKILL); backend.wait(timeout=5)
        log.close()
