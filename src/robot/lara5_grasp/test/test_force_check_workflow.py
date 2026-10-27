"""Real ROS topics into the read-only force check; synthetic hand, no hardware."""

import copy
import json
import threading
import time
import uuid

import pytest
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

from lara5_grasp.configuration import HAND_JOINTS
from lara5_grasp.force_check_node import HandForceCheck, PREFIX


def wait_for(predicate, timeout=8.):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate(): return
        time.sleep(.01)
    raise AssertionError('Force check workflow timed out')


class Harness:
    def __init__(self, **parameters):
        prefix = '/force_check_test_'+uuid.uuid4().hex
        self.context = Context(); self.context.init()
        self.peer = Node('fake_hand_driver', context=self.context)
        params = dict(measured_topic=prefix+'/measured', close_positions=[.2]*6,
            max_joint_step=.02, max_joint_speed=.10, max_joint_acceleration=.25, min_segment_duration=.4,
            hardware_force_baseline=[50.]*6, hardware_force_contact_thresholds=[25.]*6,
            hardware_force_limits=[200.]*6, open_capture_seconds=.5, min_capture_seconds=.3,
            min_samples=20, feedback_max_age=.3)
        params.update(parameters)
        remaps = []
        for name in ('record_unloaded_open', 'record_unloaded_motion', 'record_loaded', 'stop', 'reload',
                     'status', 'report'):
            remaps += ['-r', f'{PREFIX}/{name}:={prefix}/{name}']
        self.node = HandForceCheck(context=self.context, cli_args=['--ros-args', *remaps],
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        self.positions, self.forces = [0.]*6, [50.]*6
        self.backwards = False; self.reports = []
        self.pub = self.peer.create_publisher(JointState, prefix+'/measured', qos_profile_sensor_data)
        self.triggers = {name: self.peer.create_publisher(Empty, f'{prefix}/{name}', 1)
                         for name in ('record_unloaded_open', 'record_unloaded_motion', 'record_loaded',
                                      'stop', 'reload')}
        self.peer.create_subscription(String, prefix+'/report', lambda m: self.reports.append(json.loads(m.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.peer.create_timer(.01, self.tick)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.peer); self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin); self.thread.start()
        try:
            wait_for(lambda: all(p.get_subscription_count() == 1 for p in self.triggers.values())
                     and self.node.last_source is not None and self.reports)
        except BaseException:
            self.close(); raise

    def tick(self):
        # Reversed names model driver register order; each read is published twice.
        msg = JointState(name=list(reversed(HAND_JOINTS)), position=list(reversed(self.positions)),
                         effort=list(reversed(self.forces)))
        msg.header.stamp = self.peer.get_clock().now().to_msg()
        if self.backwards:
            # Fresh enough to pass the age check, but before the last reading.
            msg.header.stamp = (self.peer.get_clock().now()-Duration(seconds=.05)).to_msg()
            self.backwards = False
        self.pub.publish(msg); self.pub.publish(copy.deepcopy(msg))

    def record(self, name, done):
        count = len(self.reports)
        self.triggers[name].publish(Empty())
        wait_for(lambda: any(report['active_capture'] for report in self.reports[count:]))
        done()
        wait_for(lambda: self.reports[-1]['active_capture'] is None)
        return self.reports[-1]

    def close(self):
        self.executor.shutdown(timeout_sec=3); self.thread.join(timeout=3)
        self.node.destroy_node(); self.peer.destroy_node(); self.context.shutdown()


@pytest.fixture
def h():
    harness = Harness()
    try: yield harness
    finally: harness.close()


def move(h, seconds=.6):
    start = time.monotonic()
    while (t := time.monotonic()-start) < seconds:
        phase = min(t, seconds-t)/(seconds/2)
        h.positions = [.2*phase]*6
        h.forces = [50.+10.*phase]*6
        time.sleep(.005)
    h.positions, h.forces = [0.]*6, [50.]*6


def press(h, channel, load=60.):
    h.positions = [.02]*6
    h.forces = [50.]*6; h.forces[HAND_JOINTS.index(channel)] += load
    time.sleep(.5)
    h.forces = [50.]*6


def test_recordings_use_new_measurements_and_assess_the_candidate(h):
    report = h.record('record_unloaded_open', lambda: None)
    opened = report['recordings']['unloaded_open']
    assert opened['quality_criteria_met'], opened['reasons']
    # Every read was published twice; only distinct source times count.
    assert opened['cached_repeats_ignored'] >= opened['measurements']-1
    report = h.record('record_unloaded_motion', lambda: (move(h), h.triggers['stop'].publish(Empty())))
    assert report['recordings']['unloaded_motion']['quality_criteria_met'], report['recordings']
    assert not report['assessment']['configuration_consistent_with_measurements']
    for channel in HAND_JOINTS[1:]:
        h.node.set_parameters([Parameter('loaded_channel', value=channel),
                               Parameter('loaded_reference_n', value=2.)])
        report = h.record('record_loaded', lambda: (press(h, channel), h.triggers['stop'].publish(Empty())))
        assert report['recordings']['loaded/'+channel]['quality_criteria_met']
    assessment = report['assessment']
    assert assessment['configuration_consistent_with_measurements'], assessment['reasons']
    assert assessment['channels'][HAND_JOINTS[1]]['raw_per_newton_approx'] == pytest.approx(30., abs=1.)
    assert not assessment['force_limits_validated'] and report['hand_commands_sent'] is False


def test_backwards_source_time_fails_the_active_recording(h):
    h.triggers['record_unloaded_motion'].publish(Empty())
    wait_for(lambda: h.node.active is not None and len(h.node.active['samples']) > 3)
    h.backwards = True
    wait_for(lambda: 'unloaded_motion' in h.reports[-1]['recordings'])
    failed = h.reports[-1]['recordings']['unloaded_motion']
    assert not failed['quality_criteria_met'] and any('backwards' in r for r in failed['reasons'])


def test_loaded_settings_are_validated_and_frozen_while_recording(h):
    assert not h.node.set_parameters([Parameter('loaded_channel', value='right_thumb')])[0].successful
    assert not h.node.set_parameters([Parameter('min_samples', value=3)])[0].successful
    count = len(h.reports)
    h.triggers['record_loaded'].publish(Empty())  # rejected: no channel chosen
    time.sleep(.3)
    assert h.node.active is None and len(h.reports) == count
    h.node.set_parameters([Parameter('loaded_channel', value=HAND_JOINTS[2])])
    h.triggers['record_loaded'].publish(Empty())
    wait_for(lambda: h.node.active is not None)
    assert not h.node.set_parameters([Parameter('loaded_channel', value=HAND_JOINTS[3])])[0].successful
    h.triggers['stop'].publish(Empty())
    wait_for(lambda: 'loaded/'+HAND_JOINTS[2] in h.reports[-1]['recordings'])


def test_force_check_has_no_command_interfaces(h):
    name, namespace = h.node.get_name(), h.node.get_namespace()
    publishers = {topic for topic, _ in h.peer.get_publisher_names_and_types_by_node(name, namespace)}
    assert {topic.rsplit('/', 1)[-1] for topic in publishers} <= {'status', 'report', 'rosout', 'parameter_events'}
    assert not h.peer.get_client_names_and_types_by_node(name, namespace)


def test_reload_reevaluates_stored_recordings_against_the_edited_grasp_yaml(tmp_path):
    path = tmp_path/'hardware.yaml'
    def write(thresholds, text=None):
        path.write_text(text or json.dumps({'hand_grasp': {'ros__parameters': dict(
            hardware_force_baseline=[50.]*6, hardware_force_contact_thresholds=[thresholds]*6,
            hardware_force_limits=[200.]*6, close_positions=[.2]*6)}}))
    write(15.)
    h = Harness(grasp_config_file=str(path))
    try:
        h.record('record_unloaded_open', lambda: None)
        report = h.record('record_unloaded_motion', lambda: (move(h), h.triggers['stop'].publish(Empty())))
        assert report['candidate']['hardware_force_contact_thresholds'] == [15.]*6
        assert any(r.startswith('threshold_within_unloaded_margin') for r in report['assessment']['reasons'])
        write(25.)
        count = len(h.reports)
        h.triggers['reload'].publish(Empty())
        wait_for(lambda: len(h.reports) > count)
        report = h.reports[-1]
        assert report['candidate']['hardware_force_contact_thresholds'] == [25.]*6
        assert not any(r.startswith('threshold_within') for r in report['assessment']['reasons'])
        assert report['recordings']['unloaded_motion']['quality_criteria_met']
        write(0., text='hand_grasp: [unterminated')
        h.triggers['reload'].publish(Empty())
        wait_for(lambda: h.reports[-1]['reload_error'])
        assert h.reports[-1]['candidate']['hardware_force_contact_thresholds'] == [25.]*6
    finally:
        h.close()
