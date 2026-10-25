"""Isolated ROS preview: marker output, retained results and zero motion goals."""

import json
import threading
import time
import uuid

import pytest
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.action import MoveToPose
from rclpy.action import ActionServer
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from lara5_object_approach.preview import GraspPreview


def wait_for(predicate):
    deadline = time.monotonic()+5
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("Preview ROS test timed out")


@pytest.fixture
def harness():
    prefix = "/preview_test_"+uuid.uuid4().hex
    context = Context()
    context.init()
    peer = Node("preview_test_peer", context=context)
    remaps = []
    for suffix in ("target", "status", "markers", "flange_pose", "grasp_pose"):
        remaps.extend(["-r", f"/arm/approach/preview/{suffix}:={prefix}/{suffix}"])
    remaps.extend(["-r", "/safety/arm/move_to_pose:="+prefix+"/motion"])
    preview = GraspPreview(context=context, cli_args=["--ros-args", *remaps])
    received = dict(status=[], markers=[], flange_pose=[], grasp_pose=[], motion=[])
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []
    for suffix, msgtype in (("status", String), ("markers", MarkerArray),
                            ("flange_pose", PoseStamped), ("grasp_pose", PoseStamped)):
        subscriptions.append(peer.create_subscription(msgtype, prefix+"/"+suffix,
            lambda msg, key=suffix: received[key].append(msg), qos))
    publisher = peer.create_publisher(PoseStamped, prefix+"/target", 1)
    def execute(goal):
        received["motion"].append(goal.request)
        goal.abort()
        return MoveToPose.Result(success=False)
    # This unique remap would catch any accidentally added motion client.
    server = ActionServer(peer, MoveToPose, prefix+"/motion", execute_callback=execute)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer); executor.add_node(preview)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        wait_for(lambda: publisher.get_subscription_count() == 1)
        yield peer, preview, publisher, received, qos, prefix
    finally:
        executor.shutdown(timeout_sec=5); thread.join(timeout=5)
        server.destroy(); preview.destroy_node(); peer.destroy_node(); context.shutdown()


def send(publisher, frame="world"):
    message = PoseStamped()
    message.header.frame_id = frame
    message.pose.position.x, message.pose.position.y, message.pose.position.z = .25, -.2, .95
    message.pose.orientation.w = 1.
    publisher.publish(message)


def test_preview_emits_hand_object_frames_and_no_motion(harness):
    peer, preview, publisher, received, _, _ = harness
    send(publisher)
    wait_for(lambda: received["grasp_pose"] and received["flange_pose"] and received["markers"])
    markers = received["markers"][-1].markers
    assert markers[0].action == Marker.DELETEALL
    assert sum(m.ns == "open_hand" for m in markers) == len(preview.model.hand_links)
    assert any(m.type == Marker.CYLINDER for m in markers)
    assert sum(m.ns == "grasp_axes" for m in markers) == 3
    assert all(m.header.frame_id == "world" for m in markers[1:])
    pose = received["flange_pose"][-1].pose.position
    assert [pose.x, pose.y, pose.z] == pytest.approx([.275, -.35, 1.035])
    assert not received["motion"]
    # No ROS service/action clients at all; output cannot dispatch a motion goal.
    assert peer.get_client_names_and_types_by_node(preview.get_name(), preview.get_namespace()) == []


def test_late_subscriber_receives_preview(harness):
    peer, _, publisher, received, qos, prefix = harness
    send(publisher)
    wait_for(lambda: received["markers"])
    late = []
    sub = peer.create_subscription(MarkerArray, prefix+"/markers", late.append, qos)
    wait_for(lambda: late)
    assert any(m.ns == "open_hand" for m in late[-1].markers)
    peer.destroy_subscription(sub)


def test_invalid_pose_clears_overlay_and_reports_rejection(harness):
    _, _, publisher, received, _, _ = harness
    send(publisher)
    wait_for(lambda: received["flange_pose"])
    send(publisher, "camera_frame")
    wait_for(lambda: any(json.loads(s.data)["state"] == "rejected_invalid" for s in received["status"]))
    wait_for(lambda: len(received["markers"][-1].markers) == 1)
    assert received["markers"][-1].markers[0].action == Marker.DELETEALL
    assert len(received["flange_pose"]) == 1
    assert not received["motion"]
