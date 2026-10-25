"""Opt-in real MuJoCo approach, independent grasp, then verified 10 cm lift."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from lara5_safety_msgs.msg import SimGraspFeedback
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String

from lara5_grasp.node import HandGrasp
from lara5_object_approach.node import ObjectApproach
from lara5_object_lift.node import ObjectLift


@pytest.mark.skipif(os.environ.get('LARA5_SIM_LIFT_TEST')!='1',reason='Opt-in real simulated lift')
def test_physical_object_rises_and_remains_gripped(tmp_path):
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','42','43','44','45')
    visible=os.environ.get('LARA5_SIM_LIFT_VIEWER')=='1'
    log=open(tmp_path/'backend.log','w+')
    backend=subprocess.Popen(['ros2','launch','lara5_controller_manager','controller_manager.launch.py',
        'deployment_mode:=simulation','hands:=on','safety:=on','allow_scene_updates:=true','allow_grasp_updates:=true',
        'sim_headless:='+('false' if visible else 'true'),'rviz:='+('true' if visible else 'false'),
        'sim_camera_backend:=egl'],stdout=log,stderr=log,start_new_session=True)
    rclpy.init()
    approach=ObjectApproach();grasp=HandGrasp();lift=ObjectLift();peer=rclpy.create_node('lift_review_observer')
    executor=SingleThreadedExecutor()
    for node in (approach,grasp,lift,peer):executor.add_node(node)
    events={task:[] for task in ('approach','grasp','lift')};contacts=[];images={};started=time.monotonic()
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[]
    def status(task,msg):
        event=json.loads(msg.data);event['elapsed_s']=time.monotonic()-started
        events[task].append(event);print(task,json.dumps(event),flush=True)
    for task in events:
        subscriptions.append(peer.create_subscription(String,f'/arm/{task}/status',
            lambda msg,task=task:status(task,msg),qos))
    def feedback(msg):
        if msg.object_name=='green_cylinder':
            p=msg.object_pose.position;r=msg.object_in_flange.position
            contacts.append(dict(elapsed_s=time.monotonic()-started,object_xyz=[p.x,p.y,p.z],
                object_in_flange_xyz=[r.x,r.y,r.z],supported=msg.supported,opposing_contacts=msg.opposing_contacts,
                contact_links=list(msg.contact_links),normal_forces_n=list(msg.contact_normal_forces)))
    subscriptions.append(peer.create_subscription(SimGraspFeedback,'/sim/grasp_feedback',feedback,10))
    for camera in ('external_camera','wrist_camera'):
        subscriptions.append(peer.create_subscription(Image,f'/sim/{camera}/color/image_raw',
            lambda msg,camera=camera:images.update({camera:msg}),10))
    pose_pub=peer.create_publisher(PoseStamped,'/arm/approach/target',1)
    grasp_pub=peer.create_publisher(Empty,'/arm/grasp/target',1)
    lift_pub=peer.create_publisher(Empty,'/arm/lift/target',1)
    def until(predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate():return
        raise AssertionError('Simulated lift timeout:\n'+(tmp_path/'backend.log').read_text()[-6000:])
    def terminal(task):return any(e['state'] in ('succeeded','failed','error','rejected_state','unavailable') for e in events[task])
    try:
        until(lambda:len(lift.samples)==12 and lift.contact and lift.scene.service_is_ready()
            and approach.planner.motion.server_is_ready() and grasp.hand.server_is_ready()
            and all(c.service_is_ready() for c in (approach.planner.fk_client,approach.planner.plan_client,approach.planner.check_client))
            and len(images)==2,40)
        pose=PoseStamped();pose.header.frame_id='world'
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=.25,-.2,.95
        pose.pose.orientation.z,pose.pose.orientation.w=-.7071067811865476,.7071067811865476
        pose_pub.publish(pose);until(lambda:terminal('approach'),180)
        assert events['approach'][-1]['state']=='succeeded',events['approach'][-1]
        settled=time.monotonic()+.5;until(lambda:time.monotonic()>settled,2)
        grasp_pub.publish(Empty());until(lambda:terminal('grasp'),80)
        assert events['grasp'][-1]['state']=='succeeded',events['grasp'][-1]
        until(lambda:lift.grasp is not None,3)
        lift_pub.publish(Empty());until(lambda:terminal('lift'),65)
        outcome=events['lift'][-1]
        assert outcome['state']=='succeeded',outcome
        assert outcome['retention_verified'] and outcome['object_rise_m']>=.088
        assert outcome['supported'] is False and outcome['opposing_contacts'] is True
        assert outcome['relative_slip_m']<=.012
        settled=time.monotonic()+.3;until(lambda:time.monotonic()>settled,1)
    finally:
        output=Path(os.environ.get('LARA5_SIM_LIFT_REPORT_DIR',str(tmp_path)))
        output.mkdir(parents=True,exist_ok=True)
        (output/'lift_review.json').write_text(json.dumps(dict(events=events,physics_contacts=contacts,
            visible_viewer=visible,note='Free physics object; only collision model uses a flange-fixed payload box'),indent=2)+'\n')
        if images:
            from PIL import Image as PILImage
            for camera,msg in images.items():
                if msg.encoding=='rgb8':PILImage.frombytes('RGB',(msg.width,msg.height),bytes(msg.data),'raw','RGB',msg.step).save(output/(camera+'.png'))
        approach.cancel_active();grasp.cancel_active();lift.cancel_active()
        deadline=time.monotonic()+6
        while (approach.active_request or grasp.active_request or lift.active_request) and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
        executor.shutdown()
        for node in (approach,grasp,lift,peer):node.destroy_node()
        rclpy.shutdown();os.killpg(backend.pid,signal.SIGINT)
        try:backend.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(backend.pid,signal.SIGKILL);backend.wait(timeout=5)
        log.close()
