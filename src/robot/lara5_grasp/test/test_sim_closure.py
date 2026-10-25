"""Opt-in real MuJoCo positioning then explicit, independent hand closure.

No hardware connections; private container/domain and ROS_LOCALHOST_ONLY=1.
LARA5_SIM_CLOSURE_TEST=1 enables motion; LARA5_SIM_CLOSURE_EMPTY=1 tests an
empty hand at reset instead. No attachments, lifts, scene edits or camera control.
"""
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

from lara5_grasp.node import ARM_JOINTS, HandGrasp, distance
from lara5_object_approach.node import ObjectApproach


@pytest.mark.skipif(os.environ.get('LARA5_SIM_CLOSURE_TEST')!='1',reason='Opt-in simulated closure only')
def test_simulated_independent_closure(tmp_path):
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','42','43','44')
    empty=os.environ.get('LARA5_SIM_CLOSURE_EMPTY')=='1'
    visible=os.environ.get('LARA5_SIM_CLOSURE_VIEWER')=='1'
    if visible:
        assert os.environ.get('DISPLAY'), 'Visible test requires a private display'
    log=open(tmp_path/'simulation.log','w+')
    backend=subprocess.Popen(['ros2','launch','lara5_controller_manager','controller_manager.launch.py',
        'deployment_mode:=simulation','sim_headless:='+('false' if visible else 'true'),
        'rviz:='+('true' if visible else 'false'),'hands:=on','safety:=on',
        'sim_camera_backend:=egl'],stdout=log,stderr=log,start_new_session=True)
    rclpy.init()
    approach=ObjectApproach();grasp=HandGrasp();peer=rclpy.create_node('closure_review_observer')
    executor=SingleThreadedExecutor()
    for node in (approach,grasp,peer):executor.add_node(node)
    events={'approach':[],'grasp':[]}
    contacts=[];images={};started=time.monotonic()
    def status(task,msg):
        event=json.loads(msg.data);event['elapsed_s']=time.monotonic()-started
        events[task].append(event)
        print(task,json.dumps(event),flush=True)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs=[peer.create_subscription(String,'/arm/approach/status',lambda msg:status('approach',msg),qos),
          peer.create_subscription(String,'/arm/grasp/status',lambda msg:status('grasp',msg),qos)]
    def contact(msg):
        if msg.object_name=='green_cylinder':
            p=msg.object_pose.position
            contacts.append(dict(elapsed_s=time.monotonic()-started,links=list(msg.contact_links),
                normal_forces_n=list(msg.contact_normal_forces),object_xyz=[p.x,p.y,p.z],
                measurement_age_s=peer.get_clock().now().nanoseconds/1e9-
                (msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)))
    subs.append(peer.create_subscription(SimGraspFeedback,'/sim/grasp_feedback',contact,10))
    for name in ('external_camera','wrist_camera'):
        subs.append(peer.create_subscription(Image,'/sim/'+name+'/color/image_raw',
            lambda msg,name=name:images.update({name:msg}),10))
    pose_pub=peer.create_publisher(PoseStamped,'/arm/approach/target',1)
    closure_pub=peer.create_publisher(Empty,'/arm/grasp/target',1)
    def until(predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate():return
        raise AssertionError('Closure integration timeout:\n'+(tmp_path/'simulation.log').read_text()[-5000:])
    def terminal(task):
        return any(e['state'] in ('succeeded','failed','error','rejected_state','unavailable') for e in events[task])
    arm_start=None
    try:
        until(lambda:len(grasp.samples)==12 and grasp.contact is not None and grasp.hand.server_is_ready()
            and approach.planner.motion.server_is_ready() and all(c.service_is_ready() for c in
            (approach.planner.plan_client,approach.planner.check_client,approach.planner.fk_client))
            and pose_pub.get_subscription_count() and closure_pub.get_subscription_count(),40)
        if not empty:
            pose=PoseStamped();pose.header.frame_id='world'
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=.25,-.2,.95
            pose.pose.orientation.z,pose.pose.orientation.w=-.7071067811865476,.7071067811865476
            pose_pub.publish(pose);until(lambda:terminal('approach'),180)
            assert events['approach'][-1]['state']=='succeeded',events['approach'][-1]
        ready=time.monotonic()+.5
        until(lambda:time.monotonic()>ready,2)
        def fresh_ready():
            try:
                grasp.measured();grasp.measured(ARM_JOINTS);grasp.feedback()
                return len(images)==2 and grasp.history and time.monotonic()-grasp.history[0][0]>.20
            except ValueError:
                return False
        # Both renderers must finish startup before this test judges an empty
        # hand. The production task still rejects stale feedback immediately.
        until(fresh_ready,30)
        arm_start=grasp.measured(ARM_JOINTS)
        closure_pub.publish(Empty());until(lambda:terminal('grasp'),80)
        done=events['grasp'][-1]
        assert done['retention_verified'] is False
        if empty:
            assert done['state']=='failed',done
            assert done.get('contact_evidence') is False
            assert ('without current target finger contact' in done['message']
                    or 'Incomplete measured closure' in done['message']),done
        else:
            assert done['state']=='succeeded',done
            assert done['contact_evidence'] is True
            assert done['closure_outcome']=='contact_limited'
        assert distance(arm_start,grasp.measured(ARM_JOINTS))<.01
        ready=time.monotonic()+.3
        until(lambda:time.monotonic()>ready,1)
        if visible:
            first=next(e for e in events['grasp'] if e['state']=='opposing_thumb')['elapsed_s']
            samples=[sample for sample in contacts if sample['elapsed_s']>=first]
            assert len(samples)>2
            assert max(b['elapsed_s']-a['elapsed_s'] for a,b in zip(samples,samples[1:]))<grasp.feedback_max_age
            assert max(sample['measurement_age_s'] for sample in samples)<grasp.feedback_max_age
    finally:
        report_dir=Path(os.environ.get('LARA5_SIM_CLOSURE_REPORT_DIR',str(tmp_path)))
        report_dir.mkdir(parents=True,exist_ok=True)
        result=dict(empty_hand=empty,events=events,physics_contacts=contacts,
                    visible_viewer=visible,
                    note='Closure/contact observations only; retention assessment is step 5')
        closing=[event for event in events['grasp'] if event['state']=='opposing_thumb']
        if closing:
            samples=[s for s in contacts if s['elapsed_s']>=closing[0]['elapsed_s']]
            if len(samples)>1:
                result['max_contact_gap_during_closure_s']=max(
                    b['elapsed_s']-a['elapsed_s'] for a,b in zip(samples,samples[1:]))
                result['max_contact_measurement_age_during_closure_s']=max(s['measurement_age_s'] for s in samples)
        if arm_start is not None:
            try:result['final_arm_drift_rad']=distance(arm_start,grasp.measured(ARM_JOINTS))
            except ValueError:pass
        (report_dir/'closure_review.json').write_text(json.dumps(result,indent=2)+'\n')
        if images:
            from PIL import Image as PILImage
            for name,msg in images.items():
                if msg.encoding=='rgb8':
                    PILImage.frombytes('RGB',(msg.width,msg.height),bytes(msg.data),'raw','RGB',msg.step).save(
                        report_dir/(name+'.png'))
        approach.cancel_active();grasp.cancel_active()
        deadline=time.monotonic()+5
        while (approach.active_request or grasp.active_request) and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
        executor.shutdown()
        for node in (approach,grasp,peer):node.destroy_node()
        rclpy.shutdown();os.killpg(backend.pid,signal.SIGINT)
        try:backend.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(backend.pid,signal.SIGKILL);backend.wait(timeout=5)
        log.close()
