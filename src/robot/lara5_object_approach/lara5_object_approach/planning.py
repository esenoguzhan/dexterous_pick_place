"""Two-phase trajectory preview. Only read-only services; no controller clients."""

import copy
import json
import math
import time
import uuid

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from lara5_safety_msgs.srv import CheckTrajectory, GetFramePose, PlanCartesianPath
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from .grasp_geometry import transform
from .planning_geometry import ArmModel, path_poses, pose_error, pregrasp_target, retime, seconds


class TrajectoryPreview:
    def __init__(self, node, description, flange_link, open_names, open_positions):
        self.node = node
        def param(name, default):
            return node.declare_parameter(name, default).value
        self.names = list(param("arm_joint_names", [f"lara5_joint{i}" for i in range(1, 7)]))
        self.model = ArmModel(description, node.world_frame, flange_link, self.names)
        self.flange_link = flange_link
        self.open_names, self.open_positions = list(open_names), np.asarray(open_positions)
        self.clearance = param("pregrasp_flange_clearance", .30)
        self.state_age = param("planning_state_max_age", 1.)
        self.drift = param("planning_state_tolerance", .01)
        self.open_tolerance = param("planning_open_tolerance", .03)
        self.max_step = param("planning_max_joint_step", .2)
        self.velocity = param("planning_max_joint_speed", .25)
        self.acceleration = param("planning_max_joint_acceleration", .5)
        self.position_tolerance = param("planning_position_tolerance", .001)
        self.angle_tolerance = param("planning_angle_tolerance", .01)
        self.path_tolerance = param("planning_path_tolerance", .003)
        self.timeout = param("planning_timeout", 60.)
        self.service_timeout = param("planning_service_timeout", 15.)
        values = [self.clearance, self.state_age, self.drift, self.open_tolerance, self.max_step,
                  self.velocity, self.acceleration, self.position_tolerance, self.angle_tolerance,
                  self.path_tolerance, self.timeout, self.service_timeout]
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("planning limits must all be finite and positive")
        self.samples = {}
        self.active = None
        self.valid = None
        self.pending = None
        self.last_id = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        base = "/arm/approach/preview/"
        self.status = node.create_publisher(String, base+"plan_status", qos)
        self.outputs = [node.create_publisher(JointTrajectory, base+suffix, qos)
                        for suffix in ("pregrasp_trajectory", "final_trajectory")]
        self.markers = node.create_publisher(MarkerArray, base+"path_markers", qos)
        self.pregrasp_pose = node.create_publisher(PoseStamped, base+"pregrasp_pose", qos)
        self.joints = node.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.plan_client = node.create_client(PlanCartesianPath, "/safety/arm/plan_cartesian_path")
        self.check_client = node.create_client(CheckTrajectory, "/safety/arm/check_trajectory")
        self.fk_client = node.create_client(GetFramePose, "/safety/arm/get_frame_pose")
        self.timer = node.create_timer(.1, self.on_timer)
        self.report("idle", "Planning only; waiting for manual pose and fresh open-hand joint state")

    @property
    def busy(self):
        return self.active is not None

    def report(self, state, message, **values):
        payload = dict(state=state, message=message, plan_id=self.last_id,
                       active_plan_id=self.active["id"] if self.active else None,
                       executable=False, **values)
        self.status.publish(String(data=json.dumps(payload)))
        self.node.get_logger().info(f"{state}: {message}")

    def clear_outputs(self):
        self.valid = None
        for output in self.outputs:
            output.publish(JointTrajectory(joint_names=self.names))
        self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        self.pregrasp_pose.publish(PoseStamped())

    def reject_input(self, error):
        self.last_id = str(uuid.uuid4())
        self.clear_outputs()
        self.report("rejected_invalid", str(error))

    def on_state(self, message):
        required = set(self.names+self.open_names)
        if (len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)
                or not np.isfinite(message.position).all()):
            for name in required:
                self.samples.pop(name, None)
            return
        receipt = time.monotonic()
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        for name, value in zip(message.name, message.position):
            if name in required:
                self.samples[name] = (float(value), receipt, stamp)

    def measured(self):
        now, ros_now = time.monotonic(), self.node.get_clock().now().nanoseconds/1e9
        values = []
        for name in self.names+self.open_names:
            sample = self.samples.get(name)
            if sample is None:
                raise ValueError(f"missing measured joint {name}")
            value, receipt, stamp = sample
            if stamp <= 0 or now-receipt > self.state_age or not -.1 <= ros_now-stamp <= self.state_age:
                raise ValueError(f"stale or invalid timestamp for measured joint {name}")
            values.append(value)
        arm = self.model.check(values[:6])
        if np.max(np.abs(np.asarray(values[6:])-self.open_positions)) > self.open_tolerance:
            raise ValueError("hand is not in the configured open posture; open it explicitly")
        return arm

    def stationary(self, start):
        if np.max(np.abs(self.measured()-start)) > self.drift:
            raise ValueError("measured arm changed since planning started; request a new preview")

    def state_message(self, q):
        message = JointState(name=self.names, position=list(map(float, q)))
        message.header.stamp = self.node.get_clock().now().to_msg()
        return message

    def request(self, message, obj, final, grasp):
        if self.busy:
            self.report("rejected_busy", "A plan is active; new target discarded")
            return
        self.last_id = str(uuid.uuid4())
        self.clear_outputs()
        try:
            start = self.measured().copy()
            pregrasp = pregrasp_target(obj, final, grasp, self.clearance)
            if not all(c.service_is_ready() for c in (self.plan_client, self.check_client, self.fk_client)):
                self.report("unavailable", "Read-only native planning/check/FK services are required; target discarded")
                return
        except ValueError as error:
            self.report("rejected_state", str(error))
            return
        context = dict(id=self.last_id, start=start, deadline=time.monotonic()+self.timeout,
                       targets=[pregrasp, final], phases=[], paths=[], errors=[], clearances=[], phase=0,
                       phase_start=start, phase_start_pose=self.model.fk(start))
        self.active = context
        self.report("planning", "Checking measured start and planning pregrasp", pregrasp_flange_clearance_m=self.clearance)
        self.call(context, self.fk_client, GetFramePose.Request(start_state=self.state_message(start),
                  frame=self.flange_link, reference_frame=self.node.world_frame), self.on_start_fk)

    def call(self, context, client, request, callback):
        try:
            future = client.call_async(request)
            self.pending = (client, future, time.monotonic()+self.service_timeout)
            def done(result):
                if self.active is not context:
                    return
                self.pending = None
                try:
                    self.stationary(context["start"])
                    response = result.result()
                    if response is None:
                        raise ValueError("native service returned no response")
                    callback(context, response)
                except Exception as error:
                    self.fail(str(error))
            future.add_done_callback(done)
        except Exception as error:
            self.fail(f"Cannot request native planning service: {error}")

    def checked_pose(self, response, desired):
        if not response.success:
            raise ValueError("Native FK failed: "+response.message)
        if response.pose.header.frame_id != self.node.world_frame:
            raise ValueError("Native FK returned an unexpected reference frame")
        p, q = response.pose.pose.position, response.pose.pose.orientation
        actual = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        error = pose_error(actual, desired)
        if error[0] > self.position_tolerance or error[1] > self.angle_tolerance:
            raise ValueError(f"FK endpoint/model mismatch: {error[0]:.6f} m, {error[1]:.6f} rad")
        return error

    def on_start_fk(self, context, response):
        self.checked_pose(response, context["phase_start_pose"])
        self.plan_phase(context)

    def plan_phase(self, context):
        # Phase B explicitly starts at Phase A's final arm configuration.
        from .preview import ros_pose
        target = PoseStamped()
        target.header.frame_id = self.node.world_frame
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose = ros_pose(context["targets"][context["phase"]])
        request = PlanCartesianPath.Request(start_state=self.state_message(context["phase_start"]),
                                             target=target, relative=False, duration=0.)
        self.call(context, self.plan_client, request, self.on_plan)

    def on_plan(self, context, response):
        if not response.success:
            raise ValueError("Native Cartesian planning failed: "+response.message)
        path = retime(response.trajectory, context["phase_start"], self.model,
                      self.max_step, self.velocity, self.acceleration)
        desired = context["targets"][context["phase"]]
        endpoint = self.model.fk(path.points[-1].positions)
        local_error = pose_error(endpoint, desired)
        if local_error[0] > self.position_tolerance or local_error[1] > self.angle_tolerance:
            raise ValueError("Incomplete native path: endpoint does not reach the requested flange target")
        poses = path_poses(path, self.model, context["phase_start_pose"], desired,
                           self.path_tolerance, self.angle_tolerance)
        context["candidate"], context["candidate_poses"] = path, poses
        self.call(context, self.fk_client, GetFramePose.Request(
            start_state=self.state_message(path.points[-1].positions), frame=self.flange_link,
            reference_frame=self.node.world_frame), self.on_endpoint)

    def on_endpoint(self, context, response):
        error = self.checked_pose(response, context["targets"][context["phase"]])
        context["errors"].append(error)
        request = CheckTrajectory.Request(start_state=self.state_message(context["phase_start"]),
                                           trajectory=context["candidate"])
        self.call(context, self.check_client, request, self.on_checked)

    def on_checked(self, context, response):
        if not response.valid:
            raise ValueError("Native trajectory check rejected phase: "+response.message)
        if math.isnan(response.clearance) or response.clearance < 0:
            raise ValueError("Native trajectory check returned invalid clearance")
        path = context["candidate"]
        context["phases"].append(path)
        context["paths"].append(context["candidate_poses"])
        # The backend uses +inf when broad-phase rejection means no finite
        # distance was evaluated. Keep its valid verdict, report no measurement.
        context["clearances"].append(response.clearance if math.isfinite(response.clearance) else None)
        if context["phase"] == 0:
            context["phase"] = 1
            context["phase_start"] = np.asarray(path.points[-1].positions)
            context["phase_start_pose"] = self.model.fk(context["phase_start"])
            self.report("planning", "Pregrasp validated; planning final approach from its endpoint")
            self.plan_phase(context)
        else:
            self.publish_plan(context)

    def publish_plan(self, context):
        from .preview import ros_pose
        self.stationary(context["start"])
        a, b = context["phases"]
        if not np.allclose(a.points[-1].positions, b.points[0].positions, atol=1e-10, rtol=0):
            raise ValueError("Phase boundary is discontinuous")
        stamp = self.node.get_clock().now().to_msg()
        for output, path in zip(self.outputs, context["phases"]):
            path.header.frame_id, path.header.stamp = self.node.world_frame, stamp
            output.publish(path)
        pregrasp = PoseStamped()
        pregrasp.header.frame_id, pregrasp.header.stamp = self.node.world_frame, stamp
        pregrasp.pose = ros_pose(context["targets"][0])
        self.pregrasp_pose.publish(pregrasp)
        markers = [Marker(action=Marker.DELETEALL)]
        for i, poses in enumerate(context["paths"]):
            line = Marker()
            line.header = copy.deepcopy(pregrasp.header)
            line.ns, line.id, line.type = "planned_flange_path", i, Marker.LINE_STRIP
            line.pose.orientation.w = 1.
            line.scale.x = .006
            line.color.r, line.color.g, line.color.b, line.color.a = ((1., .65, .1, 1.) if i == 0 else (.1, 1., 1., 1.))
            line.points = [Point(x=float(p[0, 3]), y=float(p[1, 3]), z=float(p[2, 3])) for p in poses]
            markers.append(line)
        self.markers.publish(MarkerArray(markers=markers))
        self.active = None
        self.valid = context
        self.report("plan_ready", "Both phases validated for preview; no execution command sent",
                    trajectory_stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec),
                    pregrasp_xyz=context["targets"][0][:3, 3].tolist(),
                    final_xyz=context["targets"][1][:3, 3].tolist(),
                    durations_s=[seconds(p.points[-1].time_from_start) for p in context["phases"]],
                    waypoints=[len(p.points) for p in context["phases"]],
                    endpoint_errors_m_rad=context["errors"], native_clearance_m=context["clearances"],
                    start_joint_names=self.names, start_joint_positions=context["start"].tolist(),
                    holder_camera_collision_checked=False, manual_object_collision_checked=False)

    def fail(self, message):
        self.active = None
        if self.pending:
            client, future, _ = self.pending
            client.remove_pending_request(future)
            future.cancel()
            self.pending = None
        self.clear_outputs()
        self.report("plan_failed", message)

    def on_timer(self):
        try:
            if self.active:
                self.stationary(self.active["start"])
                if time.monotonic() > self.active["deadline"] or (self.pending and time.monotonic() > self.pending[2]):
                    self.fail("Read-only planning request timed out")
            elif self.valid:
                self.stationary(self.valid["start"])
        except ValueError as error:
            if self.active:
                self.fail(str(error))
            elif self.valid:
                self.clear_outputs()
                self.report("plan_invalidated", str(error))
