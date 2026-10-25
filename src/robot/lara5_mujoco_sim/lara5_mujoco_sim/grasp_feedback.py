"""Physics contact feedback for grasp verification; no attachment or teleporting."""

import mujoco
import numpy as np
from lara5_safety_msgs.msg import SimGraspFeedback


def opposing_contacts(links, normals=None):
    """Require thumb and a distinct non-thumb finger, not two meshes on one digit."""
    thumbs = [name for name in links if "thumb" in name]
    fingers = [name for name in links
               if any(finger in name for finger in ("index", "middle", "ring", "pinky"))]
    if not thumbs or not fingers:
        return False
    if normals is None:
        return True
    return any(np.dot(a, b) < -0.2 for thumb in thumbs for finger in fingers
               for a in normals[thumb] for b in normals[finger])


def assign_pose(pose, position, rotation):
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, np.asarray(rotation).reshape(9))
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = map(float, quaternion)


class GraspFeedbackPublisher:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(SimGraspFeedback, "/sim/grasp_feedback", 20)
        self.flange = mujoco.mj_name2id(node.model, mujoco.mjtObj.mjOBJ_BODY, "lara5_flange")
        if self.flange < 0:
            raise ValueError("Grasp feedback needs a retained lara5_flange body")
        self.objects = {}
        for obj in node.config.get("objects", []):
            body = mujoco.mj_name2id(node.model, mujoco.mjtObj.mjOBJ_BODY, obj["name"])
            if body < 0:
                raise ValueError(f"Missing simulation object {obj['name']}")
            self.objects[obj["name"]] = body

    def publish(self):
        node, model, data = self.node, self.node.model, self.node.data
        messages = []
        with node.lock:
            stamp = node.get_clock().now().to_msg()
            flange_r = data.xmat[self.flange].reshape(3, 3)
            for name, body in self.objects.items():
                links, normals, forces, supported = set(), {}, {}, False
                for index, contact in enumerate(data.contact[:data.ncon]):
                    first, second = int(contact.geom1), int(contact.geom2)
                    bodies = int(model.geom_bodyid[first]), int(model.geom_bodyid[second])
                    if body not in bodies or contact.dist > 0.001:
                        continue
                    force = np.empty(6)
                    mujoco.mj_contactForce(model, data, index, force)
                    if force[0] < 0.01:
                        continue
                    other = second if bodies[0] == body else first
                    other_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                                  int(model.geom_bodyid[other])) or "world"
                    if other_name.startswith("right_"):
                        links.add(other_name)
                        forces[other_name] = forces.get(other_name, 0.0) + float(force[0])
                        normals.setdefault(other_name, []).append(
                            np.asarray(contact.frame[:3]).copy() * (-1 if bodies[0] == body else 1))
                    elif model.geom_contype[other] & 1:
                        supported = True
                message = SimGraspFeedback(object_name=name)
                message.header.stamp, message.header.frame_id = stamp, "world"
                assign_pose(message.object_pose, data.xpos[body], data.xmat[body])
                assign_pose(message.object_in_flange,
                            flange_r.T @ (data.xpos[body] - data.xpos[self.flange]),
                            flange_r.T @ data.xmat[body].reshape(3, 3))
                message.contact_links = sorted(links)
                message.contact_normal_forces = [forces[link] for link in message.contact_links]
                message.opposing_contacts = opposing_contacts(links, normals)
                message.supported = supported
                velocity = np.empty(6)
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
                message.linear_speed = float(np.linalg.norm(velocity[3:]))
                messages.append(message)
        for message in messages:
            self.publisher.publish(message)
