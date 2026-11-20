#!/usr/bin/env python3
#
# Rectangular peg-in-hole insertion with force control, following the methodology of
#
#   F. Ulloa Rios, S. Iregui, E. Aertbeliën, W. Decré, H. Bruyninckx, "Robust Force-Controlled Robotic Insertion
#   Skills for Geometrically Diverse Parts Under Uncertainty", CASE 2026.
#
# for a rectangular peg (30 x 20 mm, 10 cm long) and a rectangular hole 50 mm deep with a clearance of 0.5 mm
# (30.5 x 20.5 mm), 2, 1 or 0.1 mm (HOLE_CLEARANCE), environments peg_insertion_rectangular/table_with_rectangular_hole_<c>mm_clearance.xml
# (setups ur10e_gripper_peg_insertion_rectangular_<c>mm.setup.json), UR10e with a parallel gripper.
#
#   pick:   home, open the gripper, above the peg, down to the grasp pose (absolute Cartesian poses of the task list),
#           close the gripper (the fingers hold the top 30 mm of the peg), lift (LiftPeg of the task list);
#   insert: initial placement: the peg is brought above the hole with a known offset (pose) w.r.t. the hole, then the
#           offset is reduced one degree of freedom (DoF) at a time by a sequence of insertion actions (InsertionSequence:
#           guarded motions with force control, task insertion_action of force_control_task_lib), each finishing when a
#           monitor detects the contact transition of its DoF; a last action pushes the peg to the bottom of the hole;
#           release, retreat (Retreat of the task list), home.
#
# Mating features: corners of the tip face of the peg (named as in the paper: f, u, m), expressed in the control frame
# {f_i} at the feature corner:
#
#     short-edge corner (m) ●───────────────────● ·          y ↑         z (insertion direction): into the drawing
#                           │      peg tip      │              │
#        feature corner (f) ●───────────────────● long-edge    └──→ x
#                                                 corner (u)
#     (long edge: 30 mm along x, short edge: 20 mm along y)
#
# The hole has the same corner, with the frame {f_h} of the housing feature (z along the insertion direction). The
# initial placement puts {f_i} at the offset T_off w.r.t. {f_h} (Sec. V-C of the paper): a translation OFFSET_DISTANCE
# along the bisector of the corner (into the hole) and OFFSET_DISTANCE up, and the rotations
# Rz(theta_z) Rx(-theta_x) Ry(theta_y): the feature corner is the lowest point of the peg, inside the outline of the
# hole. The insertion actions (Sec. V-D of the paper; the target forces at the corners are superposed at {f_i}, see
# InsertionSequence below) then reduce:
#   I   z         (the feature corner dips into the hole: the peg lands on the rim)   {z: -F} at the feature corner
#   II  y         (the feature corner against the long wall of the hole)               {y: F, z: -eF} at the feature corner
#   III x         (the feature corner in the corner of the hole)                       {x: F, y: eF, z: -eF} at the feature corner
#   IV  theta_y   (long-edge corner down: the long edge level)                        ... + {z: -F} at the long-edge corner
#   V   theta_z   (long-edge corner against the long wall: the long edge along it)    ... + {y: F, z: -eF} at the long-edge corner
#   VI  theta_x   (short-edge corner down: the peg aligned with the hole)             ... + {y: eF, z: -eF} at the long-edge corner,
#                                                                                         {z: -F} at the short-edge corner
#   VII insertion (pushed to the bottom, compliant in the other DoFs)                 {x: 0, y: 0, z: -F, rx: 0, ry: 0, rz: 0}
# F: FEATURE_FORCE, e: REDUCED_FORCE_FACTOR (for the DoFs already reduced). Change HOLE_POSE_ERROR to see that the skill
# does not need the exact pose of the hole (robustness to the uncertainty of its estimation).
#
# The gripper is not controlled by eTaSL: the BeTFSM states of ../pick_and_place/fake_gripper_control.py command the
# simulated gripper of the Crospi Playground over ROS 2 topics (only for the simulation: for a real gripper, states for
# its driver replace them).

#  Copyright (c) 2026 KU Leuven, Belgium
#
#  Author: Santiago Iregui (based on Federico Ulloa's code)
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

import math
import os
import sys

import numpy as np
import rclpy
from betfsm import Message, Sequence, get_logger, set_logger
from betfsm_crospi import CrospiTask, load_task_list
from betfsm_ros import BeTFSMNode, ROSRunner, ServiceClient
from std_srvs.srv import Trigger

from crospi_playground import playground_markers as mk
from crospi_playground.betfsm_markers import MarkerEdit

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pick_and_place"))
from fake_gripper_control import GripperClose, GripperOpen  # noqa: E402

# ---- geometry (robot base frame = world)
HOLE_ENTRANCE = [0.173664, -0.614962, 0.210]  # center of the hole at the top surface of the block [m]
HOLE_YAW = -0.000789                          # rotation of the hole about z [rad] (long side along x)
PEG_SIZE = [0.030, 0.020]                     # long and short side of the peg [m]
HOLE_CLEARANCE = 0.0005                       # diametral clearance of the hole [m]: as the environment of the setup
                                              # (table_with_rectangular_hole_<clearance>mm_clearance.xml: 0.5 (default), 2, 1, 0.1 mm)
HOLE_SIZE = [PEG_SIZE[0] + HOLE_CLEARANCE, PEG_SIZE[1] + HOLE_CLEARANCE]   # long and short side of the hole [m]
PEG_TIP_BELOW_TCP = 0.070                     # held peg: its tip is 70 mm below the tcp_frame (fingertips) [m]
HOLE_POSE_ERROR = [0.0, 0.0, 0.0]             # error of the assumed pose of the hole [dx m, dy m, dyaw rad]

# ---- initial placement (offset of {f_i} w.r.t. {f_h}, Sec. V-C of the paper)
OFFSET_DISTANCE = 0.008                       # translation offset d [m]: along the bisector of the corner and up
OFFSET_ANGLES = [math.radians(10), math.radians(10), math.radians(5)]   # theta_x, theta_y, theta_z [rad]
APPROACH_HEIGHT = 0.05                        # the offset pose is approached from this height above it [m]

# ---- insertion actions
FEATURE_FORCE = 5.0                           # force set-point at a mating feature (F of the paper) [N]
REDUCED_FORCE_FACTOR = 0.5                    # multiplier of the force of the DoFs already reduced (e of the paper)
GRASP_FORCE = 30.0                            # per finger [N]


# ---------------------------------------------------------------------------------------------------- frames
def rot_x(angle):
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, cos_a, -sin_a], [0, sin_a, cos_a]])


def rot_y(angle):
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return np.array([[cos_a, 0, sin_a], [0, 1, 0], [-sin_a, 0, cos_a]])


def rot_z(angle):
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return np.array([[cos_a, -sin_a, 0], [sin_a, cos_a, 0], [0, 0, 1]])


def frame(rotation, position):
    """4x4 homogeneous transformation."""
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, position
    return transform


def quaternion(rotation):
    """Unit quaternion [qx, qy, qz, qw] of a rotation matrix."""
    R = rotation
    qw = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    qx = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    qy = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    qz = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [qx, qy, qz, qw]


def pose(transform):
    """[x, y, z, qx, qy, qz, qw] of a frame (the poses of the task specifications)."""
    return list(transform[:3, 3]) + quaternion(transform[:3, :3])


def rpy(rotation):
    """roll, pitch, yaw (eTaSL convention R = Rz(yaw) Ry(pitch) Rx(roll)) of a rotation matrix (markers)."""
    R = rotation
    return [math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0])]


# {f_h}: corner of the hole (the housing feature), z along the insertion direction (down) and x along the long side;
# the orientation of the tool pointing down, as when the peg was grasped (Ry(pi): x of the tool along -x of the world)
R_WORLD_HOLE_CORNER = rot_z(HOLE_YAW + HOLE_POSE_ERROR[2]) @ rot_y(math.pi)
T_WORLD_HOLE_CORNER = frame(R_WORLD_HOLE_CORNER, np.array(HOLE_ENTRANCE) + np.array(HOLE_POSE_ERROR[:2] + [0.0])
                            + R_WORLD_HOLE_CORNER @ [-HOLE_SIZE[0] / 2, -HOLE_SIZE[1] / 2, 0.0])
# {f_i}: feature corner of the tip face of the held peg, w.r.t. the tcp_frame (same orientation)
T_TCP_PEG_CORNER = frame(np.eye(3), [-PEG_SIZE[0] / 2, -PEG_SIZE[1] / 2, PEG_TIP_BELOW_TCP])
# the mating features (corners of the tip face of the peg), in {f_i}
FEATURE_CORNER = np.array([0.0, 0.0, 0.0])              # f of the paper: origin of {f_i}
LONG_EDGE_CORNER = np.array([PEG_SIZE[0], 0.0, 0.0])    # u of the paper: other end of the long edge
SHORT_EDGE_CORNER = np.array([0.0, PEG_SIZE[1], 0.0])   # m of the paper: farthest from the feature corner along y

# offset T_off (Eq. 2-5): translation d*bisector - d*insertion_direction, rotation Rz(s*theta_z) Rx(-s*theta_x) Ry(theta_y)
INSERTION_DIRECTION = np.array([0.0, 0.0, 1.0])
ROTATION_SIGN = float(np.sign(np.dot(np.cross(LONG_EDGE_CORNER, SHORT_EDGE_CORNER), INSERTION_DIRECTION)))  # s_psi: +1 here
bisector = LONG_EDGE_CORNER / np.linalg.norm(LONG_EDGE_CORNER) + SHORT_EDGE_CORNER / np.linalg.norm(SHORT_EDGE_CORNER)
bisector = bisector / np.linalg.norm(bisector)
T_OFFSET = frame(rot_z(ROTATION_SIGN * OFFSET_ANGLES[2]) @ rot_x(-ROTATION_SIGN * OFFSET_ANGLES[0]) @ rot_y(OFFSET_ANGLES[1]),
                 OFFSET_DISTANCE * bisector - OFFSET_DISTANCE * INSERTION_DIRECTION)
T_WORLD_TCP_OFFSET = T_WORLD_HOLE_CORNER @ T_OFFSET @ np.linalg.inv(T_TCP_PEG_CORNER)
T_WORLD_TCP_APPROACH = frame(T_WORLD_TCP_OFFSET[:3, :3], T_WORLD_TCP_OFFSET[:3, 3] + [0, 0, APPROACH_HEIGHT])


# ---------------------------------------------------------------------------------------------------- BeTFSM states
def wrench_at(corner, force):
    """Wrench [Fx, Fy, Fz, Tx, Ty, Tz] at {f_i} of a target force [Fx, Fy, Fz] applied at a corner (position in {f_i})."""
    force = np.array(force, dtype=float)
    return np.concatenate([force, np.cross(corner, force)])


def label(text, color="#ffffff"):
    return MarkerEdit(f"label: {text}", "label", text=text, color=color)


def move_to(name, transform):
    """CrospiTask of MoveToPose (task list) to a pose of the tcp_frame computed above."""
    return CrospiTask(name, "MoveToPose", cb=lambda state, bb, desired_pose=pose(transform): {"desired_pose": desired_pose})


class InsertionAction(Sequence):
    """
    One insertion action: InsertionAction of the task list (task insertion_action), with the target wrench at {f_i}
    and the reduced DoF of this action. By default, the DoFs with a non-zero target are force controlled, the others
    keep a zero velocity.
    """

    def __init__(self, name, reduced_dof, target_wrench, force_controlled=None):
        target_wrench = [round(float(value), 6) for value in target_wrench]
        if force_controlled is None:
            force_controlled = [1 if value != 0 else 0 for value in target_wrench]
        parameters = {
            "feature_frame": pose(T_TCP_PEG_CORNER),
            "housing_orientation": quaternion(R_WORLD_HOLE_CORNER),
            "target_wrench": target_wrench,
            "force_controlled": force_controlled,
            "reduced_dof": reduced_dof,
        }
        super().__init__(name, [
            label(f"insertion action {name}"),
            Message(None, msg=f"insertion action {name}: target wrench {target_wrench}, force controlled {force_controlled}"),
            CrospiTask(name, "InsertionAction", cb=lambda state, bb: parameters),
        ])


class InsertionSequence(Sequence):
    """Sequential DoF reductions (Sec. V-D of the paper), then the insertion to the bottom of the hole."""

    def __init__(self):
        force = FEATURE_FORCE                                  # F: a new contact
        reduced = REDUCED_FORCE_FACTOR * FEATURE_FORCE         # e*F: keeps a contact of a DoF already reduced
        sign = ROTATION_SIGN
        keep_feature_corner = wrench_at(FEATURE_CORNER, [reduced, sign * reduced, -reduced])
        super().__init__("insertion_sequence", [
            InsertionAction("I: reduce the z offset", "z",
                            wrench_at(FEATURE_CORNER, [0, 0, -force])),
            InsertionAction("II: reduce the y offset", "y",
                            wrench_at(FEATURE_CORNER, [0, sign * force, -reduced])),
            InsertionAction("III: reduce the x offset", "x",
                            wrench_at(FEATURE_CORNER, [force, sign * reduced, -reduced])),
            InsertionAction("IV: reduce theta_y", "ry",
                            keep_feature_corner + wrench_at(LONG_EDGE_CORNER, [0, 0, -force])),
            InsertionAction("V: reduce theta_z", "rz",
                            keep_feature_corner + wrench_at(LONG_EDGE_CORNER, [0, sign * force, -reduced])),
            InsertionAction("VI: reduce theta_x", "rx",
                            keep_feature_corner + wrench_at(LONG_EDGE_CORNER, [0, sign * reduced, -reduced])
                            + wrench_at(SHORT_EDGE_CORNER, [0, 0, -force])),
            # all DoFs compliant (zero targets) except the push along the insertion direction
            InsertionAction("VII: insert", "z",
                            wrench_at(FEATURE_CORNER, [0, 0, -force]), force_controlled=[1, 1, 1, 1, 1, 1]),
        ])


class RectangularPegInsertion(Sequence):
    def __init__(self):
        super().__init__("rectangular_peg_insertion", [
            Message(None, msg="rectangular_peg_insertion: pick the peg and insert it in the hole"),
            # markers: the frame {f_h} of the corner of the hole (as assumed), the frame {f_i} of the corner of the peg
            # (during the insertion actions) and a label
            MarkerEdit("hole corner", type="frame", position=list(T_WORLD_HOLE_CORNER[:3, 3]),
                       orientation=rpy(R_WORLD_HOLE_CORNER), size=0.03),
            MarkerEdit("peg corner", type="frame", position=mk.expr("feature_frame"), size=0.03),
            MarkerEdit("label", type="text", position=[0.17, -0.45, 0.45], size=0.025, text="moving home"),
            GripperOpen("open gripper"),
            CrospiTask("MovingHome", "MovingHome"),
            # ---- pick
            label("pick: above the peg"),
            CrospiTask("above the peg", "AbovePeg"),
            CrospiTask("down to the grasp pose", "GraspPeg"),
            label("pick: closing the gripper"),
            GripperClose("close gripper", force=GRASP_FORCE),  # CANCEL (no peg between the fingers): the skill fails
            CrospiTask("lift the peg", "LiftPeg"),
            # ---- initial placement
            label("insert: initial placement (offset w.r.t. the hole)"),
            move_to("above the offset pose", T_WORLD_TCP_APPROACH),
            ServiceClient("tare F/T sensor (peg held)", "/mujoco_robotsimulator/tare_ft_sensor", Trigger, []),  # the weight of the peg becomes the offset
            move_to("offset pose", T_WORLD_TCP_OFFSET),
            # ---- sequential DoF reductions
            InsertionSequence(),
            # ---- release
            label("inserted: releasing the peg", "#59a14f"),
            GripperOpen("release"),
            CrospiTask("retreat", "Retreat"),
            label("done", "#59a14f"),
            CrospiTask("MovingHome at the end", "MovingHome"),
        ])


def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("rectangular_peg_insertion")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("rectangular_peg_insertion started")
    mk.init(my_node)  # outside the playground: RViz markers published by this node
    blackboard = {}
    json_name = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.path.splitext(os.path.basename(__file__))[0] + ".json")
    load_task_list(json_name, blackboard)
    runner = ROSRunner(my_node, RectangularPegInsertion(), blackboard, frequency=100.0, publish_frequency=5.0,
                       debug=False, display_active=False)
    try:
        runner.run()
    except KeyboardInterrupt:
        my_node.destroy_node()
        return
    my_node.destroy_node()
    rclpy.shutdown()
    print("shutdown")


if __name__ == "__main__":
    sys.exit(main(sys.argv))
