#!/usr/bin/env python3
#
# Pick and place with the gripper of the robot (Franka Emika Panda: pick_and_place.json; Trossen WidowX AI:
# pick_and_place.widowx_ai.json), environment pick_and_place/table_with_small_box.xml: the robot stands on a pedestal
# next to the table and moves a cube (5 cm, 0.15 kg, a free body of the simulation) to another place of the table.
#
#   pick:  home pose, open the gripper, fast motion (move_cartesianspace_absolute_pose, AboveCube of the task list) until
#          the fingertips are 2 cm above the table, guarded motion down until they touch it (the exact height of the
#          table is not assumed), up a little, close the gripper (SUCCEED: the cube is held; CANCEL:
#          the fingers closed completely: a red label "no cube", the gripper opens, the robot moves home and the skill
#          ends with NO_OBJECT), lift;
#   place: fast motion above the target until the cube is 2 cm above the table (AboveTarget), tare of the F/T sensor
#          (the weight of the held cube becomes its offset), guarded motion down until the cube touches the table, open
#          the gripper, retreat, home.
#
# The gripper is not controlled by eTaSL: the BeTFSM states of fake_gripper_control.py command the simulated gripper of
# the Crospi Playground ("gripper" of the setup) over ROS 2 topics of the simulator. Only for the simulation: for a real
# gripper, states for its driver replace them. Positions are in the robot base frame (the robot faces the table: its x
# axis points along -y of the world). The poses of AboveCube and AboveTarget (tcp_frame, tool pointing down) are in the
# task list of the robot: they depend on its tcp_frame (Panda: 12 mm above the fingertips; WidowX AI: at the fingertips)
# and match CUBE, TARGET and TABLE_Z below; the small motions (up, lift, retreat) are move_cartesianspace_relative.

import os
import sys

import rclpy
from betfsm import AlwaysOutcome, Compute, Fallback, Message, Sequence, get_logger, set_logger
from betfsm_crospi import CrospiTask, load_task_list
from betfsm_ros import BeTFSMNode, ROSRunner, ServiceClient
from std_srvs.srv import Trigger

from crospi_playground import playground_markers as mk
from crospi_playground.betfsm_markers import MarkerEdit
from fake_gripper_control import GripperClose, GripperOpen

CUBE = [0.40, 0.0]        # cube on the table (robot base frame) [m] (AboveCube of the task list)
TARGET = [0.35, 0.15]     # where it is placed [m] (AboveTarget of the task list)
TABLE_Z = -0.05           # table top (robot base frame) [m]: for the markers (the guarded motions find it)
GRASP_HEIGHT = 0.015      # the fingertips touch the table, then the gripper moves up by this before closing [m]
LIFT = 0.10               # lift of the cube [m]
GRASP_FORCE = 20.0        # per finger [N]


def relative(dx=0.0, dy=0.0, dz=0.0):
    """CrospiTask callback: parameters of MoveRelative (overrides the task list): a displacement of the tool [m]."""
    return lambda state, bb: {"delta_pos": [dx, dy, dz], "delta_euler": [0.0, 0.0, 0.0]}


def label(text, color="#ffffff"):
    return MarkerEdit(f"label: {text}", "label", text=text, color=color)


class PickAndPlace(Sequence):
    def __init__(self):
        super().__init__("pick_and_place", [
            Message(None, msg="pick_and_place: move the cube to the target"),
            # markers: the drop target on the table and a label (JSON as copied from the Markers tab)
            MarkerEdit("target", json='{"is-cylinder":true,"name":"drop target","length":0.002,"radius":0.035,'
                                      '"color":"#59a14f","opacity":0.6}',
                       position=[TARGET[0], TARGET[1], TABLE_Z + 0.002]),
            MarkerEdit("label", json='{"is-text":true,"name":"label","position":[0.3,0.0,0.3],"size":0.025,'
                                     '"color":"#ffffff","opacity":1}', text="moving home"),
            GripperOpen("open gripper"),
            CrospiTask("MovingHome", "MovingHome"),
            ServiceClient("tare F/T sensor", "/mujoco_robotsimulator/tare_ft_sensor", Trigger, []),  # tare of the F/T sensor before force control
            # ---- pick
            label("pick: above the cube"),
            CrospiTask("above the cube", "AboveCube"),
            label("pick: finding the table (guarded motion)"),
            CrospiTask("guarded down to the table", "GuardedDownPick"),
            CrospiTask("up to the grasp height", "MoveRelative", cb=relative(dz=GRASP_HEIGHT)),
            label("pick: closing the gripper"),
            # CANCEL of GripperClose (the fingers closed completely, nothing between them) is a failure of the Fallback: it
            # continues with "no cube", which reports it, opens the gripper and moves home; its outcome NO_OBJECT is not a
            # failure: the Fallback, and so the skill, ends with NO_OBJECT. TIMEOUT (the gripper did not stop) is not a
            # failure either: returned as is (no "no cube" label)
            Fallback("grasp", [
                GripperClose("close gripper", force=GRASP_FORCE),
                Sequence("no cube", [
                    label("no cube between the fingers", "#e15759"),
                    Message(None, msg="pick_and_place: no cube between the fingers"),
                    GripperOpen("open gripper (no cube)"),
                    CrospiTask("MovingHome (no cube)", "MovingHome"),
                    AlwaysOutcome("no cube", "NO_OBJECT"),
                ]),
            ]),
            label("pick: lifting the cube"),
            CrospiTask("lift", "MoveRelative", cb=relative(dz=LIFT)),
            # ---- place
            label("place: moving to the target"),
            CrospiTask("above the target", "AboveTarget"),
            ServiceClient("tare F/T sensor (cube held)", "/mujoco_robotsimulator/tare_ft_sensor", Trigger, []),  # tare of the weight of the cube before the guarded motion
            label("place: lowering until the cube touches the table (guarded motion)"),
            CrospiTask("guarded down with the cube", "GuardedDownPlace"),
            GripperOpen("release"),
            CrospiTask("retreat", "MoveRelative", cb=relative(dz=LIFT)),
            label("done", "#59a14f"),
            CrospiTask("MovingHome at the end", "MovingHome"),
            Compute("report", ["result"], lambda bb: {"opening_when_held": bb.get("gripper", {}).get("opening")}),
        ])


def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("pick_and_place")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("pick_and_place started")
    mk.init(my_node)  # outside the playground: RViz markers published by this node
    blackboard = {}
    json_name = os.path.splitext(os.path.basename(__file__))[0] + ".json"
    load_task_list(json_name, blackboard)
    runner = ROSRunner(my_node, PickAndPlace(), blackboard, frequency=100.0, publish_frequency=5.0, debug=False,
                       display_active=False)
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
