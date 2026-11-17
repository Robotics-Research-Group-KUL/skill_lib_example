#!/usr/bin/env python3
#
# Pick and place with the gripper of the robot (Franka Emika Panda: pick_and_place.json; Trossen WidowX AI:
# pick_and_place.widowx_ai.json), environment pick_and_place/table_with_small_box.xml: the robot stands on a pedestal
# next to the table and moves a cube (5 cm, 0.15 kg, a free body of the simulation) to another place of the table.
#
#   pick:  above the cube (home pose), open the gripper, guarded motion down until the fingertips touch the table (the
#          height of the table is not assumed), up a little, close the gripper (SUCCEED: the cube is held; NO_OBJECT:
#          the fingers closed completely), lift;
#   place: above the target, guarded motion down until the cube touches the table (the force monitor of this guarded
#          motion, 2 N, is above the weight of the cube held by the gripper, about 1 N after the dead zone), open
#          the gripper, retreat, home.
#
# The gripper is not controlled by eTaSL: the BeTFSM states of gripper_states.py command it over ROS 2 topics (in the
# Crospi Playground: the gripper of the simulator, "gripper" of the setup). The motions are relative to the home pose
# (tool pointing down above the cube): move_cartesianspace_relative with the displacement computed from the plan in the
# blackboard. Positions are in the robot base frame (the robot faces the table: its x axis points along -y of the world).

import os
import sys

import rclpy
from betfsm import Compute, Message, Sequence, get_logger, set_logger
from betfsm_crospi import CrospiTask, load_task_list
from betfsm_ros import BeTFSMNode, ROSRunner

from crospi_playground import playground_markers as mk
from crospi_playground.betfsm_markers import MarkerEdit
from gripper_states import GripperClose, GripperOpen

CUBE = [0.40, 0.0]        # cube on the table (robot base frame) [m]: below the tool at the home pose
TARGET = [0.35, 0.15]     # where it is placed [m]
TABLE_Z = -0.05           # table top (robot base frame) [m]: only for the markers (the motions find it)
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
            # ---- pick
            label("pick: finding the table (guarded motion)"),
            CrospiTask("guarded down to the table", "GuardedDownPick"),
            CrospiTask("up to the grasp height", "MoveRelative", cb=relative(dz=GRASP_HEIGHT)),
            label("pick: closing the gripper"),
            GripperClose("close gripper", force=GRASP_FORCE),
            label("pick: lifting the cube"),
            CrospiTask("lift", "MoveRelative", cb=relative(dz=LIFT)),
            # ---- place
            label("place: moving to the target"),
            CrospiTask("to the target", "MoveRelative", cb=relative(dx=TARGET[0] - CUBE[0], dy=TARGET[1] - CUBE[1])),
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
