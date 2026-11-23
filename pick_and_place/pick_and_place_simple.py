#!/usr/bin/env python3
#
# Pick and place (simple) with the gripper of the robot (task list of the robot: Franka Emika Panda:
# pick_and_place_simple.json (the default); Trossen WidowX AI: pick_and_place_simple_widowx_ai.json; UR10e with a parallel gripper:
# pick_and_place_simple_ur10e_gripper.json; given as an argument: python3 pick_and_place_simple.py pick_and_place_simple_widowx_ai.json),
# environment pick_and_place/table_with_small_box.xml: the robot stands on a
# pedestal next to the table and moves a cube (5 cm, 0.15 kg, a free body of the simulation) to another place of the
# table. Only motions to known poses (no guarded motions, no markers) and the gripper commands, in a fixed sequence:
#
#   pick:  home pose (above the cube), open the gripper, down to the grasp pose (GraspPose of the task list), close the
#          gripper, lift;
#   place: above the target (AboveTarget), down to the place pose (PlacePose, the cube just above the table), open the
#          gripper, retreat, home.
#
# The outcome of the grasp is not checked: without a cube, the sequence is the same (see pick_and_place.py for the
# advanced version: guarded motions to find the table, a "no cube" branch, markers).
#
# The poses (tcp_frame, tool pointing down, robot base frame) are in the task list of the robot: they depend on its
# tcp_frame (Panda: 12 mm above the fingertips; WidowX AI: at the fingertips). The gripper is not controlled by eTaSL:
# the BeTFSM states of fake_gripper_control.py command the simulated gripper of the Crospi Playground ("gripper" of the
# setup) over ROS 2 topics of the simulator; only for the simulation: for a real gripper, states for its driver replace
# them.

import os
import sys

import rclpy
from betfsm import Message, Sequence, get_logger, set_logger
from betfsm_crospi import CrospiTask, load_task_list
from betfsm_ros import BeTFSMNode, ROSRunner
# Breakpoint of the Crospi Playground: uncomment to pause the playground in a state, e.g.
#   PlaygroundBreakpoint("check pose", message="Check the pose of the tool, then Resume")
# (does nothing outside the playground)
# from crospi_playground.betfsm_debug import PlaygroundBreakpoint

from fake_gripper_control import GripperClose, GripperMove, GripperOpen  # noqa: F401 (GripperClose: see below)

LIFT = 0.10               # lift of the cube and retreat after placing it [m]
GRASP_FORCE = 20.0        # per finger [N]


def up(dz):
    """CrospiTask callback: parameters of MoveRelative (overrides the task list): the tool moves up by dz [m]."""
    return lambda state, bb: {"delta_pos": [0.0, 0.0, dz], "delta_euler": [0.0, 0.0, 0.0]}


class PickAndPlaceSimple(Sequence):
    def __init__(self):
        super().__init__("pick_and_place_simple", [
            Message(None, msg="pick_and_place_simple: move the cube to the target"),
            GripperOpen("open gripper"),
            CrospiTask("MovingHome", "MovingHome"),
            # ---- pick
            CrospiTask("down to the grasp pose", "GraspPose"),
            GripperMove("close gripper", 0.0, GRASP_FORCE),   # closes until the fingers stop (the result is not checked)
            # GripperClose("close gripper", force=GRASP_FORCE),  # closes until the fingers stop (the result is checked and it fails if the cube is not grasped)
            CrospiTask("lift", "MoveRelative", cb=up(LIFT)),
            # ---- place
            CrospiTask("above the target", "AboveTarget"),
            CrospiTask("down to the place pose", "PlacePose"),
            GripperOpen("release"),
            CrospiTask("retreat", "MoveRelative", cb=up(LIFT)),
            CrospiTask("MovingHome at the end", "MovingHome"),
        ])


def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("pick_and_place_simple")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("pick_and_place_simple started")
    blackboard = {}
    # Task list: by default the .json file with the name of this script; another one (e.g. the task list of another
    # robot) can be given as the first argument: python3 <this script> <task list: a file name or a complete path>
    # It is taken out of sys.argv: the runner of BeTFSM parses the other arguments (its options, e.g. --serve).
    json_name = sys.argv.pop(1) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else os.path.splitext(os.path.basename(__file__))[0] + ".json"
    load_task_list(json_name, blackboard)
    runner = ROSRunner(my_node, PickAndPlaceSimple(), blackboard, frequency=100.0, publish_frequency=5.0, debug=False,
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
