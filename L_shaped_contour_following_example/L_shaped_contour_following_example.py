#!/usr/bin/env python3

#
# Contour following: approach a contour with guarded motions and follow it with force control.
#
#   1. move home (above the contour)
#   2. guarded motion down until contact with the table
#   3. move up, such that the tool does not touch the table any more
#   4. guarded motion towards the contour until contact with it
#   5. follow the contour with force control (2D_contour_following task of force_control_task_lib)
#
# Designed for the MuJoCo simulator with the L-shaped contour
# ($[mujoco_robotsimulator_crospi]/config/setups/L_shaped_contour_following_example.setup.json).
#
# If control-C is pressed, the currently running crospi task will continue to run
#


#  Copyright (c) 2026 KU Leuven, Belgium
#
#  Author: Santiago Iregui, Erwin Aertbelien
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

import os
import rclpy
import sys

from betfsm import (
    Sequence, Message, get_logger, set_logger
)
from betfsm_crospi import load_task_list, CrospiTask
from betfsm_ros import BeTFSMNode, ROSRunner, ServiceClient
# Breakpoint of the Crospi Playground: uncomment to pause the playground in a state, e.g.
#   PlaygroundBreakpoint("check pose", message="Check the pose of the tool, then Resume")
# (does nothing outside the playground)
# from crospi_playground.betfsm_debug import PlaygroundBreakpoint
from std_srvs.srv import Trigger
from enum import StrEnum

class SkillMode(StrEnum):
    FOLLOW_INNER_CONTOUR = "follow_inner_contour"
    FOLLOW_OUTER_CONTOUR = "follow_outer_contour"


mode = SkillMode.FOLLOW_INNER_CONTOUR #Uncomment to follow inner contour
# mode = SkillMode.FOLLOW_OUTER_CONTOUR #Uncomment to follow outer contour


class ContourFollowingInner(Sequence):
    def __init__(self):
        super().__init__("contour_following_sequence_inner", [
            Message(None, msg="preparing inner contour following"),
            CrospiTask("MovingHome", "MovingHome"),
            ContourFollowingCommon()
        ])

class ContourFollowingOuter(Sequence):
    def __init__(self):
        super().__init__("contour_following_sequence_outer", [
            Message(None, msg="preparing outer contour following"),
            CrospiTask("MovingHome", "MovingHome"),
            CrospiTask("MovingOut", "MovingOut"), 
            ContourFollowingCommon()
        ])

class ContourFollowingCommon(Sequence):
    def __init__(self):
        super().__init__("contour_following_sequence_common", [
            Message(None, msg="start of contour following"),
            ServiceClient("tare F/T sensor", "/mujoco_robotsimulator/tare_ft_sensor", Trigger, []), # tare of the F/T sensor before force control
            CrospiTask("guarded_motion_down", "guarded_motion_down"),
            CrospiTask("move_up", "move_up"),
            CrospiTask("guarded_motion_to_contour", "guarded_motion_to_contour"),
            CrospiTask("contour_following", "contour_following"),
            Message(None, msg="contour following finished"),
        ])

def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("contour_following_example")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("Contour following example started")
    blackboard = {}

    # Task list: by default the .json file with the name of this script; another one (e.g. the task list of another
    # robot) can be given as the first argument: python3 <this script> <task list: a file name or a complete path>
    # It is taken out of sys.argv: the runner of BeTFSM parses the other arguments (its options, e.g. --serve).
    json_name = sys.argv.pop(1) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else os.path.join(os.path.dirname(os.path.abspath(__file__)), os.path.splitext(os.path.basename(__file__))[0] + ".json")
    load_task_list(json_name, blackboard)

    if mode == SkillMode.FOLLOW_INNER_CONTOUR:
        sm = ContourFollowingInner()
    elif mode == SkillMode.FOLLOW_OUTER_CONTOUR:
        sm = ContourFollowingOuter()
    # sm = ContourFollowingInner()
    # sm = ContourFollowingOuter()
    runner = ROSRunner(my_node, sm, blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
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
