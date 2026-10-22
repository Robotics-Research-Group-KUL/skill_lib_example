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
# Designed for the MuJoCo simulator with the circular contour
# ($[mujoco_robotsimulator_crospi]/config/setups/ur10e_contour_following_circular.setup.json).
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
from betfsm_ros import BeTFSMNode, ROSRunner

class MyTree(Sequence):
    def __init__(self):
        super().__init__("contour_following_sequence", [
            Message(None, msg="start of contour following"),
            CrospiTask("MovingHome", "MovingHome"),
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

    # Derive the JSON filename from the Python file, assuming they are both located in the same directory and have the same base name.
    json_name = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.path.splitext(os.path.basename(__file__))[0] + ".json")
    load_task_list(json_name, blackboard)

    sm = MyTree()
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
