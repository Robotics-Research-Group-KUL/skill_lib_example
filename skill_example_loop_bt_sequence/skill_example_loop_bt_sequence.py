#!/usr/bin/env python3

#
# using a Sequence (stops after one time going through the sequence)
#
# Also illustrates proper shutdown
#
# If control-C is pressed, the currently running crospi task will continue to run
#


#  Copyright (c) 2025 KU Leuven, Belgium
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
    Sequence, Repeat,  Message, SUCCEED, TICKING, CANCEL, ABORT, get_logger,set_logger
)
from betfsm_crospi import load_task_list, CrospiTask
from betfsm_ros import BeTFSMNode,ROSRunner
# Breakpoint of the Crospi Playground: uncomment to pause the playground in a state, e.g.
#   PlaygroundBreakpoint("check pose", message="Check the pose of the tool, then Resume")
# (does nothing outside the playground)
# from crospi_playground.betfsm_debug import PlaygroundBreakpoint

class MyTree(Repeat):
    def __init__(self):
        sequence = Sequence("my_sequence", [
            Message(None,msg="start of a new loop"),
            CrospiTask("MovingHome","MovingHome"),
            CrospiTask("MovingDown","MovingDown"),
            CrospiTask("MovingUp","MovingUp"),
            CrospiTask("MovingSpline","MovingSpline")
        ])
        super().__init__("my_tree",-1,sequence) # -1 for infinite loop, 1 for one time through the sequence, 2 for two times, etc.


def main(args=None):
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("skill_example")
    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())
    get_logger().info("skill_example_2 started")
    blackboard = {}

    #Use the following to load the task list relative to a ROS2 package, e.g. crospi_application_template
    # load_task_list("$[crospi_application_template]/skill_specifications/libraries/skill_lib_example/skill_example_loop_fsm/skill_example_loop_fsm.json",blackboard)
    
    # Task list: by default the .json file with the name of this script; another one (e.g. the task list of another
    # robot) can be given as the first argument: python3 <this script> <task list: a file name or a complete path>
    # It is taken out of sys.argv: the runner of BeTFSM parses the other arguments (its options, e.g. --serve).
    json_name = sys.argv.pop(1) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else os.path.splitext(os.path.basename(__file__))[0] + ".json"
    load_task_list(json_name, blackboard)

    sm = MyTree()
    runner = ROSRunner(my_node,sm,blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
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
