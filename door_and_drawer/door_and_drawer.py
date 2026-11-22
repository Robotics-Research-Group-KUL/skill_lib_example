#!/usr/bin/env python3
#
# Opens a drawer and a door of a cabinet with the gripper of a Franka Emika Panda (setup franka_panda_door_and_drawer.setup.json,
# environment door_and_drawer/cabinet_with_door_and_drawer.xml). The handle of each is a cube of 5 cm on a short stem.
# For each of them (TARGETS below, in this order):
#
#   approach:  home pose, a pose 10 cm in front of the handle (DrawerPregrasp, DoorPregrasp: the approximately known position
#              of the handle, on purpose about 1 cm off), then force control (ApproachHandle = move_compliant_along_axis):
#              the gripper moves slowly along its own axis, compliant sideways, until the hand touches the cube (3 N); the
#              contact is the reference for the depth: BackOff moves back by 1.2 cm;
#   grasp:     the gripper closes on the handle (CANCEL if nothing is between the fingers: the skill ends with CANCEL);
#   open:      force control (PullDrawer, PullDoor = move_compliant_along_axis): the gripper pulls horizontally with 12 N
#              along the axis that is horizontal (tf_2_insframe of the task list: the gripper is tilted 60 degrees below the
#              horizontal), compliant in the other directions and with the torques controlled to zero: a drawer moves on a
#              straight line, a door on a circular arc (the tool rotates with the handle). The task finishes when the
#              distance travelled along the axis, computed by an integrator (a feature variable and a constraint), reaches
#              "distance" of PullDrawer / PullDoor in the task list (0.20 m for the drawer, 0.27 m (path length, 0.9 rad) for the door);
#   release:   the gripper opens, moves back 12 cm along its axis, and to a safe pose above the robot (SafePose: the direct
#              way home would sweep the hand through the open door); the skill ends at home.
#
# Only for the simulation: the BeTFSM states of fake_gripper_control.py command the simulated gripper of the Crospi Playground
# ("gripper" of the setup) over ROS 2 topics of the simulator; for a real gripper, states for its driver replace them.

import os
import sys

import rclpy
from betfsm import Message, Sequence, get_logger, set_logger
from betfsm_crospi import CrospiTask, load_task_list
from betfsm_ros import BeTFSMNode, ROSRunner, ServiceClient
# Breakpoint of the Crospi Playground: uncomment to pause the playground in a state, e.g.
#   PlaygroundBreakpoint("check pose", message="Check the pose of the tool, then Resume")
# (does nothing outside the playground)
# from crospi_playground.betfsm_debug import PlaygroundBreakpoint
from std_srvs.srv import Trigger

from fake_gripper_control import GripperClose, GripperOpen

#TARGETS = ("drawer", "door")   # what to open, in this order: "drawer", "door" or both
TARGETS = ("door")   # what to open, in this order: "drawer", "door" or both
GRASP_FORCE = 30.0             # per finger [N]


def open_target(name, pregrasp, pull):
    """Sequence: approach the handle (force based), grasp it and open the drawer or door (force based), release."""
    return Sequence(f"open the {name}", [
        Message(None, msg=f"door_and_drawer: open the {name}"),
        CrospiTask("MovingHome", "MovingHome"),
        CrospiTask(f"in front of the {name} handle", pregrasp),
        ServiceClient("tare F/T sensor", "/mujoco_robotsimulator/tare_ft_sensor", Trigger, []),  # force control: the sensor starts at zero
        CrospiTask(f"approach the {name} handle (until contact)", "ApproachHandle"),
        CrospiTask("back off from the contact", "BackOff"),
        GripperClose("close gripper", force=GRASP_FORCE),   # SUCCEED: a handle is between the fingers
        CrospiTask(f"pull the {name} open (force control, torques 0)", pull),
        GripperOpen("release the handle"),
        CrospiTask("retreat", "RetreatFromHandle"),
        # a joint space motion to a pose above the robot (SafePose): the direct way home passes through the open door
        CrospiTask("to a safe pose (clear of the cabinet)", "SafePose"),
    ])


class DoorAndDrawer(Sequence):
    def __init__(self):
        steps = [
            Message(None, msg="door_and_drawer: open " + " and ".join(TARGETS)),
            GripperOpen("open gripper"),
        ]
        if "drawer" in TARGETS:
            steps.append(open_target("drawer", "DrawerPregrasp", "PullDrawer"))
        if "door" in TARGETS:
            steps.append(open_target("door", "DoorPregrasp", "PullDoor"))
        steps.append(CrospiTask("MovingHome at the end", "MovingHome"))
        super().__init__("door_and_drawer", steps)


def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("door_and_drawer")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("door_and_drawer started")
    blackboard = {}
    json_name = os.path.splitext(os.path.basename(__file__))[0] + ".json"
    load_task_list(json_name, blackboard)
    runner = ROSRunner(my_node, DoorAndDrawer(), blackboard, frequency=100.0, publish_frequency=5.0, debug=False,
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
