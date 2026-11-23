#!/usr/bin/env python3
#
# Markers driven by a skill: visits a list of waypoints with a Cartesian motion and illustrates the plan in the 3D
# view with markers (crospi_playground.playground_markers / betfsm_markers).
#
# BeTFSM blocks used: Sequence, Compute (plan in the blackboard), Repeat (one iteration per waypoint), CrospiTask with
# a parameter callback (target of the motion from the blackboard), Concurrent + TimedWait (a pause with a countdown
# label), TimedRepeat (blinking marker), Message; and the marker states MarkerEdit (one-shot change), WithMarkers
# (markers shown only during a state) and MarkerUpdater (continuous update).
#
# Outside the playground (crospi + ROS 2) the same script publishes the markers to RViz (/visualization_marker); the
# markers linked to eTaSL expressions are drawn at their default (pins: at the waypoints) or, without default (tool
# trail, desired frame), not drawn; this is reported in warnings.

import os
import sys
import time

import rclpy
from betfsm import (
    Compute, Concurrent, Message, Repeat, Sequence, TimedRepeat, TimedWait, get_logger, set_logger,
)
from betfsm_crospi import CrospiTask, load_task_list
from betfsm_ros import BeTFSMNode, ROSRunner

from crospi_playground import playground_markers as mk
from crospi_playground.betfsm_markers import MarkerEdit, MarkerUpdater, WithMarkers
# Breakpoint of the Crospi Playground: uncomment to pause the playground in a state, e.g.
#   PlaygroundBreakpoint("check pose", message="Check the pose of the tool, then Resume")
# (does nothing outside the playground)
# from crospi_playground.betfsm_debug import PlaygroundBreakpoint

HOME_TCP = [-0.174, 0.615, 0.391]  # tool position at the joints of MovingHome (robot base frame) [m]
WAYPOINTS = [                # in the robot base frame [m]
    [-0.10, 0.55, 0.30],
    [-0.25, 0.55, 0.30],
    [-0.25, 0.70, 0.30],
    [-0.10, 0.70, 0.30],
]


# ---------------------------------------------------------------- blackboard helpers
def make_plan(bb):
    """Compute: the plan in the blackboard (bb["plan"]); "at": where the tool is."""
    return {"waypoints": WAYPOINTS, "index": 0, "at": HOME_TCP}


def current_waypoint(bb):
    return bb["plan"]["waypoints"][bb["plan"]["index"]]


def waypoint_parameters(state, bb):
    """CrospiTask callback: parameters of MoveToWaypoint (override the task list): straight line to the waypoint."""
    target, at = current_waypoint(bb), bb["plan"]["at"]
    return {"delta_pos": [target[i] - at[i] for i in range(3)]}


def next_waypoint(bb):
    return {"index": bb["plan"]["index"] + 1, "at": current_waypoint(bb)}


def moving_text(bb):
    return f"moving to waypoint {bb['plan']['index'] + 1}/{len(bb['plan']['waypoints'])}"


def start_pause(bb):
    return {"pause_end": time.monotonic() + PAUSE}


def countdown(bb):
    return f"next lap in {max(0.0, bb['timing']['pause_end'] - time.monotonic()):.1f} s"


def toggle(bb):
    bb["blink"] = not bb.get("blink", True)
    return bb["blink"]


PAUSE = 2.0


# ---------------------------------------------------------------- tree
def visit_waypoint():
    """One waypoint: show it as the target, move to it (target frame shown only during the motion), drop a pin."""
    return Sequence("visit_waypoint", [
        # the markers as JSON (as copied with "Copy JSON" in the Markers tab); the values that change with the skill
        # (from the blackboard) are arguments
        MarkerEdit("show target", json='{"is-sphere":true,"name":"target","radius":0.015,"color":"#e15759","opacity":0.7}',
                   position=current_waypoint),
        WithMarkers("move with markers", CrospiTask("MoveToWaypoint", "MoveToWaypoint", cb=waypoint_parameters), {
            "label": {"text": moving_text},
            # desired pose of the tool along the motion (global "target" of move_cartesianspace_relative.etasl.lua)
            "desired frame": '{"is-frame":true,"name":"desired frame","position":{"etasl":"target"},"size":0.06,'
                             '"visibility":"when_linked","color":"#ffffff","opacity":1}',
        }),
        # pin where the tool is now (frozen eTaSL expression); in RViz: at the waypoint (default of the link)
        MarkerEdit("pin", lambda bb: f"visited {bb['plan']['index'] + 1}", json='{"is-point":true,"size":0.02,"color":"#59a14f","opacity":1}',
                   position=lambda bb: mk.expr("origin(task_frame)", default=current_waypoint(bb)), freeze=True),
        Compute("next waypoint", ["plan"], next_waypoint),
    ])


class MarkersWaypoints(Sequence):
    def __init__(self):
        super().__init__("markers_waypoints", [
            Message(None, msg="markers_waypoints: plan, visit the waypoints, pause, blink"),
            CrospiTask("MovingHome", "MovingHome"),
            Compute("plan", ["plan"], make_plan),
            # the plan: polyline through the waypoints, and the smooth curve (Catmull-Rom) for comparison
            MarkerEdit("draw plan", json='{"is-path":true,"name":"plan","closed":false,"interpolation":"polyline","width":0.003,'
                                         '"show_points":true,"color":"#f28e2b","opacity":0.9}',
                       points=lambda bb: bb["plan"]["waypoints"]),
            # MarkerEdit("draw smooth plan", json='{"is-path":true,"name":"smooth plan","closed":false,"interpolation":"catmull_rom",'
            #                                     '"width":0.0015,"color":"#76b7b2","opacity":0.8}',
            #            points=lambda bb: bb["plan"]["waypoints"]),
            MarkerEdit("label", json='{"is-text":true,"name":"label","position":[-0.175,0.625,0.45],"size":0.03,'
                                     '"color":"#ffffff","opacity":1}',
                       text=f"plan: {len(WAYPOINTS)} waypoints"),
            # path of the tool, from an eTaSL expression of the running task (playground only)
            MarkerEdit("tool trail", json='{"is-trail":true,"name":"tool trail","position":{"etasl":"origin(task_frame)"},'
                                          '"duration":60,"color":"#4e9af1","opacity":1,"visibility":"keep_last"}'),
            Repeat("visit all waypoints", len(WAYPOINTS), visit_waypoint()),
            MarkerEdit("hide target", "target", enabled=False),
            # pause with a countdown: the Concurrent ends when TimedWait succeeds (MarkerUpdater never ends)
            Compute("pause start", ["timing"], start_pause),
            Concurrent("pause", [
                TimedWait("wait", PAUSE),
                MarkerUpdater("countdown", "label", countdown, prop="text", frequency=10),
            ]),
            # blink the plan 6 times (every 0.25 s), then show it again
            TimedRepeat("blink plan", 6, 0.25, MarkerEdit("toggle plan", "plan", enabled=toggle)),
            MarkerEdit("plan visible", "plan", enabled=True),
            MarkerEdit("done", "label", text=f"done: {len(WAYPOINTS)} waypoints visited", color="#59a14f"),
            CrospiTask("MovingHome", "MovingHome"),
        ])


def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("markers_waypoints")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("markers_waypoints started")
    mk.init(my_node)  # outside the playground: RViz markers published by this node (does nothing in the playground)
    blackboard = {}
    # Task list: by default the .json file with the name of this script; another one (e.g. the task list of another
    # robot) can be given as the first argument: python3 <this script> <task list: a file name or a complete path>
    # It is taken out of sys.argv: the runner of BeTFSM parses the other arguments (its options, e.g. --serve).
    json_name = sys.argv.pop(1) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else os.path.splitext(os.path.basename(__file__))[0] + ".json"
    load_task_list(json_name, blackboard)
    runner = ROSRunner(my_node, MarkersWaypoints(), blackboard, frequency=100.0, publish_frequency=5.0, debug=False,
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
