#!/usr/bin/env python3
#
# Markers driven by a skill: localizes the inside corner on the table with three guarded motions (down, +x, +y) and
# illustrates it in the 3D view: the measured contact force as an arrow during each guarded motion, a pin at each
# contact, the corner estimated from the contacts, and the ground truth of the scene for comparison.
#
# BeTFSM blocks used: Sequence, Fallback (no contact with the table: back home and give up), While (time budget of a
# guarded motion), Adapt (a wall that is not found is not an error), AlwaysOutcome, Concurrent (motion + force
# arrow), ConcurrentSequence (report while moving home), Compute, LogBlackboard, Message; a ConfigCallback reading the
# forces published by crospi (/forces_etasl); and the marker states MarkerEdit, MarkerUpdater and WithMarkers.
#
# Outside the playground (crospi + ROS 2) the same script publishes the markers to RViz; the markers linked to eTaSL
# expressions (force arrow position, contact pins) cannot be drawn there and are reported in warnings.

import math
import os
import sys
import time

import rclpy
from betfsm import (
    CANCEL, SUCCEED, Adapt, AlwaysOutcome, Compute, Concurrent, ConcurrentSequence, ConfigCallback, Fallback,
    LogBlackboard, Message, Sequence, While, get_logger, set_logger,
)
from betfsm_crospi import CrospiTask, load_task_list
from betfsm_ros import BeTFSMNode, ROSRunner, ServiceClient
from std_srvs.srv import Trigger
from crospi_interfaces.msg import Output

from crospi_playground import playground_markers as mk
from crospi_playground.betfsm_markers import MarkerEdit, MarkerUpdater, WithMarkers
# Breakpoint of the Crospi Playground: uncomment to pause the playground in a state, e.g.
#   PlaygroundBreakpoint("check pose", message="Check the pose of the tool, then Resume")
# (does nothing outside the playground)
# from crospi_playground.betfsm_debug import PlaygroundBreakpoint

TOOL_RADIUS = 0.015       # cylindrical tool (ur10e_cylinder_tool.urdf): 3 cm diameter


class ForceCallback(ConfigCallback):
    """Latest contact force [fx, fy, fz] published by crospi on /forces_etasl (TopicOutputHandler of the setup)."""

    def __init__(self, topic="/forces_etasl"):
        super().__init__()
        self.topic = topic
        self.sub = None
        self.force = None

    def _cb(self, msg):
        values = dict(zip(msg.names, msg.data))
        if all(k in values for k in ("x_force", "y_force", "z_force")):
            self.force = [values["x_force"], values["y_force"], values["z_force"]]

    def reset(self, state, blackboard):
        if self.sub is None:
            self.sub = BeTFSMNode.get_instance().create_subscription(Output, self.topic, self._cb, 10)
        self.force = None
        return True

    def __call__(self, state, blackboard):
        return self.force  # None until the first message: the marker is not updated


def label(text, color="#ffffff"):
    return MarkerEdit(f"label: {text}", "label", text=text, color=color)


def guarded(axis, budget):
    """
    Guarded motion along `axis` with the force arrow at the tool; CANCEL if there is no contact within `budget`
    seconds (While); a pin where the contact is.
    """
    return Sequence(f"probe {axis}", [
        label(f"guarded motion along {axis}: waiting for contact"),
        Compute(f"start {axis}", ["timing"], lambda bb: {"start": time.monotonic()}),
        While(f"{axis} within {budget} s", lambda bb: time.monotonic() - bb["timing"]["start"] < budget,
              WithMarkers(f"guarded {axis} with force arrow",
                          Concurrent(f"guarded {axis} and force", [
                              CrospiTask(f"guarded_motion_{axis}", f"guarded_motion_{axis}"),
                              MarkerUpdater("force", "force", ForceCallback(), prop="vector", frequency=20),
                          ]),
                          {"force": '{"is-arrow":true,"name":"force","position":{"etasl":"origin(task_frame)"},'
                                    '"vector":[0,0,0],"scale":0.02,"color":"#edc948","opacity":1}'})),
        # pin at the contact: the tool position now (eTaSL expression of the last task), frozen
        MarkerEdit(f"pin {axis}", f"contact {axis}", freeze=True,
                   json='{"is-point":true,"position":{"etasl":"origin(task_frame)"},"size":0.012,"color":"#e15759","opacity":1}'),
    ])


def contact(axis):
    p = (mk.get(f"contact {axis}") or {}).get("position")
    return p if isinstance(p, list) else None  # a link (outside the playground): unknown


def estimate_corner(bb):
    """Compute: corner from the contacts (tool tip on the table, tool side against the walls) and its marker."""
    cz, cx, cy = contact("z"), contact("x"), contact("y")
    if not (cz and cx and cy):
        return {"estimate": None}
    est = [cx[0] + TOOL_RADIUS, cy[1] + TOOL_RADIUS, cz[2]]
    truth = [0.30, -0.50, 0.15]  # site "corner" of the scene (robot base = world in this setup)
    err = 1000 * math.dist(est, truth)
    mk.add("estimated corner", "sphere", position=est, radius=0.008, color="#59a14f", opacity=0.9)
    mk.update("label", text=f"corner estimated {err:.1f} mm from the ground truth", color="#59a14f")
    return {"estimate": est, "error_mm": err}


class MarkersCorner(Sequence):
    def __init__(self):
        no_table = Sequence("no table: give up", [
            label("no contact with the table: back home", "#e15759"),
            CrospiTask("MovingHome after failure", "MovingHome"),
            AlwaysOutcome("give up", CANCEL),
        ])
        super().__init__("markers_corner", [
            Message(None, msg="markers_corner: localize the corner on the table with guarded motions"),
            # the markers as JSON (as copied with "Copy JSON" in the Markers tab)
            MarkerEdit("label", json='{"is-text":true,"name":"label","position":[0.15,-0.65,0.45],"text":"moving to the table",'
                                     '"size":0.03,"reference":"world","color":"#ffffff","opacity":1}'),
            # ground truth: the site "corner" is at (0.15, 0.15, 0) in the body "corner" of the scene
            MarkerEdit("ground truth", json='{"is-frame":true,"name":"corner (ground truth)","reference":"corner",'
                                            '"position":[0.15,0.15,0],"size":0.05,"opacity":1}'),
            CrospiTask("MovingHome", "MovingHome"),
            ServiceClient("tare F/T sensor", "/mujoco_robotsimulator/tare_ft_sensor", Trigger, []),  # tare of the F/T sensor before force control
            CrospiTask("approach_table_cartesian", "approach_table_cartesian"),
            # the table must be found (else: home, CANCEL); the walls are optional (CANCEL → SUCCEED)
            Fallback("find the table", [guarded("z", 15.0), no_table]),
            Adapt("wall x (optional)", guarded("x", 15.0), {CANCEL: SUCCEED}),
            Adapt("wall y (optional)", guarded("y", 15.0), {CANCEL: SUCCEED}),
            # report while moving home: both run together, ConcurrentSequence waits for both
            ConcurrentSequence("report and go home", [
                Sequence("report", [
                    Compute("estimate corner", ["corner"], estimate_corner),
                    LogBlackboard("log estimate", ["corner"]),
                ]),
                CrospiTask("MovingHome at the end", "MovingHome"),
            ]),
        ])


def main(args=None):
    rclpy.init(args=args)
    my_node = BeTFSMNode.get_instance("markers_corner")
    set_logger("default", my_node.get_logger())
    set_logger("crospi", my_node.get_logger())
    get_logger().info("markers_corner started")
    mk.init(my_node)  # outside the playground: RViz markers published by this node (does nothing in the playground)
    blackboard = {}
    # Task list: by default the .json file with the name of this script; another one (e.g. the task list of another
    # robot) can be given as the first argument: python3 <this script> <task list: a file name or a complete path>
    # It is taken out of sys.argv: the runner of BeTFSM parses the other arguments (its options, e.g. --serve).
    json_name = sys.argv.pop(1) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else os.path.splitext(os.path.basename(__file__))[0] + ".json"
    load_task_list(json_name, blackboard)
    runner = ROSRunner(my_node, MarkersCorner(), blackboard, frequency=100.0, publish_frequency=5.0, debug=False,
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
