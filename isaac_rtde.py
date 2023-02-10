# MIT License

# Copyright (c) 2023 Fravebot

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Author: Martin Juříček 

# Isaac Sim app library
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

# Isaac Sim extenstions + core libraries
from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import cuboid
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.motion_generation.interface_config_loader import (
    load_supported_motion_policy_config,
)

# ur rtde communication
import rtde_control
import rtde_receive

import numpy as np
import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument(
    "--robot-ip",
    type=str,
    default="127.0.0.1",
    help="IP adress of robot Real world UR Polyscope or VM UR Polyscope",
)
arg = parser.parse_args()

# set up paths and prims
robot_name = "UR5e"
prim_path = "/UR5e"
usd_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"

# set references to staget in isaac
add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

# add world
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# add robot to world
robot = my_world.scene.add(Robot(prim_path=prim_path, name=robot_name))

# The load_supported_motion_policy_config() function is currently the simplest way to load supported robots.
# In the future, Isaac Sim will provide a centralized registry of robots with Lula robot description files
# and RMP configuration files stored alongside the robot USD.
rmp_config = load_supported_motion_policy_config(robot_name, "RMPflow")

# Initialize an RmpFlow object and set up
rmpflow = RmpFlow(**rmp_config)
physics_dt = 1.0/60
articulation_rmpflow = ArticulationMotionPolicy(robot, rmpflow, physics_dt)
articulation_controller = robot.get_articulation_controller()

# Make a target to follow
target_cube = cuboid.VisualCuboid(
    "/World/target", position=np.array([0.5, 0, 0.5]), color=np.array([1.0, 0, 0]), size=0.1, scale=np.array([0.5,0.5,0.5])
)

# Make an obstacle to avoid
ground = cuboid.VisualCuboid(
    "/World/ground", position=np.array([0.0, 0, -0.0525]), color=np.array([0, 1.0, 0]), size=0.1, scale=np.array([40,40,1])
)
rmpflow.add_obstacle(ground)

# prereset world
my_world.reset()

# IP adress of robot Real world UR Polyscope or VM UR Polyscope
try:
    rtde_r = rtde_receive.RTDEReceiveInterface(arg.robot_ip)
    rtde_c = rtde_control.RTDEControlInterface(arg.robot_ip)
    robot.set_joint_positions(np.array(rtde_r.getActualQ()))

except:
    print("[ERROR] Robot is not connected")
    # close isaac sim
    simulation_app.close()
    sys.exit()

while simulation_app.is_running():
    # on step render
    my_world.step(render=True)
    if my_world.is_playing():
        # first frame -> reset world
        if my_world.current_time_step_index == 0:
            my_world.reset()

        # set target to RMP Flow
        rmpflow.set_end_effector_target(
            target_position=target_cube.get_world_pose()[0], target_orientation=target_cube.get_world_pose()[1]
        )
        
        # Parameters
        velocity = 0.1
        acceleration = 0.1
        dt = 1.0/500  # 2ms
        lookahead_time = 0.1
        gain = 300
        
        # jointq = get joints positions
        joint_q = robot.get_joint_positions()

        # time start period
        t_start = rtde_c.initPeriod()

        # run servoJ
        rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
        rtde_c.waitPeriod(t_start)
        
        # Query the current obstacle position
        rmpflow.update_world()
        actions = articulation_rmpflow.get_next_articulation_action()
        articulation_controller.apply_action(actions)
        
        # get actual q from robot and update isaac model
        robot.set_joint_positions(np.array(rtde_r.getActualQ()))

# rtde control stop script and disconnect
rtde_c.servoStop()
rtde_c.stopScript()
rtde_r.disconnect()

# close isaac sim
simulation_app.close()