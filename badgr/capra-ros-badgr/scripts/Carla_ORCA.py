#!/usr/bin/env python

from cgi import test
from cgitb import handler
import glob
import os
import sys

try:
    sys.path.append(
        glob.glob(
            "../carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass

import faulthandler
import carla
from Carla_Run_Herdr import HERDRenv
from pyorca import ORCAAgent, orca
import time
import random
import numpy as np
from matplotlib import pyplot as plt
from pathlib import Path


def spawn():
    client = carla.Client("localhost", 2000)
    client.set_timeout(15.0)
    world = client.get_world()
    client.reload_world()
    world.wait_for_tick()
    new_settings = world.get_settings()
    new_settings.synchronous_mode = True
    new_settings.max_substeps = 16
    new_settings.max_substep_delta_time = 0.0125
    new_settings.fixed_delta_seconds = 1 / 5
    world.apply_settings(new_settings)

    blueprint_library = world.get_blueprint_library()
    walker_bp = random.choice(blueprint_library.filter("walker.pedestrian.*"))
    num_peds = 200
    actor_list = []
    orca_actor_list = []
    for i in range(num_peds):
        world.tick()
        walker_bp = random.choice(blueprint_library.filter("walker.pedestrian.*"))
        trans = carla.Transform()
        trans.location = world.get_random_location_from_navigation()
        trans.location.z += 1

        walker_actor = world.try_spawn_actor(walker_bp, trans)
        world.tick()
        if walker_actor is None:
            continue
        # print(trans.location)
        actor_list.append(walker_actor)
        orca_actor_list.append(
            ORCAAgent(walker_actor, world.get_random_location_from_navigation())
        )
    return actor_list, orca_actor_list, client, world


def update_orcas(orca_list, tau, dt):
    for i, agent in enumerate(orca_list):
        agent.update()
        candidates = orca_list[:i] + orca_list[i + 1 :]
        new_vels, _ = orca(agent, candidates, tau, dt)
        mag_new_vels = np.linalg.norm(new_vels)
        new_vels = new_vels / mag_new_vels
        dir = carla.Vector3D(*new_vels)
        spd = mag_new_vels
        actor_list[i].apply_control(carla.WalkerControl(direction=dir, speed=spd))


def run(actor_list, orca_actor_list, world, tau=1, dt=1 / 5):
    ego_agent = orca_actor_list[-1]
    orca_actor_list = orca_actor_list[:-1]
    ego_vel = []
    ego_heading = []
    start_time = time.time()
    while (time.time() - start_time) < 2:
        update_orcas(orca_actor_list, tau, dt)

        ego_agent.update()
        ego_vel.append(np.linalg.norm(ego_agent.velocity))
        ego_heading.append(ego_agent.carla_agent.get_transform().rotation.yaw)
        new_vels, _ = orca(ego_agent, orca_actor_list, tau, dt)
        mag_new_vels = np.linalg.norm(new_vels)
        new_vels = new_vels / mag_new_vels
        dir = carla.Vector3D(*new_vels)
        spd = mag_new_vels
        actor_list[-1].apply_control(carla.WalkerControl(direction=dir, speed=spd))
        world.tick()
        if ego_agent.check_goal():
            break
    plot(ego_vel, ego_heading)


def plot(velocities, headings):
    dir_name = Path(Path.cwd())
    dir_name = str(dir_name) + "/ORCA"
    fig = plt.figure(figsize=(16, 9), dpi=80)
    plt.plot(velocities)
    plt.savefig(f"{dir_name}/OCRA_agent_velocity_time.jpg")
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Time (sec)")
    plt.title(f"ORCA Pedestrain Velocity through Time")
    plt.clf()
    plt.plot(headings)
    plt.xlabel("Heading (°)")
    plt.ylabel("Time (sec)")
    plt.title(f"ORCA Pedestrain Heading through Time")
    plt.savefig(f"{dir_name}//OCRA_agent_heading_time.jpg")


if __name__ == "__main__":
    try:
        actor_list, orca_actor_list, client, world = spawn()
        run(actor_list, orca_actor_list, world)
    finally:
        print("Destroying Agent")
        client.apply_batch([carla.command.DestroyActor(x.id) for x in actor_list])
        world.tick()
