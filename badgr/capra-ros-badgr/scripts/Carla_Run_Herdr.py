#!/usr/bin/env python

import glob
import os
import sys
from cgi import test
from cgitb import handler

# from memory_profiler import profile

try:
    sys.path.append(
        glob.glob(
            "/opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                7,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass

import csv
import faulthandler
import gc
import pickle
import random
import time
from datetime import datetime
from multiprocessing import Process, Queue
from pathlib import Path

import rospy
import carla
import h5py
import matplotlib as mpl
import matplotlib.animation as animation
import numpy as np
import torch
from matplotlib import pyplot as plt
from torch import nn
from torch.utils.data.sampler import SubsetRandomSampler
from torch.utils.tensorboard import SummaryWriter

from actionplanner import HERDRPlan
from Badgrnet import HERDR, HERDR_Resnet
from Carla_Trainer import carla_hdf5dataclass

# from Herdr_agent import Herdragent
from ros_dummy_agent import ros_agent
from metrics_utils import (
    plot_action_cam_view,
    plot_actions,
    plot_heatmap,
    plot_trajectory,
)
from Ped_agent import Pedagent
from pyorca import ORCAAgent, orca


def location2tensor(location):
    ls = [location.x, location.y, location.z]
    return torch.tensor(ls)


def EucDistance(x, y):
    if type(x) == carla.libcarla.Location:
        x = location2tensor(x)
    if type(y) == carla.libcarla.Location:
        y = location2tensor(y)
    if type(x) == type(np.array([])):
        x = torch.tensor(x)
    if type(y) == type(np.array([])):
        y = torch.tensor(y)
    dist = torch.norm(x[0:2] - y[0:2])
    return dist


class HERDRenv:
    SPL_hist = []
    control_freq = 5  # Hz
    run = 0
    client = carla.Client("localhost", 2000)
    client.set_timeout(8.0)
    world = client.get_world()
    new_settings = world.get_settings()
    new_settings.synchronous_mode = True
    new_settings.max_substeps = 16
    new_settings.max_substep_delta_time = 0.0125
    new_settings.fixed_delta_seconds = 1 / control_freq
    blueprint_library = world.get_blueprint_library()
    preset_list = [
        item for item in dir(carla.WeatherParameters)[0:22] if "Night" not in item
    ]
    dict_WP = carla.WeatherParameters.__dict__

    def __init__(self, training=False, test_block=None):
        self.actor_list = []
        self.controller_list = []
        self.collision_hist = []
        self.pos_hist = []
        self.ped_space_count = 0
        self.dist2peds_list = []
        self.orca_actor_list = []
        self.H5File_name = None
        self.enable_settings()
        if test_block != None:
            self.load_spawns(f"./spawn_locations_test{test_block}.txt")

    def __del__(self):
        print("Env deleted")

    def enable_settings(self):
        self.world.apply_settings(self.new_settings)

    def reworld(self):
        self.client.reload_world()
        self.world.wait_for_tick()

    def pop_map_pedestrians(self, num_peds=200, test_block=None):
        # self.ped_actor_start_idx = len(self.actor_list)-1
        controller_bp = self.blueprint_library.find("controller.ai.walker")

        for i in range(num_peds):
            walker_bp = random.choice(
                self.blueprint_library.filter("walker.pedestrian.*")
            )
            if test_block is None:
                trans = carla.Transform()
                trans.location = self.world.get_random_location_from_navigation()
                trans.location.z += 1
            else:
                trans = self.get_spawn()

            walker_actor = self.world.try_spawn_actor(walker_bp, trans)

            self.world.tick()
            if walker_actor is None:
                continue
            self.actor_list.append(walker_actor)

            controller_walker = self.world.try_spawn_actor(
                controller_bp, carla.Transform(), walker_actor
            )

            self.world.tick()
            controller_walker.start()
            if test_block is None:
                controller_walker.go_to_location(
                    self.world.get_random_location_from_navigation()
                )
            else:
                controller_walker.go_to_location(self.get_spawn().location)

            self.controller_list.append(controller_walker)
            self.world.tick()
        # del self.blueprint_library
        print(f"{len(self.actor_list)}/{num_peds} Pedestrians Added.")
        return len(self.actor_list)

    def set_weather(self):
        weather = self.preset_list[random.randint(0, len(self.preset_list) - 1)]
        self.world.set_weather(self.dict_WP[weather])
        return weather

    def reset(self):
        # self.client = carla.Client('localhost', 2000)
        # self.client.set_timeout(8.0)
        self.world = self.client.get_world()
        new_settings = self.world.get_settings()
        new_settings.synchronous_mode = True
        new_settings.max_substeps = 16
        new_settings.max_substep_delta_time = 0.0125
        new_settings.fixed_delta_seconds = 1 / self.control_freq
        self.ped_space_count = 0
        # self.wmap = self.world.get_map()
        # self.set_weather()
        self.world.apply_settings(new_settings)

    def pathlength(self, pos_list):
        np_pos_hist = np.asarray(pos_list)
        x, y = np_pos_hist[:, 0], np_pos_hist[:, 1]
        n = len(x)
        lv = [
            np.sqrt((x[i] - x[i - 1]) ** 2 + (y[i] - y[i - 1]) ** 2)
            for i in range(1, n)
        ]
        # L = sum(lv)
        L = 0
        for item in lv:
            if item >= 1.0:
                continue
            L += item
        return L

    def dist2peds(self, agent):
        agent_location = agent.vehicle.get_location()
        dist_list = []
        for i, ped in enumerate(self.actor_list):
            if i == len(self.actor_list) - 1:
                continue
            ped_location = ped.get_location()
            dist_list.append(EucDistance(ped_location, agent_location))
        dist_list = np.asarray(dist_list)
        # print(dist_list)
        min_dist = dist_list.min()
        avg_dist = dist_list.mean()
        max_dist = dist_list.max()
        # return min_dist
        self.dist2peds_list.append(min_dist)

    def calc_SPL(self):
        sum_var = 0
        for success, length, p2pdist in self.SPL_hist:
            sum_var += success * (p2pdist / np.max([p2pdist, length, 1e-9]))
        spl = 1 / len(self.SPL_hist) * sum_var
        print(f"\n&& SPL: {spl:.4f} &&\n")
        return spl

    def set_recordings(self, log_name):
        plt.clf()
        fig = plt.figure(figsize=(16, 9), dpi=80)
        cmap = mpl.cm.YlOrRd
        norm = mpl.colors.Normalize(vmin=0, vmax=1)
        cb = plt.colorbar(
            mpl.cm.ScalarMappable(norm=norm, cmap=cmap), label="Probability"
        )
        fps = 8
        self.writer = animation.FFMpegWriter(fps=fps)
        p = Path(f"/home/nathan/HERDR/Carla_Results/{log_name[0:5]}")
        if not p.is_dir():
            os.mkdir(f"/home/nathan/HERDR/Carla_Results/{log_name[0:5]}")
        self.writer.setup(
            fig,
            f"/home/nathan/HERDR/Carla_Results/{log_name[0:5]}/{log_name[7:]}_actions_cam_view.mp4",
        )
        self.top_writer = animation.FFMpegWriter(fps=fps)
        self.top_writer.setup(
            fig,
            f"/home/nathan/HERDR/Carla_Results/{log_name[0:5]}/{log_name[7:]}_Top_view.mp4",
        )
        self.plot_hist_front = Queue()
        self.plot_hist_top = Queue()

    def get_recordings(self, agent):
        self.plot_hist_front.put(
            [
                location2tensor(agent.vehicle.get_location()),
                agent.frame.permute(1, 2, 0),
                agent.event,
                agent.state,
                agent.planner.mean,
            ]
        )
        td_cam_pos = agent.vehicle.get_location()
        topdown_camera_transform = carla.Transform(
            carla.Location(x=td_cam_pos.x, y=td_cam_pos.y, z=10),
            carla.Rotation(pitch=-90),
        )
        agent.tdcam.set_transform(topdown_camera_transform)
        self.plot_hist_top.put(
            [
                agent.state,
                agent.event,
                location2tensor(topdown_camera_transform.location).numpy(),
                agent.GOAL[0, 0, :],
                agent.topview,
            ]
        )
        return

    def background_save(self, Q1, Q2):
        count = 0
        while True:
            if Q1.empty() | Q2.empty():
                time.sleep(1)
                count += 1
                if count == 10:
                    print("Saving timed out")
                    return
            else:
                count = 0
                front = Q1.get()
                if "done" in front:
                    print("Saving Done")
                    return
                plot_action_cam_view(*front)
                self.writer.grab_frame()
                top = Q2.get()
                plot_actions(*top)
                self.top_writer.grab_frame()

    def new_run(self, action_hist, im_hist, GND_hist, folder):
        dir_name = Path(Path.cwd())
        p = Path(f"{dir_name}/old_carla_hdf5s/{folder}")
        if not p.is_dir():
            os.mkdir(f"{dir_name}/old_carla_hdf5s/{folder}")
        self.H5File = h5py.File(
            f"{dir_name}/old_carla_hdf5s/{folder}/{self.H5File_name}.h5", "a"
        )
        self.now = datetime.now()
        group = self.H5File.create_group(f"{self.now}")
        self.d1 = self.H5File.create_dataset(f"{self.now}/actions", data=action_hist)
        self.d2 = self.H5File.create_dataset(f"{self.now}/gnd", data=GND_hist)
        maxlen = len(str(self.now))
        dtipe = "S{0}".format(maxlen)
        self.d3 = self.H5File.create_dataset(
            f"{self.now}/img", data=im_hist, dtype=dtipe
        )

    def cleanup(self):
        print("Destroying Pedestrians")
        self.client.apply_batch_sync(
            [carla.command.DestroyActor(x.id) for x in self.actor_list]
        )
        self.world.tick()
        print("Destroying AI Controllers")
        [x.stop() for x in self.controller_list]
        self.client.apply_batch_sync(
            [carla.command.DestroyActor(x.id) for x in self.controller_list]
        )
        self.world.tick()
        print("Done.")
        self.actor_list.clear()
        self.orca_actor_list.clear()
        self.controller_list.clear()
        self.dist2peds_list.clear()
        self.SPL_hist.clear()
        self.world.tick()
        self.new_settings.synchronous_mode = False
        self.enable_settings()
        self.world.tick()

    def load_spawns(self, file):
        self.spawn_locations = []
        with open(file, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                self.spawn_locations.append(row)
            self.spawn_locations = np.asarray(self.spawn_locations).astype(np.int16)

    def get_spawn(self):
        rot = float(np.random.uniform(0, 360, 1))
        loc = np.random.choice(np.arange(0, self.spawn_locations.shape[0] - 1))
        return carla.Transform(
            carla.Location(
                x=float(self.spawn_locations[loc, 0]),
                y=float(self.spawn_locations[loc, 1]),
                z=0.177637,
            ),
            carla.Rotation(yaw=rot),
        )

    def get_spawn_ORCA(self, side=0, num_spots=20):
        if side == 1:
            x = self.spawn_locations[:, 0].max()
            y = np.random.choice(
                np.linspace(
                    self.spawn_locations[:, 1].max(),
                    self.spawn_locations[:, 1].min(),
                    num_spots,
                )
            )
            return carla.Transform(
                carla.Location(x=float(x), y=float(y), z=0.177637),
                carla.Rotation(yaw=180),
            )
        x = self.spawn_locations[:, 0].min()
        y = np.random.choice(
            np.linspace(
                self.spawn_locations[:, 1].max(),
                self.spawn_locations[:, 1].min(),
                num_spots,
            )
        )
        return carla.Transform(
            carla.Location(x=float(x), y=float(y), z=0.177637), carla.Rotation(yaw=0)
        )

    def pop_peds_ORCA(self, num_peds):
        self.load_spawns(
            f"/opt/capra/overlay_ws/src/capra-badgr/capra-ros-badgr/scripts/spawn_locations_Test_Orca2.txt"
        )
        side1_list = []
        side2_list = []
        num_spots = int(
            (self.spawn_locations[:, 1].max() - self.spawn_locations[:, 1].min()) / 0.5
        )
        for i in range(num_peds):
            side1_list.append(self.get_spawn_ORCA(1, num_spots))
            side2_list.append(self.get_spawn_ORCA(2, num_spots))
        side1_list_index = np.arange(len(side1_list))
        np.random.shuffle(side1_list_index)
        side2_list_index = np.arange(len(side1_list))
        np.random.shuffle(side2_list_index)

        s1 = iter(side1_list_index)
        s2 = iter(side2_list_index)
        np.random.shuffle(side1_list_index)
        np.random.shuffle(side2_list_index)
        s1_end = iter(side1_list_index)
        s2_end = iter(side2_list_index)
        for i in range(num_peds):
            if i < num_peds / 2:
                trans = side1_list[next(s1)]
            else:
                trans = side2_list[next(s2)]
            walker_bp = random.choice(
                self.blueprint_library.filter("walker.pedestrian.*")
            )
            walker_actor = self.world.try_spawn_actor(walker_bp, trans)

            self.world.tick()
            if walker_actor is None:
                continue
            self.actor_list.append(walker_actor)
            if i < num_peds / 2:
                goal_location = side2_list[next(s2_end)].location
            else:
                goal_location = side1_list[next(s1_end)].location
            # print(f'Current location is: {trans.location} Goal location is: {goal_location}')
            self.orca_actor_list.append(ORCAAgent(walker_actor, goal_location))


def main(
    env,
    Herdr,
):
    faulthandler.enable()
    torch.set_default_dtype(torch.float32)
    torch.no_grad()
    max_loss = 10000
    try:
        log_time = datetime.now().strftime("%m-%d-%Y--%H-%M")
        recording_data = log_time
        Herdr.create_recording_folder(recording_data)
        env.reset()
        env.pop_map_pedestrians(num_peds=200)
        # writer = SummaryWriter(log_dir=f'/home/nathan/HERDR/carla_logs/{log_time}')
        if os.path.isfile(f"/home/nathan/HERDR/pickles/training_counts_{log_time}.pkl"):
            with open(
                f"/home/nathan/HERDR/pickles/training_counts_{log_time}.pkl", "rb"
            ) as f:
                total_sim_time, env.run, end_step, env.SPL_hist = pickle.load(f)
        else:
            end_step = 0
            total_sim_time = 0
        round = 0

        while True:
            print(f"Round {round+1}!\n")
            if recording_data:
                H5File_name = f"{datetime.now()}"
                env.H5File_name = H5File_name
            try:
                num_epoch = 100
                for i in range(num_epoch):
                    env.set_weather()
                    env.run += 1
                    env.ped_space_count = 0
                    Herdr.reset()
                    print(
                        f'--- START Run {i+1}/Round {round+1} at: {datetime.now().strftime("%m-%d-%Y--%H:%M:%S")} ---'
                    )
                    start_time = time.time()
                    save_frame_count = 0
                    while not Herdr.done | (time.time() - start_time >= 300):
                        Herdr.step()
                        env.world.tick()
                        Herdr.is_safe(env.controller_list)
                        if Herdr.safe == 1:
                            env.ped_space_count += 1
                        Herdr.GND_hist.append(Herdr.safe)
                        save_frame_count += 1
                        if save_frame_count % 100 == 0:
                            print(
                                f"PING - I'm Alive - at {Herdr.vehicle.get_transform().location}"
                            )
                            print(f"Current Speed = {Herdr.vel_hist[-1]:.2f}")
                        if len(Herdr.collision_hist) > 0:
                            if Herdr.GND_hist[-1] == 0:
                                Herdr.GND_hist[-1] = 1
                            break

                    Herdr.done = True
                    print(
                        f'--- DONE Run {i+1}/Round {round+1} at: {datetime.now().strftime("%m-%d-%Y--%H:%M:%S")} ---\n'
                    )
                    sim_time = save_frame_count / 5
                    print(f"Sim Time est: {int(sim_time)} seconds")
                    total_sim_time += int(sim_time)
                    if recording_data:
                        env.new_run(
                            Herdr.action_hist,
                            Herdr.im_hist,
                            Herdr.GND_hist,
                            recording_data,
                        )
                        env.H5File.close()
                    """ Calculate and Save SPL metric"""
                    # pl = env.pathlength(Herdr.pos_hist)
                    # env.SPL_hist.append([Herdr.success, pl, Herdr.p2pdist])
                    # spl = env.calc_SPL()
                    """ Empty memory and kill actors and sensors and save metrics"""
                    # writer.add_scalar("Validation/Run_time", sim_time, env.run)
                    # writer.add_scalar("Validation/In_Pedestrain_Space", env.ped_space_count/sim_time, env.run)
                    # writer.add_scalar("Validation/Distance_Traveled", EucDistance(Herdr.pos_hist[0],Herdr.pos_hist[-1]), env.run)
                    # writer.add_scalar("Validation/SPL", spl, env.run)
                    """ Plot actions and optimal path at collision every 5th run """
                    # if env.run % 5 == 0:
                    #     fig = plt.figure()
                    #     plot_args = [location2tensor(Herdr.vehicle.get_location()), Herdr.frame.permute(1,2,0), Herdr.event, Herdr.state, Herdr.planner.mean]
                    #     plot_action_cam_view(*plot_args)
                    #     fig.canvas.draw()
                    #     # Now we can save it to a numpy array.
                    #     data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
                    #     data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
                    #     data = data.transpose(2,0,1)/255
                    #     try:
                    #         writer.add_image("Validation/Context_Image", data, env.run)
                    #     finally:
                    #         del data, plot_args
                    #         plt.close(fig)
                    """ Clean up from run """
                    # writer.flush()
                    Herdr.cleanup()
                    env.world.tick()
                    print(f"Real time: {time.time()-start_time:.4f}\n")

            finally:
                """Train on collected data"""
                # run_data = carla_hdf5dataclass(f"/home/nathan/HERDR/all_carla_hdf5s/{recording_data}/", Herdr.horizon, imagefile_path=f"/home/nathan/HERDR/carla_images/{recording_data}/", counting=True, load_all_files=True)
                # test_sampler = SubsetRandomSampler(run_data.valid_start_indices)
                # testloader = torch.utils.data.DataLoader(run_data, sampler=test_sampler, batch_size=1)
                # run_data.set_pos_w(count_data_ratio(testloader))
                # testloader = torch.utils.data.DataLoader(run_data, sampler=test_sampler, batch_size=32)
                # opt = torch.optim.Adam(Herdr.model.parameters(), lr=1e-4, weight_decay=1e-2)
                # epoches = 2
                # for epoch in range(epoches):
                #     loss, pos_accuracy, accuracy, end_step = run_data.one_epoch(Herdr.model,testloader, start_step=end_step, writer=writer, opt=opt)
                #     print(f"{epoch+1} - Epoch Loss: {loss:.2f}, Epoch +Accuracy: {pos_accuracy:.2f}, Epoch Accuracy: {accuracy:.2f}, # steps: {len(testloader)}")
                #     writer.flush()
                # if loss < max_loss:
                #     max_loss = loss
                #     torch.save(Herdr.model, f'./models/carla{log_time}.pth')
                # del run_data, test_sampler, testloader, opt
                with open(
                    f"/home/nathan/HERDR/pickles/training_counts_{log_time}.pkl", "wb"
                ) as f:
                    pickle.dump([total_sim_time, env.run, end_step, env.SPL_hist], f)
                """ Finish Run"""
                round += 1
                print(
                    f"Total Sim Time in HRs:{int(total_sim_time/3600)}, in Mins:{int(total_sim_time/60)}"
                )
                env.cleanup()
                # env.reworld()
                # writer.flush()
                env.reset()
                env.pop_map_pedestrians(num_peds=200)
                env.world.tick()
    finally:
        Herdr.cleanup()
        env.world.tick()
        env.cleanup()
        # writer.close()
        del env, Herdr


def test(env, Herdr, blk):
    env.reset()
    # log_time = datetime.now().strftime("%m-%d--%H-%M")
    # log_time = log_time +f'_Block-{blk}-{model_name}'
    try:
        # env.reworld()
        # env.reset()
        num_peds = env.pop_map_pedestrians(num_peds=75, test_block=blk)
        Herdr.reset()
        # env.set_recordings(log_time)
        # job = Process(target=env.background_save, args=(env.plot_hist_front,env.plot_hist_top,), daemon=True)
        # job.start()
        start_time = time.time()
        save_frame_count = 0
        while not Herdr.done:
            Herdr.step()
            # env.get_recordings(Herdr)
            env.world.tick()
            env.dist2peds(Herdr)
            Herdr.is_safe(env.controller_list)
            if Herdr.safe == 1:
                env.ped_space_count += 1
                print(f"In Pedestrain Space #{env.ped_space_count}")
            save_frame_count += 1
            if save_frame_count % 100 == 0:
                print(f"PING - I'm Alive - at {Herdr.vehicle.get_transform().location}")
                print(f"Current Speed = {Herdr.vel_hist[-1]:.2f}")
            # env.pos_hist.append(location2tensor(Herdr.vehicle.get_location()).numpy())
            if len(Herdr.collision_hist) > 0:
                Herdr.collision_locs.append(
                    location2tensor(Herdr.vehicle.get_location())
                )
                break
            if time.time() - start_time >= 300:
                print(f"Timed-out")
                break
            if Herdr.done:
                print(f"Random done flag?")
        Herdr.done = True
        del start_time
        # env.plot_hist_front.put('done')
        # job.join()
        # job.close()
        pl = env.pathlength(Herdr.pos_hist)
        env.SPL_hist.append([Herdr.success, pl, Herdr.p2pdist])
        spl = env.calc_SPL()
        # env.top_writer.finish()
        # env.writer.finish()
        data = {
            "Times in ped space": env.ped_space_count,
            "Path length": pl,
            "Avg min dist to ped": np.asarray(env.dist2peds_list).mean(),
            "Success": Herdr.success,
            "Point2Point Dist": Herdr.p2pdist.item(),
            "Agent goal gain": Herdr.goal_gain,
            "Agent action gain": Herdr.action_gain,
            "# of Peds": num_peds,
            "Sim Time": int(save_frame_count / 5),
        }
        print(data)
        # p = Path(f'/home/nathan/HERDR/Carla_Results/{log_time[0:5]}')
        # if not p.is_dir():
        #     os.mkdir(f'/home/nathan/HERDR/Carla_Results/{log_time[0:5]}')
        # plot_trajectory(np.asarray(env.pos_hist), torch.ones((len(env.pos_hist))), Herdr.GOAL[0,0,:], collision=len(Herdr.collision_hist), traj_length=env.pathlength(Herdr.pos_hist))
        # plt.savefig(f'/home/nathan/HERDR/Carla_Results/{log_time[0:5]}/{log_time[7:]}_trajectory.png')
        # plt.close('all')
        # Herdr.plot()

    finally:
        plot_heatmap(Herdr.collision_locs)
        Herdr.cleanup()
        env.cleanup()
        env.world.tick()
        # del env, Herdr
        # gc.collect()
        return data


def test_ped():
    env = HERDRenv()
    preset_list = dir(carla.WeatherParameters)[0:12]
    dict_WP = carla.WeatherParameters.__dict__
    env.world.set_weather(dict_WP[preset_list[random.randint(0, 11)]])
    PED = Pedagent(test_block=2)
    try:
        # env.reworld()
        # env.reset()
        env.pop_map_pedestrians(num_peds=200)
        PED.reset()
        env.set_recordings()
        job = Process(
            target=env.background_save,
            args=(
                env.plot_hist_front,
                env.plot_hist_top,
            ),
            daemon=True,
        )
        job.start()
        start_time = time.time()
        save_frame_count = 0
        while not PED.done | (time.time() - start_time >= 300):
            PED.step()
            env.world.tick()
            PED.is_safe(env.controller_list)
            if PED.safe == 1:
                env.ped_space_count += 1
            PED.GND_hist.append(PED.safe)
            save_frame_count += 1
            if save_frame_count % 200 == 0:
                loc = carla.Location(
                    x=float(PED.GOAL[0, 0, 0].numpy()),
                    y=float(PED.GOAL[0, 0, 1].numpy()),
                )
                PED.controller_walker.go_to_location(loc)
                print(f"PING - I'm Alive - at {PED.vehicle.get_transform().location}")
            env.pos_hist.append(location2tensor(PED.vehicle.get_location()).numpy())
            env.get_recordings(PED)
            if len(PED.collision_hist) > 0:
                if PED.GND_hist[-1] == 0:
                    PED.GND_hist[-1] = 1
                break
        PED.done = True
        env.plot_hist_front.put("done")
        job.join()
        job.close()
        pl = env.pathlength(PED.pos_hist)
        env.SPL_hist.append([PED.success, pl, PED.p2pdist])
        spl = env.calc_SPL()
        env.top_writer.finish()
        env.writer.finish()
        plot_trajectory(
            np.asarray(env.pos_hist),
            torch.ones((len(env.pos_hist))),
            PED.GOAL[0, 0, :],
            collision=len(PED.collision_hist),
        )
        plt.savefig("./trajectory.png")
        plt.close("all")

    finally:
        PED.cleanup()
        env.cleanup()
        print("Destroying Pedestrians")
        env.client.apply_batch(
            [carla.command.DestroyActor(x.id) for x in env.actor_list]
        )
        print("Destroying AI Controllers")
        [x.stop() for x in env.controller_list]
        env.client.apply_batch(
            [carla.command.DestroyActor(x.id) for x in env.controller_list]
        )
        print("Done.")
        env.world.tick()
        del env, PED


def check_spawn_points():
    client = carla.Client("localhost", 2000)
    world = client.get_world()
    # preset_list = dir(carla.WeatherParameters)[6]
    # dict_WP = carla.WeatherParameters.__dict__
    # world.set_weather(dict_WP[preset_list])
    blueprint_library = world.get_blueprint_library()
    omafiets = blueprint_library.filter("omafiets")[0]
    # transform = carla.Transform(carla.Location(x=114, y=-65, z=0.177637))
    vehicle = None
    start_time = time.time()
    actor_list = []
    # sensor_list = []
    debug = world.debug
    x, y, z = 93.0, -13.0, 0.0
    arrow = carla.Location(x=x, y=y, z=z)
    arrow_top = carla.Location(x=x, y=y, z=z + 10)
    debug.draw_arrow(arrow, arrow_top, life_time=30.0)

    # try:
    #     while vehicle is None:
    #         vehicle = world.try_spawn_actor(omafiets, transform)
    #         # time.sleep(0.3)
    #     actor_list.append(vehicle)

    #     time.sleep(15)
    # finally:
    #     client.apply_batch([carla.command.DestroyActor(x.id) for x in actor_list])
    #     print('Destroying Agent')
    pass


def ORCA_Test(env, agent):
    # faulthandler.enable()
    # client = carla.Client('localhost', 2000)
    # world = client.get_world()
    env.reset()
    env.world.tick()
    # controller_bp = env.blueprint_library.find('controller.ai.walker')
    # fountain = world.try_spawn_actor(controller_bp, carla.Transform(location=carla.Location(x=4., y=-98., z=0.)))
    # env.actor_list.append(fountain)
    # env.orca_actor_list.append(ORCAAgent(fountain,carla.Location(x=4., y=-98., z=0.),radius=6., max_speed=0.))
    log_time = datetime.now().strftime("%m-%d--%H-%M")
    # env.set_recordings(log_time)
    try:
        num_peds = 1
        env.pop_peds_ORCA(num_peds)
        tau = 2
        dt = 1 / 5
        agent_spawn = carla.Transform(
            carla.Location(
                x=float(env.spawn_locations[:, 0].min()), y=-109, z=0.177637
            ),
            rotation=carla.Rotation(yaw=0.0),
        )
        agent_spawn.location.y = np.random.choice(
            np.linspace(
                env.spawn_locations[:, 1].min(), env.spawn_locations[:, 1].max(), 4
            )
        )
        agent_goal = carla.Transform(
            carla.Location(
                x=float(env.spawn_locations[:, 0].max()),
                y=np.random.choice(
                    np.linspace(
                        env.spawn_locations[:, 1].min(),
                        env.spawn_locations[:, 1].max(),
                        2,
                    )
                ),
                z=0.177637,
            )
        )
        # debug = world.debug
        # x, y, z = agent_goal.location.x, agent_goal.location.y, 0.
        # arrow = carla.Location(x=x, y=y, z=z)
        # arrow_top = carla.Location(x=x, y=y, z=z+10)
        # debug.draw_arrow(arrow, arrow_top, thickness=1.5, life_time=15.)

        # env.enable_settings()
        # model_name = 'carla23-04-2022--14:57--from09:34'
        # agent = Herdragent(model_name=f'{model_name}.pth')
        agent.reset(agent_spawn, agent_goal, True)
        env.actor_list.append(agent.vehicle)
        env.orca_actor_list.append(
            ORCAAgent(agent.vehicle, agent_goal.location, radius=0.5, max_speed=1.4)
        )
        weather = env.set_weather()
        # job = Process(target=env.background_save, args=(env.plot_hist_front,env.plot_hist_top,), daemon=True)
        # job.start()
        start_time = time.time()
        save_frame_count = 0
        # agent.start_spin()
        # while (time.time()-start_time) < 60:
        while True:
            time.sleep(0.1)
            # if (time.time()-start_time) > 1:
            # update_orcas(env, tau, dt)
            # agent.step()
            # # env.get_recordings(agent)
            env.world.tick()
            agent.reset_check(agent_spawn, agent_goal, True)
            # save_frame_count += 1
            # env.dist2peds(agent)
            # agent.is_safe(env.controller_list)
            # if agent.safe == 1:
            #     env.ped_space_count += 1
            #     print(f'In Pedestrain Space #{env.ped_space_count}')
            # env.pos_hist.append(location2tensor(agent.vehicle.get_location()).numpy())
            # if len(agent.collision_hist) > 0:
            #     break
            #     # agent.cleanup()
            #     # agent.reset(agent_spawn,agent_goal,True)
            #     # env.actor_list[-1] = agent.vehicle
            #     # env.orca_actor_list[-1] = ORCAAgent(agent.vehicle,agent_goal.location,radius=0.5, max_speed=0)
            # if agent.done == True:
            #     break

    finally:
        del start_time
        agent.done = True
        # env.plot_hist_front.put('done')
        pl = env.pathlength(agent.pos_hist)
        collision_object = (
            "None" if len(agent.collision_hist) == 0 else agent.collision_hist[-1]
        )
        data = {
            "Times in ped space": env.ped_space_count,
            "Path length": pl,
            "Avg min dist to ped": np.asarray(env.dist2peds_list).mean(),
            "Success": agent.success,
            "Collision Object": collision_object,
            "Point2Point Dist": agent.p2pdist.item(),
            "Agent goal gain": agent.goal_gain,
            "Agent action gain": agent.action_gain,
            "# of Peds": num_peds,
            "Sim Time": int(save_frame_count / 5),
            "Weather": weather,
        }
        agent.cleanup()
        # try:
        #     job.join()
        #     job.close()
        # finally:
        # plot_trajectory(np.asarray(env.pos_hist), torch.ones((len(env.pos_hist))), agent.GOAL[0,0,:], collision=len(agent.collision_hist), traj_length=env.pathlength(agent.pos_hist))
        # plt.savefig(f'/home/nathan/HERDR/Carla_Results/{log_time}_trajectory.png')
        # plt.close('all')
        # env.top_writer.finish()
        # env.writer.finish()
        env.cleanup()
        return data


def update_orcas(env, tau, dt):
    for i, agent in enumerate(env.orca_actor_list):
        # if i == len(env.orca_actor_list)-1:
        #     continue
        agent.update()
        candidates = env.orca_actor_list[:i] + env.orca_actor_list[i + 1 :]
        new_vels, _ = orca(agent, candidates, tau, dt)
        new_vels = np.asarray(new_vels)
        mag_new_vels = np.linalg.norm(new_vels)
        if mag_new_vels == 0:
            mag_new_vels = 1e-10
        new_vels = new_vels / mag_new_vels
        dir = carla.Vector3D(*new_vels)
        spd = mag_new_vels
        try:
            env.actor_list[i].apply_control(
                carla.WalkerControl(direction=dir, speed=spd)
            )
        except:
            pass


if __name__ == "__main__":
    # test_ped()
    # check_spawn_points()
    env = HERDRenv()
    # env.reworld()
    # blk = 1
    model_name = (
        "carla23-04-2022--14:57--from09:34"  # 'carla23-04-2022--14:57--from09:34'
    )
    # agent = Herdragent(training=False, model_name=f'{model_name}.pth',test_block=blk) # 'carla07-04-2022--14:41.pth'
    # agent = Herdragent(training=False, model_name=f'{model_name}.pth')
    agent = ros_agent()
    # len_df = 0
    # while len_df < 1:
    data = ORCA_Test(env, agent)
    # data = test(env, agent, blk)
    # print(data)
    # file_name = f'/home/nathan/HERDR/Carla_Results/ORCA_1Peds_Test.pkl'
    # len_df = add2pickle(file_name, data)
    # len_df +=1
    # main(env, agent)
