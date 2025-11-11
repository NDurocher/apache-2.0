from cgitb import handler
import glob
import imp
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import faulthandler
import carla
import cv2
import csv
import time
import numpy as np
import torch
from torch import nn
import time
from datetime import datetime
from pathlib import Path
import random
from matplotlib import pyplot as plt

from Badgrnet import HERDR
from actionplanner import HERDRPlan
from metrics_utils import plot_action_cam_view, plot_actions, plot_trajectory


def location2tensor(location):
    ls = [location.x, location.y, location.z]
    return torch.tensor(ls)

class Pedagent():
    
    im_width = 640 # pixels
    im_height = 480 # pixels
    FOV = 150.0 # degrees

    td_im_width = 500 # pixels
    td_im_height = 500 # pixels
    td_FOV = 90.0 # degrees
    
    control_freq = 5 # Hz
    horizon = 1
    batches = 1
    init_vel = 1.5 # m/s
    goal_gain = 0.25 # magic number - set to 0 for no target location reward
    action_gain = 0.0
    wheelbase = 0.7 # m
    CAM_SHOW = False # bool to show front rgb camera preview
    

    def __init__(self, test_block=1):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(8.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.walker_bp = random.choice(self.blueprint_library.filter('walker.pedestrian.*'))
        self.controller_bp = self.blueprint_library.find('controller.ai.walker')
        self.planner = HERDRPlan(Horizon=self.horizon, vel_init=self.init_vel, gamma=20)
        self.spawn_location_file = f'/home/nathan/HERDR/spawn_locations_test{test_block}.txt'
        self.state = torch.zeros((self.batches, self.horizon, 4))
        self.event = torch.zeros((self.batches, self.horizon))
        self.planner = HERDRPlan(self.horizon)
        self.planner.mean = None

        self.actor_list = []
        self.sensor_list = []
        self.collision_hist = []
        self.pos_hist = []
        self.action_hist = []
        self.im_hist = []
        self.GND_hist = []
        self.vel_hist = []
        self.spawn_locations = []

        self.done = False # flag for done
        self.success = 0.
        self.score = None
        self.p2pdist = 0
        self.vehicle = None
        self.frame = None # initalizer for front rgb camera img
        self.depth_frame = None # initalizer for front depth camera img
        self.topview = None # initalizer for top view rgb camera img

        self.load_spawns()       

    def load_spawns(self):
        with open(self.spawn_location_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                self.spawn_locations.append(row)
            self.spawn_locations = np.asarray(self.spawn_locations).astype(np.int16)

    def get_spawn(self):
        rot = float(np.random.uniform(0,360,1))
        loc = np.random.choice(np.arange(0,self.spawn_locations.shape[0]-1))
        return carla.Transform(carla.Location(x=float(self.spawn_locations[loc,0]), y=float(self.spawn_locations[loc,1]), z=0.177637), carla.Rotation(yaw=rot))
        
    def get_goal(self, tf, ai_controller):
        while True:
            trans = carla.Transform()
            ''' Get random spawn point from list '''
            # trans.location = self.world.get_random_location_from_navigation()
            loc = np.random.choice(np.arange(0,self.spawn_locations.shape[0]-1))
            trans.location = carla.Location(x=float(self.spawn_locations[loc,0]), y=float(self.spawn_locations[loc,1]), z=0.16)
            if (trans.location.x != tf.location.x) and (trans.location.y != tf.location.y) :
                break
        ''' *** Try to remove .repeat() to save memory *** '''
        ai_controller.go_to_location(trans.location)
        self.GOAL = torch.tensor([trans.location.x, trans.location.y]).repeat(self.batches, 1, 1)
        print(f'Current location is: {tf.location}')
        print(f'Goal location is: {trans.location}')

    def img_process(self, image, cam):
        img = torch.tensor(image.raw_data)
        if cam == 'car':
            img = img.reshape((self.im_height, self.im_width, 4))
            img = img[:, :, :3]
            '''Image shape [im_height, im_width, 3], Torch tensor BGR'''
            if self.CAM_SHOW == True:
                cv2.imshow("Front_CAM", img.float().numpy()/255)
                # cv2.waitKey(1)
            '''Image shape [3, im_height, im_width]'''
            self.frame = img.permute(2, 0, 1).float()
        elif cam == 'td':
            img = img.reshape((self.td_im_height, self.td_im_width, 4))
            img = img[:, :, :3]
            '''Image shape [im_height, im_width, 3], Torch tensor BGR'''
            self.topview = img.float()
                
    def collison_check(self, event):
        if not self.done:
            print(f"Collided with {event.other_actor.type_id}")
            self.collision_hist.append(event.other_actor.type_id)
    
    def lane_check(self, event):
         if not self.done:
            # self.collision_hist.append(event.timestamp)
            print("Supposedly drove onto Road")
    
    def tilt_check(self, event):
        roll, pitch, yaw = event.gyroscope.x, event.gyroscope.y, event.gyroscope.z
        # print(roll, pitch, yaw) 
        if not self.done:
            max_tilt_rate = 0.80
            if abs(roll) >= max_tilt_rate or abs(pitch) >= max_tilt_rate:
                print("Tilted")
                self.collision_hist.append(event.timestamp)

    def reset(self, tf=None, goal=None, orca_test=False):
        self.pos_hist.clear()
        self.done = False
        self.success = 0.

        # self.world = self.client.get_world()
        self.world.tick()
        # faulthandler.enable()
        if orca_test:
            transform = tf
            self.GOAL = torch.tensor([goal.location.x, goal.location.y]).repeat(self.batches, 1, 1)
            y = tf.location.y
            while self.vehicle is None:
                try:
                    self.vehicle = self.world.spawn_actor(self.walker_bp, transform)
                except:
                    y = np.random.choice(np.linspace(y-3,y+3,20))
                    transform.location.y = y
        else:
            transform = self.get_spawn()
            start_time = time.time()
        
            while self.vehicle is None:
                # transform = carla.Transform(carla.Location(x=115.203445, y=-37.268463, z=0.177637))
                try:
                    self.vehicle = self.world.spawn_actor(self.walker_bp, transform)
                except:
                    transform = self.get_spawn()
                if (time.time() - start_time >= 40):
                    break
        self.actor_list.append(self.vehicle)
        self.controller_walker = self.world.try_spawn_actor(self.controller_bp, carla.Transform(), self.vehicle)
        self.sensor_list.append(self.controller_walker)
        


        self.camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        self.camera_bp.set_attribute('sensor_tick', f'{1/self.control_freq}')

        '''Add forward facing camera '''
        self.camera_bp.set_attribute("image_size_x", f'{self.im_width}')
        self.camera_bp.set_attribute("image_size_y", f'{self.im_height}')
        self.camera_bp.set_attribute("fov", f'{self.FOV}')
        car_camera_transform = carla.Transform(carla.Location(x=0.5, z=1.4))
        self.cam = self.world.spawn_actor(self.camera_bp, car_camera_transform, attach_to=self.controller_walker)
        self.sensor_list.append(self.cam)
        self.cam.listen(lambda image: self.img_process(image, 'car'))

        '''Add top down camera '''
        td_cam_pos = self.vehicle.get_location()
        topdown_camera_transform = carla.Transform(carla.Location(x=td_cam_pos.x, y=td_cam_pos.y, z=15), carla.Rotation(pitch=-90))
        self.camera_bp.set_attribute("image_size_x", f'{self.td_im_width}')
        self.camera_bp.set_attribute("image_size_y", f'{self.td_im_height}')
        self.camera_bp.set_attribute("fov", f'{self.td_FOV}')
        self.tdcam = self.world.spawn_actor(self.camera_bp, topdown_camera_transform)
        self.sensor_list.append(self.tdcam)
        self.tdcam.listen(lambda image: self.img_process(image, 'td'))

        ''' Add collision sensor '''
        collison_sensor = self.blueprint_library.find("sensor.other.collision")
        self.collison_sensor = self.world.spawn_actor(collison_sensor, car_camera_transform, attach_to=self.controller_walker)
        self.sensor_list.append(self.collison_sensor)
        self.collison_sensor.listen(lambda event: self.collison_check(event))

        # ''' Add road detection sensor '''
        # lane_sensor = self.blueprint_library.find("sensor.other.lane_invasion")
        # self.lane_sensor = self.world.spawn_actor(lane_sensor, car_camera_transform, attach_to=self.controller_walker)
        # self.sensor_list.append(self.lane_sensor)
        # self.lane_sensor.listen(lambda event: self.lane_check(event))

        ''' Add IMU '''
        imu = self.blueprint_library.find("sensor.other.imu")
        self.imu = self.world.spawn_actor(imu, car_camera_transform, attach_to=self.controller_walker)
        self.sensor_list.append(self.imu)
        self.imu.listen(lambda event: self.tilt_check(event))

        # del self.camera_bp 
        # del self.blueprint_library
        while self.frame is None:
            self.world.tick()
            time.sleep(0.01)
        if not orca_test:
            self.get_goal(transform, self.controller_walker)
            self.controller_walker.start()
        self.calc_p2pdist(transform.location)

    def is_safe(self, ped_list):
        self.safe = torch.zeros((1))
        if len(self.collision_hist) > 0:
            self.safe = 1 # which means unsafe
            return
        self.vehicle_location = location2tensor(self.vehicle.get_location())
        for walker in ped_list:
            walker_trans = walker.get_transform()
            walker_pos = location2tensor(walker_trans.location)
            ''' Simple personal space model with an ellipse of radii "a" & "b" and offset by "shift" '''
            a = 0.8
            b = 1.5
            A = torch.tensor(walker_trans.rotation.yaw/180*np.pi)
            shift = 1
            k = shift * torch.cos(A)
            h = - shift * torch.sin(A)
            first_term = torch.square(
                (self.vehicle_location[0] - walker_pos[0] - h) * torch.cos(A) + 
                (self.vehicle_location[1] - walker_pos[1] - k) * torch.sin(A)) / a ** 2
            second_term = torch.square(
                (self.vehicle_location[0] - walker_pos[0] - h) * torch.sin(A) - 
                (self.vehicle_location[1] - walker_pos[1] - k) * torch.cos(A)) / b ** 2
            check = (first_term + second_term) < 1
            check = check.int()
            self.safe = torch.logical_or(check, self.safe).float()
            if self.safe.item() == 1:
                print(f'In Pedestrain Space.')
                return
        return

    def calc_p2pdist(self, loc):
        loc = location2tensor(loc)[[0,1]]
        self.p2pdist = torch.cdist(loc.unsqueeze(0),self.GOAL[0], p=1.0)
        
    def reset_check(self):
        pos = location2tensor(self.vehicle.get_location())
        pos = pos[0:2]
        dist2goal = torch.linalg.norm(pos - self.GOAL[0,:,:])
        if dist2goal <= 1.5:
            self.done = True
            self.success = 1.
            print('Made it!!!')

    def step(self):
        now = datetime.now()
        self.vel_hist.append(self.vehicle.get_control().speed)
        self.reset_check()
        position = location2tensor(self.vehicle.get_location()).numpy()
        if (position[0] != 0) and (position[1] != 0):
            self.pos_hist.append(position)

    def plot(self):
        dir_name = Path(Path.cwd())
        dir_name = str(dir_name) + '/Ped_results'
        fig = plt.figure(figsize=(16, 9), dpi=80)
        plt.plot(self.vel_hist)
        plt.xlabel('Time (step  #)')
        plt.ylabel('Velocity (m/s)')
        plt.title(f'CARLA Pedestrain Velocity through Time')
        # plt.clf()
        # plt.plot(self.head_hist)
        # plt.xlabel('Time (step  #)')
        # plt.ylabel('Heading (°)')
        # plt.title(f'CARLA Pedestrain Heading through Time')
        # plt.savefig(f"{dir_name}/CARLA_agent_heading_time.jpg")
        plt.savefig(f"{dir_name}/CARLA_agent_velocity_time.jpg")
        plt.close('all')

    def cleanup(self):
        # self.plot()
        print('Destroying Agent')
        self.client.apply_batch_sync([carla.command.DestroyActor(x.id) for x in self.actor_list])
        print('Destroying Sensors')
        [x.stop() for x in self.sensor_list]
        self.client.apply_batch_sync([carla.command.DestroyActor(x.id) for x in self.sensor_list])
        self.vehicle = None
        self.actor_list.clear()
        self.sensor_list.clear()
        self.collision_hist.clear()
        self.pos_hist.clear()
        self.action_hist.clear()
        self.im_hist.clear()
        self.vel_hist.clear()
        self.GND_hist.clear()
        time.sleep(0.5)


if __name__=='__main__':
    print('This is only a class')
    pass