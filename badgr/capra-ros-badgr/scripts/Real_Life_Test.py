#!/usr/bin/env python3
from pathlib import Path

import cv2
import numpy as np
import torch
import os
from torch import nn

# import ros_compatibility as roscomp
# from ros_compatibility.node import CompatibleNode

from actionplanner import HERDRPlan
from Badgrnet import HERDR
from metrics_utils import plot_action_cam_view
from torchvision.transforms import CenterCrop

# from PIL import Image
from RL_config import get_params
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, Twist

# from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
import rospy


def img_callback(img_msg, Control_Policy):

    img = torch.tensor(
        Control_Policy.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
    )
    # img = img.reshape((self.im_height, self.im_width, 4))
    img = img[:, :, :3]
    # ''' Image assumed to be bgr '''
    indices = torch.tensor([2, 1, 0])
    img = torch.index_select(img, 2, indices)
    """ Image shape [im_height, im_width, 3] assumed to be now be rbg """
    # rospy.loginfo(img.shape)
    crop = CenterCrop((480, 640))  # (height, width)
    img = img.permute(2, 0, 1).float()
    cropped_img = crop(img)
    Control_Policy.frame = cropped_img
    Control_Policy.Step()


def speed_callback(status, Control_Policy):
    Control_Policy.speed = status.velocity
    # rospy.loginfo(status.header)


class HerdrAgent(object):
    def __init__(self):
        self.name = "Herdr Planner"

    def initialize(self, params):
        self.params = params
        # self.params = get_params()
        if torch.cuda.is_available():
            self.device = torch.device("cuda:0")
        else:
            self.device = torch.device("cpu")
        action_variance = (
            self.params["Velocity Sample Var"],
            self.params["Steering Sample Var"],
        )
        self.planner = HERDRPlan(
            self.params["Horizon"],
            self.params["Initial Speed"],
            self.params["Initial Steer Angle"],
            self.params["Gamma"],
            action_variance,
        )
        self.Get_Model()
        self.Get_Goal()

        self.bridge = CvBridge()
        self.control_pub = rospy.Publisher("output_control", Twist, queue_size=10)
        self.image_pub = rospy.Publisher("output_image", Image, queue_size=10)
        self.done = False
        self.speed = 0.0
        self.event = torch.zeros((self.params["Batches"], self.params["Horizon"], 1))

    def reset(self):
        self.planner.reset()
        self.event = torch.zeros((self.params["Batches"], self.params["Horizon"], 1))

    def Get_Model(self):
        # dir_name = str(Path(Path.cwd().parent))
        # model_path = f'{dir_name}/models/{self.params["Model Name"]}'
        model_path = (
            "/opt/capra/overlay_ws/src/capra-badgr/models/" + self.params["Model Name"]
        )
        self.model = torch.load(model_path, map_location=self.device)
        self.model.model_out = nn.Sequential(self.model.model_out, nn.Sigmoid())
        self.model.eval()
        self.model.to(self.device)

    def Get_Interupt(self):
        """Get Stop Signal from ROS if present"""
        ## TODO Implement (might not need)
        pass

    def Get_Position(self):
        """Retrive Position and Orientation from ROS"""
        self.Position = torch.zeros((2))
        self.Rotation = torch.zeros((3))
        ## TODO Implement
        pass

    def Get_Goal(self):
        self.goal = torch.zeros((2)).repeat(self.params["Batches"], 1, 1)
        ## TODO Implement
        pass

    def Set_Actions(self):
        """Send actions over ROS"""
        # if self.planner.mean[0, 0] > self.speed:
        #     a = 0.8
        # else:
        #     a = 0.
        # controls = CarlaEgoVehicleControl()
        # controls.throttle = a
        controls = Twist()
        controls.linear.x = self.planner.mean[0, 0].item()
        controls.angular.z = self.planner.mean[1, 0].item()
        # controls.twist.linear.x = self.planner.mean[0,0].item()
        # controls.twist.angular.z = self.planner.mean[1,0].item()
        self.control_pub.publish(controls)

    def Propogate_States(self):
        new_pos = self.Position
        rotation = self.Rotation
        new_state = torch.cat((new_pos, rotation))
        batch_state = new_state.repeat(
            self.params["Batches"], self.params["Horizon"], 1
        ).transpose(1, 2)
        dt = 1 / self.params["Control Freq"]
        """ [X Y Z Phi] """
        for i in range(0, self.params["Horizon"] - 1):
            batch_state[:, 0, i + 1] = (
                batch_state[:, 0, i]
                + dt * torch.cos(batch_state[:, 3, i]) * self.actions[:, i, 0]
            )
            batch_state[:, 1, i + 1] = (
                batch_state[:, 1, i]
                + dt * torch.sin(batch_state[:, 3, i]) * self.actions[:, i, 0]
            )
            batch_state[:, 3, i + 1] = (
                batch_state[:, 3, i]
                - dt
                * self.actions[:, i, 1]
                * self.actions[:, i, 0]
                / self.params["Wheel Base"]
            )

        ## TODO Check rotation frame of robot and verify state is updating correctly

        """ Output shape: [BATCH, HRZ, 4] """
        return batch_state.permute(0, 2, 1)

    def Call_Model(self):
        img = self.frame.unsqueeze(0)
        self.event = (
            self.model(img.to(self.device), self.actions.to(self.device))[:, :, 0]
            .detach()
            .cpu()
        )

        # self.state = self.Propogate_States()
        """ goal_cost Shape: [BATCH, HRZ] """
        # goal_cost = torch.linalg.norm(self.state[:,:,:2]-self.goal, dim=2)
        # goal_cost = (goal_cost - goal_cost.min()) / (goal_cost.max() - goal_cost.min())

        action_cost = (
            self.actions[:, :, 1] ** 2 / 2 + (self.actions[:, :, 0] - 1.0) ** 2 / 2
        )
        # self.score = self.params['Goal Cost Gain'] * goal_cost + event + self.params['Action Cost Gain'] * action_cost
        self.score = self.event + self.params["Action Cost Gain"] * action_cost
        return -self.score

    def Finish_Check(self):
        """Check if Robot is within distance to goal location"""

        dist2goal = torch.linalg.norm(self.Position - self.goal[0])
        if dist2goal <= 1.5:
            self.done = True
            self.success = 1.0
            print("Made it!!!")

    def publish_output_image(self):
        output_image = plot_action_cam_view(
            self.frame,
            self.event,
            1 / self.params["Control Freq"],
            self.params["Wheel Base"],
            self.actions.numpy(),
        )

        image_msg = self.bridge.cv2_to_imgmsg(output_image)
        self.image_pub.publish(image_msg)

    def Step(self):
        # self.Get_Interupt()
        # self.Get_Image()
        # self.Get_Position()

        self.actions = self.planner.sample_new(batches=self.params["Batches"])
        score = self.Call_Model()

        # self.publish_output_image()

        self.planner.update_new(score, self.actions)

        self.Set_Actions()

        # self.Finish_Check()


if __name__ == "__main__":
    planner = HerdrAgent()
    parameters = {}
    # executor = roscomp.executors.MultiThreadedExecutor()
    # executor.add_node(planner)

    parameters["Image Topic"] = os.getenv("camera_topic", "/carla/ego/front/image")
    parameters["Output Image Topic"] = os.getenv(
        "output_image_topic", "/capra/badgr_output"
    )
    parameters["Control Topic"] = os.getenv(
        "control_topic", "/carla/ego/vehicle_control_cmd"
    )
    parameters["Velocity Topic"] = os.getenv(
        "velocity_topic", "/carla/ego/vehicle_status"
    )
    parameters["Control Freq"] = os.getenv("control_freq", 5)
    parameters["Batches"] = os.getenv("batches", 50)
    parameters["Horizon"] = os.getenv("horizon", 10)
    parameters["Initial Speed"] = os.getenv("initial_velocity", 1.5)
    parameters["Initial Steer Angle"] = os.getenv("initial_steer", 0.0)
    parameters["Gamma"] = os.getenv("gamma", 20)
    parameters["Velocity Sample Var"] = os.getenv("velocity_variance", 0.3)
    parameters["Steering Sample Var"] = os.getenv("steering_variance", 1.5)
    parameters["Goal Cost Gain"] = os.getenv("goal_gain", 0.25)
    parameters["Action Cost Gain"] = os.getenv("action_gain", 0.2)
    parameters["Wheel Base"] = os.getenv("wheel_base", 0.7)
    parameters["Model Name"] = os.getenv(
        "model_name", "carla23-04-2022--14:57--from09:34.pth"
    )

    planner.initialize(parameters)
    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("/cam0/image_raw", Image, img_callback, planner)
    # rospy.Subscriber('input_vel', CarlaEgoVehicleStatus, speed_callback, planner)
    # s = rospy.Service('image_server', Get_Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # while not agent.done:
    #     agent.Step()
