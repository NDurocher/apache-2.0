#!/usr/bin/env python3
from pathlib import Path

import cv2
import numpy as np
import torch
import os
from torch import nn

from actionplanner import HERDRPlan
from Badgrnet import HERDR
from metrics_utils import plot_action_cam_view
from torchvision.transforms import CenterCrop

from RL_config import get_params
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, Twist

import rospy

import time
from functools import wraps

def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        rospy.loginfo(f'Function {func.__name__} took {total_time:.4f} seconds to execute.')
        return result
    return timeit_wrapper


def img_callback(img_msg, Control_Policy):

    img = torch.tensor(
        Control_Policy.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
    )
    img = img[:, :, :3]
    
    # Image assumed to be bgr
    indices = torch.tensor([2, 1, 0])
    img = torch.index_select(img, 2, indices)
    
    # Image shape [im_height, im_width, 3] assumed to be now be rgb
    crop = CenterCrop((480, 640))
    img = img.permute(2, 0, 1).float()
    cropped_img = crop(img)
    Control_Policy.frame = cropped_img

    # Control_Policy.Step()


class HerdrAgent(object):
    def __init__(self):
        self.name = "Herdr Planner"

    def initialize(self, params):
        self.params = params
        if torch.cuda.is_available():
            self.device = torch.device("cuda:0")
        else:
            self.device = torch.device("cpu")
        rospy.loginfo(f"device: {self.device}")
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
        self.event = torch.zeros((self.params["Batches"], self.params["Horizon"], 1))

    def reset(self):
        self.planner.reset()
        self.event = torch.zeros((self.params["Batches"], self.params["Horizon"], 1))

    def Get_Model(self):
        model_path = (
            "/opt/capra/overlay_ws/src/capra-badgr/models/" + self.params["Model Name"]
        )
        self.model = torch.load(model_path, map_location=self.device)
        self.model.model_out = nn.Sequential(self.model.model_out, nn.Sigmoid())
        self.model.eval()
        self.model.to(self.device)

    def Get_Interupt(self):
        # Get Stop Signal from ROS if present
        # TODO 
        pass

    def Get_Goal(self):
        self.goal = torch.zeros((2)).repeat(self.params["Batches"], 1, 1)
        ## TODO 
        pass

    def Set_Actions(self):
        # Send actions
        controls = Twist()
        controls.linear.x = self.planner.mean[0, 0].item()
        controls.angular.z = self.planner.mean[1, 0].item()
        self.control_pub.publish(controls)

    def Propogate_States(self):
        new_pos = self.Position
        rotation = self.Rotation

        new_state = torch.cat((new_pos, rotation))
        batch_state = new_state.repeat(
            self.params["Batches"], self.params["Horizon"], 1
        ).transpose(1, 2)
        dt = 1 / self.params["Control Freq"]
        # [X Y Z Phi]
        
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

        # TODO Check rotation frame of robot and verify state is updating correctly

        # Output shape: [BATCH, HRZ, 4]
        return batch_state.permute(0, 2, 1)

    @timeit
    def Call_Model(self):
        img = self.frame.unsqueeze(0)
        self.event = (
            self.model(img.to(self.device), self.actions.to(self.device))[:, :, 0]
            .detach()
            .cpu()
        )

        # self.state = self.Propogate_States()
        
        # goal_cost Shape: [BATCH, HRZ] """
        # goal_cost = torch.linalg.norm(self.state[:,:,:2]-self.goal, dim=2)
        # goal_cost = (goal_cost - goal_cost.min()) / (goal_cost.max() - goal_cost.min())

        action_cost = (
            self.actions[:, :, 1] ** 2 / 2 + (self.actions[:, :, 0] - 1.0) ** 2 / 2
        )
        # self.score = self.params['Goal Cost Gain'] * goal_cost + self.event + self.params['Action Cost Gain'] * action_cost
        self.score = self.event + self.params["Action Cost Gain"] * action_cost
        return -self.score

    def Finish_Check(self):
        # Check if Robot is within distance to goal location

        dist2goal = torch.linalg.norm(self.Position - self.goal[0])
        if dist2goal <= 1.5:
            self.done = True
            self.success = 1.0
            rospy.loginfo("Made it!!!")

    def publish_output_image(self):
        output_image = plot_action_cam_view(
            self.frame,
            self.event,
            1 / self.params["Control Freq"],
            self.params["Wheel Base"],
            self.actions.numpy(),
            self.planner.mean,
        )
        
        image_msg = self.bridge.cv2_to_imgmsg(output_image)
        self.image_pub.publish(image_msg)

    def Step(self):
        self.actions = self.planner.sample_new(batches=self.params["Batches"])
        score = self.Call_Model()

        #self.publish_output_image()

        self.planner.update_new(score, self.actions)

        self.Set_Actions()

def control_callback(event, policy):
    policy.Step()


if __name__ == "__main__":
    try:
        rospy.init_node("Herdr", anonymous=True)

        parameters = {}

        parameters["Image Topic"] = rospy.get_param("~camera_topic", "/carla/ego/front/image")
        parameters["Output Image Topic"] = rospy.get_param(
            "~output_image_topic", "/herdr_output_image"
        )
        parameters["Control Topic"] = rospy.get_param(
            "~control_topic", "/vel_cmd"
        )
        parameters["Control Freq"] = rospy.get_param("~control_freq", 5)
        parameters["Batches"] = int(rospy.get_param("~batches", 50))
        parameters["Horizon"] = int(rospy.get_param("~horizon", 10))
        parameters["Initial Speed"] = rospy.get_param("~initial_velocity", 1.5)
        parameters["Initial Steer Angle"] = rospy.get_param("~initial_steer", 0.0)
        parameters["Gamma"] = rospy.get_param("~gamma", 20)
        parameters["Velocity Sample Var"] = rospy.get_param("~velocity_variance", 0.3)
        parameters["Steering Sample Var"] = rospy.get_param("~steering_variance", 1.5)
        parameters["Goal Cost Gain"] = rospy.get_param("~goal_gain", 0.25)
        parameters["Action Cost Gain"] = rospy.get_param("~action_gain", 0.2)
        parameters["Wheel Base"] = rospy.get_param("~wheel_base", 0.7)
        parameters["Model Name"] = rospy.get_param(
            "~model_name", "carla23-04-2022--14:57--from09:34.pth"
        )

        for key in parameters.keys():
            print(f'Key: {key}, value: {parameters[key]}')

        planner = HerdrAgent()
        planner.initialize(parameters)

        rospy.Subscriber("/cam0/image_raw", Image, img_callback, planner)

        rospy.loginfo("Herdr running.")
        control_timer = rospy.Timer(rospy.Duration(1.0/parameters["Control Freq"]), lambda event: control_callback(event, planner))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass








