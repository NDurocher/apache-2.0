import torch
#import onnx
from torch import nn
from actionplanner import HERDRPlan
from Badgrnet import HERDR

import numpy

model_path = "/opt/capra/overlay_ws/src/capra-badgr/models/carla23-04-2022--14:57--from09:34.pth"
model = torch.load(model_path, map_location="cpu")
model.model_out = nn.Sequential(model.model_out, nn.Sigmoid())
model.eval()

dummy_input = (torch.zeros(1,3,480,640), torch.zeros(1,10,2))

torch.onnx.export(
    model,
    dummy_input,
    "herdr.onnx",
    input_names=["image", "actions"],
    output_names=["output"],
    dynamic_axes={
        'actions': {0: 'batches', 1: 'actions'}
    }
)
