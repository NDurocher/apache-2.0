import numpy as np
import torch
import os
from pathlib import Path
import glob
from torch.utils import data
from torchvision.io import read_image
from torch import nn
from torchvision.transforms.functional import resize, hflip
from torch.utils.tensorboard import SummaryWriter
from torch.utils.data.sampler import SubsetRandomSampler, RandomSampler
from torchvision import transforms

# from PIL import Image
import h5py
from Badgrnet import HERDR
from datetime import datetime

# import deepspeed


class carla_hdf5dataclass(data.Dataset):
    """Input params:
    file_path: Path to the folder containing the dataset (multiple HDF5 files) or Path to file of single dataset.
    recursive: If True, searches for h5 files in subdirectories.
    load_all_files: If True, loads all the datasets immediately into RAM. Use this if
        trianing after collection or want to continue to grow dataset
    transform: PyTorch transform to apply to every data instance (default=None)."""

    def __init__(
        self,
        h5file_path,
        horizon,
        imagefile_path,
        load_all_files=False,
        counting=False,
        recursive=False,
        transform=None,
    ):
        self.pos_w = torch.tensor(12.6544)
        self.counting = counting
        self.transform = transform
        self.image_fp = imagefile_path
        self.preprocess = transforms.Compose(
            [
                transforms.Resize(256),
                transforms.CenterCrop(224),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
                ),
            ]
        )
        # torch.manual_seed(12)
        if torch.cuda.is_available():
            self.device = torch.device("cuda:0")
            print("Use GPU")
        else:
            self.device = torch.device("cpu")

        self.horizon = horizon
        if load_all_files:
            """Search for all h5 files else just load one"""
            p = Path(h5file_path)
            assert p.is_dir()
            if recursive:
                files = sorted(p.glob("**/*.h5"))
            else:
                files = sorted(p.glob("*.h5"))
            if len(files) < 1:
                raise RuntimeError("No hdf5 datasets found")
            self.data = np.concatenate(
                [
                    self.loadfromfile(str(h5dataset_fp.resolve()))
                    for h5dataset_fp in files
                ]
            )
            self.recursive_image_folders = [
                f.path for f in os.scandir(self.image_fp) if f.is_dir()
            ]
        else:
            self.data = self.loadfromfile(h5file_path)

        """find all indices where not done"""
        self.valid_start_indices = np.where(self.data[:, -1] == "False")[0]

    def loadfromfile(self, file_path):
        with h5py.File(file_path) as h5_file:
            file_arr = np.ndarray(shape=(1, 5))
            for gpname, gp in h5_file.items():
                # num_samples = len(gp['gnd'])
                group_arr = np.concatenate(
                    [
                        gp[name][...].astype(str)
                        if "actions" in name
                        else gp[name][...][:, None].astype(str)
                        for name in gp.keys()
                    ],
                    axis=1,
                )
                done_arr = np.array(
                    [
                        False if i < len(group_arr) - 1 else True
                        for i in range(len(group_arr))
                    ]
                )
                group_arr = np.concatenate((group_arr, done_arr[:, None]), axis=1)
                file_arr = np.concatenate((file_arr, group_arr), axis=0)

        """ Delete empty row at start of arr"""
        file_arr = file_arr[1:]

        """ file array shape: [velocity, steer angle, gnd, image name, done]"""
        return file_arr

    def get_data(self, i):
        end_i = i + self.horizon
        """ get image"""
        img_name = self.data[i, 3]
        if self.counting:
            img = torch.zeros((2, 2))
        else:
            path = Path(f"{self.image_fp}/{img_name}.jpg")
            if not path.is_file():
                folder_i = iter(self.recursive_image_folders)
                while not path.is_file():
                    folder = next(folder_i)
                    path = Path(f"{folder}/{img_name}.jpg")
            # path = glob.glob(f'{self.image_fp}/**/{img_name}.jpg', recursive=True)[0]
            # img = resize(read_image(f'{path}'),[96,192]).float()
            img = read_image(f"{path}").float()
            # img = Image.open(f'{path}')
            # img = self.preprocess(img)
            # img = read_image(f'{self.image_fp}/{img_name}.jpg').float()

        ls = np.concatenate(
            (self.data[i:end_i, 0:3], self.data[i:end_i, 4, None]), axis=1
        ).copy()
        done_index = np.where(ls[:, 3] == "True")[0]
        if len(done_index) != 0:
            done_index = done_index[0]
            num_fake_acts = end_i - (done_index + i + 1)
            mu_vel = torch.tensor(ls[done_index, 0].astype(float)).repeat(num_fake_acts)
            fake_vel = torch.normal(mu_vel, 0.1 * torch.ones(num_fake_acts))
            mu_steer = torch.tensor(ls[done_index, 1].astype(float)).repeat(
                num_fake_acts
            )
            fake_steer = torch.normal(mu_steer, 0.2 * torch.ones(num_fake_acts))
            fake_acts = torch.stack((fake_vel, fake_steer)).T
            act = torch.tensor(ls[: done_index + 1, 0:2].astype(float))
            act = torch.vstack((act, fake_acts))
            """ act.shape = [10,2], gnd.shape = [10,1]"""
            gnd = torch.tensor(ls[: done_index + 1, 2].astype(float))
            gnd = torch.hstack((gnd, torch.ones(num_fake_acts))).unsqueeze(1)
        else:
            """act.shape = [10,2], gnd.shape = [10,1]"""
            act = torch.from_numpy(ls[:, 0:2].astype(float))
            gnd = torch.from_numpy(ls[:, 2].astype(float)).unsqueeze(1)

        return img, act.float(), gnd.float()

    def __len__(self):
        return len(self.data)

    def __getitem__(self, index):
        img, act, gnd = self.get_data(index)

        if torch.rand(1).item() >= 0.5:
            img = hflip(img)
            act[:, 1] = -act[:, 1]

        return img, act, gnd

    def calculate_position(self, actions):
        state = torch.zeros((actions.shape[0], actions.shape[1], 3)).to(self.device)
        control_freq = torch.tensor(1 / 5).to(self.device)
        wheelbase = torch.tensor(0.7).to(self.device)
        """ [X Y Phi] """
        for i in range(0, actions.shape[1] - 1):
            state[:, i + 1, 0] = (
                state[:, i, 0]
                + (1 / control_freq) * torch.cos(state[:, i, 2]) * actions[:, i, 0]
            )
            state[:, i + 1, 1] = (
                state[:, i, 1]
                + (1 / control_freq) * torch.sin(state[:, i, 2]) * actions[:, i, 0]
            )
            state[:, i + 1, 2] = (
                state[:, i, 2]
                - (1 / control_freq) * actions[:, i, 1] * actions[:, i, 0] / wheelbase
            )
        """ Output shape: [batch, 3], return just X,Y """
        state[:, :, :2] = (state[:, :, :2] - state[:, :, :2].mean()) / (
            state[:, :, :2].max() - state[:, :, :2].min()
        )
        return state[:, :, :2]

    def one_epoch(self, model, dataloader, start_step=0, writer=None, opt=None):
        train = False if opt is None else True
        model.train() if train else model.eval()
        model.to(self.device)
        losses, correct, total = [], [], 0
        pos_correct, pos_total = [], 0
        incorrect = 0
        criterion = nn.BCEWithLogitsLoss(
            reduction="sum", pos_weight=self.pos_w.to(self.device)
        )
        # pos_criteria = nn.MSELoss(reduction='sum')
        sig = nn.Sigmoid()
        step = start_step
        for img, act, gnd in dataloader:
            model.zero_grad()
            img, act, gnd = (
                img.to(self.device),
                act.to(self.device),
                gnd.to(self.device),
            )
            # positions = self.calculate_position(act)
            with torch.set_grad_enabled(train):
                # logits, position_est = model(img,act)
                logits = model(img, act)
            loss = criterion(logits, gnd)
            # loss = criterion(logits, gnd) + pos_criteria(position_est, positions)

            samples = gnd.shape[0] * gnd.shape[1]
            total += samples
            pos_samples = torch.count_nonzero(gnd)
            pos_total += pos_samples
            pos_correct.append(
                torch.count_nonzero(
                    torch.logical_and((abs(gnd - sig(logits)) < 0.30), gnd)
                ).item()
            )
            correct.append(torch.count_nonzero(abs(gnd - sig(logits)) < 0.50).item())
            incorrect = torch.count_nonzero(abs(sig(logits) - gnd) >= 0.50).item()

            if train:
                loss.backward()
                opt.step()
                opt.zero_grad()

            losses.append(loss.item())
            if writer is not None:
                writer.add_scalar(
                    "Train/Loss", losses[-1], step
                )  # writer.add_scalar("Loss/valid", losses[-1], step) if opt is None else
                writer.add_scalar(
                    "Train/Accuracy", correct[-1] / samples, step
                )  # writer.add_scalar("Accuracy/valid", correct[-1]/samples, step) if opt is None else
                writer.add_scalar(
                    "Train/Incorrect", incorrect / samples, step
                )  # writer.add_scalar("Incorrect/valid", incorrect/samples, step) if opt is None else
                if pos_samples > 0:
                    writer.add_scalar(
                        "Train/Pos_Accuracy", pos_correct[-1] / pos_samples, step
                    )  # writer.add_scalar("Pos_Accuracy/valid", pos_correct[-1]/pos_samples, step) if opt is None else
            del loss
            if step % 1000:
                torch.save(model, f"./models/carla_temp_{log_time}.pth")
            step += 1

        return np.mean(losses), sum(pos_correct) / pos_total, sum(correct) / total, step


if __name__ == "__main__":
    HRZ = 10
    dataset = carla_hdf5dataclass(
        "/home/nathan/HERDR/old_carla_hdf5s/",
        HRZ,
        "/home/nathan/HERDR/old_carla_images/",
        recursive=True,
        load_all_files=True,
    )
    test_sampler = SubsetRandomSampler(dataset.valid_start_indices)
    testloader = torch.utils.data.DataLoader(
        dataset, sampler=test_sampler, batch_size=32, num_workers=3
    )
    # print(next(iter(testloader)))

    pretrained = False
    if pretrained:
        model_name = "carla03-05-2022--11:08"
        model = torch.load(f"/home/nathan/HERDR/models/{model_name}.pth")
        opt = torch.optim.Adam(model.parameters(), lr=1e-4, weight_decay=1e-2)
        # log_time = '03-04-2022--10:16'
        log_time = (
            datetime.now().strftime("%d-%m-%Y--%H:%M") + model_name[-5:]
        )  # + "--from17:38"
        writer = SummaryWriter(log_dir=f"/home/nathan/HERDR/carla_logs/{log_time}")
    else:
        model = HERDR(Horizon=HRZ)
        opt = torch.optim.Adam(model.parameters(), lr=1e-4, weight_decay=1e-2)
        log_time = datetime.now().strftime("%d-%m-%Y--%H:%M")
        writer = SummaryWriter(log_dir=f"/home/nathan/HERDR/carla_logs/{log_time}")
        # writer = None

    max_loss = 10000
    end_step = 0
    for epoch in range(0, 9):
        loss, pos_accuracy, accuracy, end_step = dataset.one_epoch(
            model, testloader, start_step=end_step, writer=writer, opt=opt
        )
        print(
            f"Epoch {epoch+1} - Loss: {loss:.3f}, +Accuracy: {pos_accuracy:.3f}, TAccuracy: {accuracy:.3f}, # steps: {end_step}"
        )
        if loss < max_loss:
            max_loss = loss
            torch.save(model, f"./models/carla{log_time}.pth")
    print("---DONE---")
