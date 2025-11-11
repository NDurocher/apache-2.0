from termios import VEOL
import numpy as np
import torch


class HERDRPlan:
    def __init__(
        self, Horizon=10, vel_init=1.5, steer_init=0, gamma=50, variance=(0.3, 1.5)
    ):
        # Set default starting value for actions
        # [Velocity (m/s), Steering angle (rad)]
        self.horizon = Horizon
        self.vel_init = vel_init
        self.steer_init = steer_init
        self.vel_max = vel_init * 1.0
        vel_mean = vel_init * torch.ones(self.horizon)
        steer_mean = steer_init * torch.zeros(self.horizon)
        self.mean = torch.stack((vel_mean, steer_mean)).double()
        # set guess for variance
        vel_cov = variance[0] * torch.ones(self.horizon, 1)
        steer_cov = variance[1] * torch.ones(
            self.horizon, 1
        )  # 0.1*torch.arange(1, self.horizon+1).unsqueeze(1)
        self.cov = torch.stack((vel_cov, steer_cov)).transpose(2, 0)
        # Define parameter to adjust for high weight updates
        self.gamma = torch.tensor(gamma)
        self.beta = 0.6

    def reset(self):
        self.__init__(self.horizon, self.vel_init, self.steer_init, self.gamma.item())

    def sample_new(self, batches=1):
        cov = self.cov.repeat(batches, 1, 1)
        mu = torch.zeros(batches, self.horizon, 2)
        noise = torch.normal(mean=mu, std=cov)
        sequence = []
        for i in range(self.horizon):
            if i == 0:
                # continue
                temp = (
                    self.beta * (self.mean[:, i + 1] + noise[:, i, :])
                    + (1 - self.beta) * self.mean[:, i]
                )
            elif i == (self.horizon - 1):
                action_init = torch.tensor([self.vel_init, self.steer_init])
                temp = (
                    self.beta * (action_init + noise[:, i, :])
                    + (1 - self.beta) * sequence[-1]
                )
            else:
                temp = (
                    self.beta * (self.mean[:, i + 1] + noise[:, i, :])
                    + (1 - self.beta) * sequence[-1]
                )
            sequence.append(temp)
        sequence = torch.stack(sequence, dim=1)

        # Clamp velocity between [0.1, 1.5*vel_init] m/s
        sequence[:, :, 0] = torch.where(sequence[:, :, 0] < 0.1, 0.1, sequence[:, :, 0])
        sequence[:, :, 0] = torch.where(
            sequence[:, :, 0] > self.vel_max, self.vel_max, sequence[:, :, 0]
        )
        # Clamp angle between [-0.95, 0.95]
        sequence[:, :, 1] = torch.where(
            sequence[:, :, 1] < -0.95, -0.95, sequence[:, :, 1]
        )
        sequence[:, :, 1] = torch.where(
            sequence[:, :, 1] > 0.95, 0.95, sequence[:, :, 1]
        )

        sequence = sequence.float()
        return sequence

    # def step(self):
    #     action_init = torch.tensor([self.vel_init,self.steer_init])[:,None]
    #     self.mean = torch.hstack((self.mean[:,1:], action_init))

    def update_new(self, reward, sequence):
        # reward is a [batch x horizon x 1] tensor, sequence is a batch x horizon x 2 tensor
        reward = reward * torch.linspace(1, 0.8, self.horizon)
        reward = reward.sum(dim=1)
        reward = reward - reward.max()
        # reward = (reward - reward.min())/(reward.max() - reward.min()) - 1
        mean = torch.zeros(self.horizon, 2)
        s_R = torch.zeros(self.horizon, 1)
        for r, seq in zip(reward, sequence):
            mean += torch.exp(self.gamma * r) * seq
            s_R += torch.exp(self.gamma * r)
        self.mean = (mean / s_R).transpose(0, 1).double()


if __name__ == "__main__":
    device = torch.device("cuda:0")
    test = HERDRPlan(Horizon=10)
    samp = test.sample_new(batches=3)  # .to(device)
    # samp1 = test.sample_new()
    # samp = torch.stack((samp, samp1), 0)
    # print(samp.shape)
    # print(torch.randint(0,49,(1,)).item())
    R = torch.tensor(np.random.rand(3, 10))  # .to(device)
    # samp = samp.unsqueeze(0)
    # test.gamma = test.gamma.to(device)
    test.update_new(R, samp)
    print(test.mean)
    test.reset()
    print(test.mean)
