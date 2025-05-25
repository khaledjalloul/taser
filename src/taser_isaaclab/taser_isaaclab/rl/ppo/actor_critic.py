import torch
import torch.nn as nn
from torch.distributions import Normal


class ActorCritic(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, act_dim)
        )
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )
        self.log_std = nn.Parameter(torch.zeros(act_dim))  # learnable log std

    def forward(self, obs) -> tuple[Normal, torch.Tensor]:
        # Actor
        mu = self.actor(obs)
        std = self.log_std.exp()
        action_dist = Normal(mu, std)

        # Critic
        value = self.critic(obs).squeeze(-1)

        return action_dist, value
