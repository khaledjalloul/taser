import torch
import torch.nn as nn
from torch.distributions import Normal


class RunningNorm:
    """Simple running mean/std normalization for inputs."""
    def __init__(self, shape, eps=1e-5, device=None):
        self.mean = torch.zeros(shape).to(device)
        self.var = torch.ones(shape).to(device)
        self.count = eps

    def update(self, x):
        batch_mean = x.mean(0)
        batch_var = x.var(0, unbiased=False)
        batch_count = x.size(0)

        delta = batch_mean - self.mean
        total_count = self.count + batch_count

        new_mean = self.mean + delta * batch_count / total_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        M2 = m_a + m_b + delta.pow(2) * self.count * batch_count / total_count
        new_var = M2 / total_count

        self.mean = new_mean.detach()
        self.var = new_var.detach()
        self.count = total_count

    def normalize(self, x):
        return (x - self.mean.to(x.device)) / torch.sqrt(self.var.to(x.device) + 1e-8)


class ActorCritic(nn.Module):
    def __init__(self, obs_dim, act_dim, device=None):
        torch.autograd.set_detect_anomaly(True)
        super().__init__()

        self.obs_norm = RunningNorm(obs_dim, device=device)

        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.LayerNorm(64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.LayerNorm(64),
            nn.ReLU(),
            nn.Linear(64, act_dim)
        )

        self.critic = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.LayerNorm(64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.LayerNorm(64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )

        # log_std initialized small and clamped during forward pass
        self.log_std = nn.Parameter(torch.ones(act_dim) * -0.5)

        self.apply(self._init_weights)

    def _init_weights(self, module):
        if isinstance(module, nn.Linear):
            nn.init.orthogonal_(module.weight, gain=1.0)
            if module.bias is not None:
                nn.init.zeros_(module.bias)

    def forward(self, obs, update_norm=True) -> tuple[Normal, torch.Tensor]:
        if update_norm:
            self.obs_norm.update(obs)
        obs_norm = self.obs_norm.normalize(obs)

        # Actor: bounded mean output with tanh
        # mu = torch.tanh(self.actor(obs_norm))
        mu = self.actor(obs_norm)
        # Clamp log_std for numerical stability
        log_std = torch.clamp(self.log_std, min=-20, max=2)
        std = log_std.exp()
        action_dist = Normal(mu, std)

        # Critic
        value = self.critic(obs_norm).squeeze(-1)

        return action_dist, value
