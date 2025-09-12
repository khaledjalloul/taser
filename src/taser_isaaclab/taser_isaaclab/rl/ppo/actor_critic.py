import torch
import torch.nn as nn
from torch.distributions import Normal


class RunningNorm:
    """Simple running mean/std normalization for inputs."""

    def __init__(self, obs_dim: int):
        self.mean = torch.zeros(obs_dim)
        self.var = torch.ones(obs_dim)
        self.count = 1e-5

    def update(self, x: torch.Tensor):
        batch_mean = x.mean(0)
        batch_var = x.var(0, unbiased=False)
        batch_count = x.shape[0]

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

    def normalize(self, x: torch.Tensor) -> torch.Tensor:
        return (x - self.mean.to(x.device)) / torch.sqrt(self.var.to(x.device) + 1e-8)


class ActorCritic(nn.Module):
    """Actor-Critic model."""

    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()

        self.obs_norm = RunningNorm(obs_dim)

        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.LayerNorm(64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.LayerNorm(64),
            nn.ReLU(),
            nn.Linear(64, act_dim),
        )

        self.critic = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.LayerNorm(64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.LayerNorm(64),
            nn.ReLU(),
            nn.Linear(64, 1),
        )

        # log_std initialized small and clamped during forward pass
        self.log_std = nn.Parameter(torch.ones(act_dim) * -0.5)

    def forward(
        self, obs: torch.Tensor, update_norm: bool = False
    ) -> tuple[Normal, torch.Tensor]:
        if update_norm:
            self.obs_norm.update(obs)
        obs_norm = self.obs_norm.normalize(obs)

        mu = self.actor(obs_norm)
        # Clamp log_std for numerical stability
        log_std = torch.clamp(self.log_std, min=-20, max=2)
        std = log_std.exp()
        action_dist = Normal(mu, std)

        value = self.critic(obs_norm).squeeze(-1)

        return action_dist, value

    def save(self, path: str):
        torch.save(
            {
                "model_state_dict": self.state_dict(),
                "obs_mean": self.obs_norm.mean,
                "obs_var": self.obs_norm.var,
                "obs_count": self.obs_norm.count,
            },
            path,
        )

    def load(self, path: str):
        saved_data = torch.load(path, weights_only=True)
        self.load_state_dict(
            saved_data["model_state_dict"],
        )
        self.obs_norm.mean = saved_data["obs_mean"]
        self.obs_norm.var = saved_data["obs_var"]
        self.obs_norm.count = saved_data["obs_count"]

    def to(self, device: torch.device, **kwargs):
        self.obs_norm.mean = self.obs_norm.mean.to(device)
        self.obs_norm.var = self.obs_norm.var.to(device)
        return super().to(device, **kwargs)
