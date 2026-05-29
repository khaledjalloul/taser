import torch
from torch import nn
from torch.nn import functional as F

from taser_training.BC.model.config import MLPCfg


class MLP(nn.Module):
    def __init__(self, config=MLPCfg()):
        super().__init__()
        self.config = config

        hidden_layers = []
        for i in range(len(config.hidden_layers)):
            in_dim = config.obs_dim if i == 0 else config.hidden_layers[i - 1]
            out_dim = config.hidden_layers[i]
            hidden_layers.append(nn.Linear(in_dim, out_dim))
            hidden_layers.append(nn.ReLU())

        self.backbone = nn.Sequential(*hidden_layers)
        self.action_head = nn.Linear(
            config.hidden_layers[-1], config.action_dim * config.action_chunk_size
        )

        self.register_buffer("obs_mean", torch.zeros(config.obs_dim))
        self.register_buffer("obs_std", torch.ones(config.obs_dim))
        self.register_buffer("action_max", torch.ones(config.action_dim))
        self.register_buffer("action_mean", torch.zeros(config.action_dim))
        self.register_buffer("action_std", torch.ones(config.action_dim))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = (x - self.obs_mean) / self.obs_std

        x = self.backbone(x[:, -1, :])
        actions = self.action_head(x)

        actions = actions.view(
            x.size(0), 1, self.config.action_chunk_size, self.config.action_dim
        )
        # actions = torch.tanh(actions)
        output = {"actions": actions}

        return output


if __name__ == "__main__":
    from taser.common.logger import logger

    model = MLP()

    x = torch.randn(10, 25, model.config.obs_dim)
    out = model(x)
    logger.info(out["actions"].shape)
