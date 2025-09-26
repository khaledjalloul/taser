import torch
from isaaclab.envs import ManagerBasedRLEnv

from taser.isaaclab.common.base_env_cfg import TaserBaseEnvCfg


class TaserBaseEnv(ManagerBasedRLEnv):
    """Custom TASER manager-based environment."""

    def __init__(self, cfg: TaserBaseEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

    def reset(self, **kwargs):
        obs_dict, extras = super().reset(**kwargs)

        obs_dict["policy"] = torch.nan_to_num(obs_dict["policy"], nan=0.0)
        obs_dict["proprio"] = torch.nan_to_num(obs_dict["proprio"], nan=0.0)

        return obs_dict, extras

    def step(self, action: torch.Tensor, **kwargs):
        obs_dict, rewards, terminated, truncated, info = super().step(action, **kwargs)

        obs_dict["policy"] = torch.nan_to_num(obs_dict["policy"], nan=0.0)
        obs_dict["proprio"] = torch.nan_to_num(obs_dict["proprio"], nan=0.0)
        rewards = torch.nan_to_num(rewards, nan=0.0)

        return obs_dict, rewards, terminated, truncated, info
