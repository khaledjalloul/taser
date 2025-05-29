import torch

from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from isaaclab.utils import configclass

from taser_isaaclab.common import ActionsCfg, ObservationsCfg, concat_obs
from taser_isaaclab.tasks.moving.mdp import EventsCfg, RewardsCfg, TerminationsCfg
from taser_isaaclab.tasks.moving.scene import SceneCfg


@configclass
class TaserEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the TASER robot environment."""

    scene = SceneCfg(num_envs=1, env_spacing=10)

    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventsCfg()

    rewards = RewardsCfg()
    terminations = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 1
        self.episode_length_s = 10
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 0.01
        self.sim.render_interval = 5


class TaserEnv(ManagerBasedRLEnv):
    """TASER robot environment."""

    def __init__(self, cfg: TaserEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

    def reset(self, **kwargs):
        obs_dict, _ = super().reset(**kwargs)
        obs = concat_obs(obs_dict, self.scene["robot"])
        return obs

    def step(self, action: torch.Tensor, **kwargs):
        obs_dict, rewards, terminated, truncated, info = super().step(action, **kwargs)
        obs = concat_obs(obs_dict, self.scene["robot"])

        rewards = torch.nan_to_num(rewards, nan=0.0)

        return obs, rewards, terminated, truncated, info

    @property
    def obs_dim(self):
        return self.observation_space["policy"].shape[1] + 13

    @property
    def act_dim(self):
        return self.action_space.shape[1]
