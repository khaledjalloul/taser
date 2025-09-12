import torch
from isaaclab.utils import configclass
from taser_isaaclab.common import BaseTaserEnv, BaseTaserEnvCfg
from taser_isaaclab.tasks.track_velocity import (
    EventsCfg,
    ObservationsCfg,
    RewardsCfg,
    TerminationsCfg,
)


@configclass
class TaserEnvCfg(BaseTaserEnvCfg):
    """TASER environment configuration for the track velocity task."""

    events = EventsCfg()
    observations = ObservationsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()


class TaserEnv(BaseTaserEnv):
    """TASER manager-based environment for the track velocity task."""

    def __init__(self, cfg: TaserEnvCfg, **kwargs):
        self._target_vel_b = torch.zeros((cfg.scene.num_envs, 2)).to(cfg.sim.device)
        self._t = torch.zeros(cfg.scene.num_envs).to(cfg.sim.device)
        self._max_lin_vel = 1.5
        self._max_ang_vel = 0.3

        super().__init__(cfg, **kwargs)

    def reset(self, **kwargs):
        self._t = torch.zeros(self.cfg.scene.num_envs).to(self.cfg.sim.device)
        self._generate_target_vel()
        return super().reset(**kwargs)

    def step(self, action: torch.Tensor, **kwargs):
        self._t += self.cfg.sim.dt
        return super().step(action, **kwargs)

    def _generate_target_vel(self, env_ids: torch.Tensor | None = None):
        """Generate a target velocity at the current time step to be used as an observation for the robot training."""
        # base_sin_t = torch.sin(
        #     (self._t / self.cfg.episode_length_s) * 2 * torch.pi)
        # v = self._max_lin_vel * base_sin_t
        # w = self._max_ang_vel * base_sin_t

        if env_ids is None:
            self._target_vel_b[:, 0] = (
                torch.rand(self.cfg.scene.num_envs) * (2 * self._max_lin_vel)
                - self._max_lin_vel
            )
            self._target_vel_b[:, 1] = (
                torch.rand(self.cfg.scene.num_envs) * (2 * self._max_ang_vel)
                - self._max_ang_vel
            )
        else:
            self._target_vel_b[env_ids, 0] = (
                torch.rand(env_ids.shape[0], device=self.cfg.sim.device)
                * (2 * self._max_lin_vel)
                - self._max_lin_vel
            )
            self._target_vel_b[env_ids, 1] = (
                torch.rand(env_ids.shape[0], device=self.cfg.sim.device)
                * (2 * self._max_ang_vel)
                - self._max_ang_vel
            )

    @property
    def target_vel_b(self):
        return self._target_vel_b
