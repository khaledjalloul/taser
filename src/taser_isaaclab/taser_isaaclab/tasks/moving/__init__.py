from .env import TaserEnvCfg

__all__ = ["TaserEnvCfg"]

import gymnasium as gym

gym.register(
    id="Isaac-TASER-Moving-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env:TaserEnvCfg",
        # "sb3_cfg_entry_point": f"{__name__}.config:sb3_ppo_cfg.yaml",
        # "skrl_cfg_entry_point": f"{__name__}.config:skrl_ppo_cfg.yaml",
        # "rsl_rl_cfg_entry_point": f"{__name__}.config.rsl_rl_cfg:TaserPPORunnerCfg"
    },
)
