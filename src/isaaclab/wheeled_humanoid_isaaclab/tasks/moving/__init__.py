from .env import WheeledHumanoidEnvCfg

__all__ = ["WheeledHumanoidEnvCfg"]

import gymnasium as gym

gym.register(
    id="Isaac-Wheeled-Humanoid-Moving-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env:WheeledHumanoidEnvCfg",
        # "sb3_cfg_entry_point": f"{__name__}.config:sb3_ppo_cfg.yaml",
        # "skrl_cfg_entry_point": f"{__name__}.config:skrl_ppo_cfg.yaml",
        # "rsl_rl_cfg_entry_point": f"{__name__}.config.rsl_rl_cfg:WheeledHumanoidPPORunnerCfg"
    },
)
