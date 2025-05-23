import gymnasium as gym


gym.register(
    id="Isaac-Wheeled-Humanoid-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.cfgs.env:WheeledHumanoidEnvCfg",
    },
)
