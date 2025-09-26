import gymnasium as gym

gym.register(
    id="TASER-balance",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.balance_env_cfg:TaserEnvCfg",
        "rsl_rl_cfg_entry_point": "taser.isaaclab.rl.rsl_rl.ppo_cfg:TaserPPORunnerCfg",
    },
)


gym.register(
    id="TASER-track_velocity",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.track_velocity_env_cfg:TaserEnvCfg",
        "rsl_rl_cfg_entry_point": "taser.isaaclab.rl.rsl_rl.ppo_cfg:TaserPPORunnerCfg",
    },
)
