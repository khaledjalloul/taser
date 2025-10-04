import gymnasium as gym

gym.register(
    id="TASER-balance",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.balance_env_cfg:TaserBalanceEnvCfg",
        "rsl_rl_cfg_entry_point": "taser.isaaclab.rl.rsl_rl.ppo_cfg:TaserPPORunnerCfg",
    },
)


gym.register(
    id="TASER-track_velocity",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.track_velocity_env_cfg:TaserTrackVelocityEnvCfg",
        "rsl_rl_cfg_entry_point": "taser.isaaclab.rl.rsl_rl.ppo_cfg:TaserPPORunnerCfg",
    },
)

gym.register(
    id="TASER-stand_up",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.stand_up_env_cfg:TaserStandUpEnvCfg",
        "rsl_rl_cfg_entry_point": "taser.isaaclab.rl.rsl_rl.ppo_cfg:TaserPPORunnerCfg",
    },
)

gym.register(
    id="TASER-pick",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pick_env_cfg:TaserPickEnvCfg",
        "rsl_rl_cfg_entry_point": "taser.isaaclab.rl.rsl_rl.ppo_cfg:TaserPPORunnerCfg",
    },
)
