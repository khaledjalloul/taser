from .mdp.events import EventsCfg
from .mdp.observations import ObservationsCfg
from .mdp.rewards import RewardsCfg
from .mdp.terminations import TerminationsCfg
from .env import TaserEnv, TaserEnvCfg

import gymnasium as gym


TASK_NAME = "Isaac-TASER-TrackVelocity-v0"

gym.register(
    id=TASK_NAME,
    entry_point=f"{__name__}.env:TaserEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env:TaserEnvCfg",
    },
)

__all__ = ["EventsCfg", "ObservationsCfg", "RewardsCfg",
           "TerminationsCfg", "TaserEnv", "TaserEnvCfg", "TASK_NAME"]
