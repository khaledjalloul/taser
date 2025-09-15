from isaaclab.utils import configclass

from taser.isaaclab.common import BaseTaserEnv, BaseTaserEnvCfg
from taser.isaaclab.tasks.balance import (
    EventsCfg,
    ObservationsCfg,
    RewardsCfg,
    TerminationsCfg,
)


@configclass
class TaserEnvCfg(BaseTaserEnvCfg):
    """TASER environment configuration for the balance task."""

    events = EventsCfg()
    observations = ObservationsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()


class TaserEnv(BaseTaserEnv):
    """TASER manager-based environment for the balance task."""

    pass
