from dataclasses import dataclass


@dataclass
class PPOTrainerCfg:
    """Configuration for the PPO trainer."""

    learning_rate: float
    """Learning rate"""

    num_steps: int
    """Number of steps per PPO update"""

    batch_epochs: int
    """Number of PPO epochs per update"""

    gamma: float
    """Discount factor"""

    gae_lambda: float
    """GAE lambda"""

    clip_eps: float
    """Clip epsilon"""

    ent_coef: float
    """Entropy coefficient"""

    vf_coef: float
    """Value function coefficient"""

    target_kl: float
    """Target KL divergence threshold for early stopping"""

    device: str
    """Device to run the PPO trainer on"""
