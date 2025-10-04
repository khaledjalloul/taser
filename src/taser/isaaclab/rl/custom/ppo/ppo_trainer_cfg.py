from dataclasses import dataclass


@dataclass
class PPOTrainerCfg:
    """Configuration for the PPO trainer."""

    num_iters: int
    """Number of training iterations"""

    num_rollout_steps: int
    """Number of timesteps for each training step rollout"""

    num_epochs: int
    """Number of PPO epochs per update"""

    learning_rate: float
    """Learning rate"""

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

    eval_freq: int
    """Frequency of evaluation"""

    num_eval_steps: int
    """Number of timesteps to evaluate on"""

    save_freq: int
    """Frequency of saving model"""

    device: str
    """Device to run the PPO trainer on"""
