from dataclasses import MISSING, dataclass

import torch


@dataclass
class TrainCfg:
    """Configuration for the PPO trainer."""

    num_iters: int = MISSING
    """Number of PPO updates to perform"""

    num_rollout_steps: int = 10
    """Number of rollout timesteps to collect for each PPO update"""

    num_epochs: int = 5
    """Number of epochs to train per PPO update"""

    learning_rate: float = 3e-4
    """Learning rate"""

    lr_decay_factor: float = 1.0  # Disabled, tried 0.995 but led to worse performance
    """Decay factor for exponential learning rate schedule"""

    gamma: float = 0.99
    """Discount factor"""

    gae_lambda: float = 0.95
    """GAE lambda"""

    clip_eps: float = 0.2
    """Clip epsilon"""

    ent_coef: float = 0.01
    """Entropy coefficient"""

    vf_coef: float = 0.5
    """Value function coefficient"""

    target_kl: float = 0.015
    """Target KL divergence threshold for early stopping"""

    save_freq: int = 500
    """Frequency of evaluation and model saving (in iterations)"""

    device: str = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    """Device to run the training on"""
