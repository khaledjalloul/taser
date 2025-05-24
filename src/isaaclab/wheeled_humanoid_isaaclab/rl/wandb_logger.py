import wandb
from pathlib import Path
from typing import Dict, Any, Optional
from datetime import datetime


class WandbLogger:
    """Logger class for Weights & Biases integration."""

    # Default wandb configuration
    WANDB_PROJECT = "wheeled-humanoid"
    WANDB_ENTITY = "khaledjalloul-eth-zurich"
    DEFAULT_EXP_NAME = f"ppo_training_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    def __init__(
        self,
        config: Dict[str, Any],
        exp_name: Optional[str] = None,
        project: Optional[str] = None,
        entity: Optional[str] = None
    ):
        """Initialize the wandb logger.

        Args:
            config: Dictionary of training configuration/hyperparameters
            exp_name: Name of the experiment (run)
            project: Wandb project name
            entity: Wandb entity (username or team name)
        """
        self.run = wandb.init(
            project=project or self.WANDB_PROJECT,
            entity=entity or self.WANDB_ENTITY,
            name=exp_name or self.DEFAULT_EXP_NAME,
            config=config
        )

        # Initialize episode tracking
        self.episode_rewards = []
        self.episode_lengths = []

    def log_training_step(
        self,
        train_info: Dict[str, float],
        update: int
    ):
        """Log training metrics for a single update step."""
        wandb.log({
            "train/policy_loss": train_info['policy_loss'],
            "train/value_loss": train_info['value_loss'],
            "train/entropy": train_info['entropy'],
            "train/total_loss": train_info['loss'],
        }, step=update)

    def log_evaluation(
        self,
        eval_reward: float,
        best_reward: float,
        update: int
    ):
        """Log evaluation metrics."""
        wandb.log({
            "eval/mean_reward": eval_reward,
            "eval/best_reward": best_reward,
        }, step=update)

    def save_model(self, model_path: Path):
        """Save model checkpoint to wandb."""
        wandb.save(str(model_path))

    def finish(self):
        """Close the wandb run."""
        wandb.finish()
