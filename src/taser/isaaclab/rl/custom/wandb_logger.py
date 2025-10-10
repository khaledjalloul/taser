import logging
from pathlib import Path
from typing import Any, Dict, Optional

import wandb
from dotenv import dotenv_values


class WandbLogger:
    """Logger class for Weights & Biases integration."""

    # Default wandb configuration
    WANDB_PROJECT = "TASER"

    def __init__(
        self,
        config: Dict[str, Any],
        exp_name: Optional[str] = None,
        project: Optional[str] = None,
        base_path: Optional[str] = None,
    ):
        """Initialize the wandb logger.

        Args:
            config: Dictionary of training configuration/hyperparameters
            exp_name: Name of the experiment (run)
            project: Wandb project name
            base_path: Path to save the model checkpoints
        """

        logger = logging.getLogger("TASER")

        env_config = dotenv_values(".env.local")
        api_key = env_config.get("WANDB_API_KEY")

        if not api_key:
            logger.warning(
                "WANDB_API_KEY environment variable not set. "
                "Weights & Biases logging will not be available."
            )
            self.enabled = False
            return

        self.enabled = True

        wandb.login(key=api_key)
        wandb.init(
            project=project or self.WANDB_PROJECT,
            name=exp_name,
            config=config,
        )

        # Initialize episode tracking
        self.episode_rewards = []
        self.episode_lengths = []
        self.base_path = base_path

    def log_training_step(self, train_info: Dict[str, float], update: int):
        if not self.enabled:
            return

        """Log training metrics for a single update step."""
        wandb.log(
            {
                "train/policy_loss": train_info["policy_loss"],
                "train/value_loss": train_info["value_loss"],
                "train/entropy": train_info["entropy"],
                "train/total_loss": train_info["loss"],
                "train/kl_divergence": train_info["kl"],
                "train/common_step_counter": train_info["common_step_counter"],
            },
            step=update,
        )

    def log_evaluation(self, eval_reward: float, best_reward: float, update: int):
        if not self.enabled:
            return

        """Log evaluation metrics."""
        wandb.log(
            {
                "eval/mean_reward": eval_reward,
                "eval/best_reward": best_reward,
            },
            step=update,
        )

    def save_model(self, model_path: Path):
        """Save model checkpoint to wandb."""
        if not self.enabled:
            return

        wandb.save(str(model_path), base_path=self.base_path)

    def finish(self):
        """Close the wandb run."""
        if not self.enabled:
            return

        wandb.finish()
