import logging
import os
from pathlib import Path
from typing import Any, Dict, Optional

import wandb


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

        api_key = os.environ.get("WANDB_API_KEY")

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

    def log(self, metrics: Dict[str, Any], step: Optional[int] = None):
        """Log metrics to wandb."""
        if not self.enabled:
            return

        wandb.log(metrics, step=step)

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
