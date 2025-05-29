import argparse
from datetime import datetime
from pathlib import Path

RUN_NAME = f"ppo_training_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
DEFAULT_OUT_PATH = Path.home() / "colcon_ws" / "outputs" / \
    RUN_NAME / "checkpoints"

parser = argparse.ArgumentParser(
    description="Train one of the TASER tasks."
)
parser.add_argument("--num_envs", type=int, default=1,
                    help="Number of environments to spawn.")
parser.add_argument("--checkpoint_path", type=str,
                    default=DEFAULT_OUT_PATH,
                    help="Directory to save checkpoints.")
parser.add_argument("--resume", type=str, default=None,
                    help="Path to checkpoint to resume from.")

############################################################

from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

############################################################

import gymnasium as gym
import torch
from dataclasses import asdict
from tqdm import tqdm

from taser_isaaclab.rl import PPOTrainer, PPOTrainerCfg, WandbLogger
from taser_isaaclab.tasks.moving import TaserEnvCfg


TOTAL_TIMESTEPS = int(1e8)
"""Total number of timesteps for all envs to train for"""
EVAL_FREQ = 5
"""Frequency of evaluation"""
NUM_EVAL_TIMESTEPS = 512
"""Number of timesteps to evaluate on"""
SAVE_FREQ = 5
"""Frequency of saving model"""


def train(env: gym.Env):
    # Setup checkpoint directory
    checkpoint_path = Path(args.checkpoint_path)
    progress_path = checkpoint_path / "progress"
    progress_path.mkdir(parents=True, exist_ok=True)

    trainer_cfg = PPOTrainerCfg(
        learning_rate=3e-4,
        num_steps=2048,
        batch_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_eps=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        target_kl=0.015,
        device=env.unwrapped.device,
    )

    # Initialize wandb logger
    logger = WandbLogger(
        exp_name=RUN_NAME,
        base_path=args.checkpoint_path,
        config={
            "num_envs": args.num_envs,
            "total_timesteps": TOTAL_TIMESTEPS,
            **asdict(trainer_cfg)
        }
    )

    # Initialize trainer
    trainer = PPOTrainer(env=env, cfg=trainer_cfg)

    # Load checkpoint if resuming
    start_update = 0
    if args.resume is not None:
        trainer.load_model(args.resume)
        try:
            start_update = int(Path(args.resume).stem.split('_')[-1])
        except:
            pass

    # Training loop
    num_updates = TOTAL_TIMESTEPS // (args.num_envs * trainer_cfg.num_steps)
    best_reward = float('-inf')

    for update in tqdm(range(start_update, num_updates)):
        # Training update
        train_info = trainer.train_step()

        # Log training metrics
        logger.log_training_step(train_info, update)

        # Evaluation
        if (update + 1) % EVAL_FREQ == 0:
            eval_rewards = torch.zeros(args.num_envs, device=args.device)
            obs = env.reset()

            with torch.no_grad():
                for _ in range(NUM_EVAL_TIMESTEPS):
                    dist, _ = trainer.policy(obs)
                    action = dist.mean  # Use mean action for evaluation
                    obs, reward, _, _, _ = env.step(action)
                    eval_rewards += reward

            eval_reward = eval_rewards.mean()

            # Log evaluation metrics
            logger.log_evaluation(eval_reward.item(), best_reward, update)

            # Save best model
            if eval_reward.mean() > best_reward:
                best_reward = eval_reward.mean()
                best_model_path = checkpoint_path / "best_model.pth"
                trainer.save_model(best_model_path)
                logger.save_model(best_model_path)

        # Regular checkpointing
        if (update + 1) % SAVE_FREQ == 0:
            model_path = progress_path / f"model_{update + 1}.pth"
            trainer.save_model(model_path)
            logger.save_model(model_path)

    # Save final model
    final_model_path = checkpoint_path / "model_final.pth"
    trainer.save_model(final_model_path)
    logger.save_model(final_model_path)

    # Close wandb run
    logger.finish()

    return trainer, best_reward


def main():
    # Create environment
    env_cfg = TaserEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = args.device
    env = gym.make("Isaac-TASER-Moving-v0", cfg=env_cfg)

    # Train the model
    train(env)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
