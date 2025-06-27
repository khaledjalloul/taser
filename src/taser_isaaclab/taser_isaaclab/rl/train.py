import argparse

parser = argparse.ArgumentParser(
    description="Train one of the TASER tasks."
)
parser.add_argument("--task", type=str, default="TrackVelocity",
                    help="Task to train on.")
parser.add_argument("--num_envs", type=int, default=128,
                    help="Number of environments to spawn.")
parser.add_argument("--num_iters", type=int, default=200,
                    help="Number of iterations (rollout + training).")
parser.add_argument("--output_path", type=str,
                    help="Directory to save checkpoints.")
parser.add_argument("--resume", type=str,
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
from datetime import datetime
from isaaclab_tasks.utils import parse_env_cfg
from pathlib import Path
from tqdm import tqdm

from taser_isaaclab.rl import PPOTrainer, PPOTrainerCfg, WandbLogger


def train(env: gym.Env):
    # Set up output path
    run_name = f"PPO_{args.task}_{datetime.now().strftime('%m%d%H%M%S')}"

    if args.output_path is None:
        output_path = Path.home() / "colcon_ws" / "outputs" / run_name
    else:
        output_path = Path(args.output_path)

    progress_path = output_path / "progress"
    progress_path.mkdir(parents=True, exist_ok=True)

    trainer_cfg = PPOTrainerCfg(
        num_rollout_steps=2048,
        total_num_steps=int(args.num_envs * 2048 * args.num_iters),
        num_epochs=10,
        learning_rate=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_eps=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        target_kl=0.015,
        eval_freq=5,
        num_eval_steps=256,
        save_freq=5,
        device=env.unwrapped.device,
    )

    # Initialize wandb logger
    logger = WandbLogger(
        exp_name=run_name,
        base_path=output_path,
        config={
            "num_envs": args.num_envs,
            **asdict(trainer_cfg)
        }
    )

    # Initialize trainer
    trainer = PPOTrainer(env=env, cfg=trainer_cfg)

    # Load checkpoint if resuming
    if args.resume:
        trainer.policy.load(args.resume)

    # Training loop
    num_updates = trainer_cfg.total_num_steps // (args.num_envs *
                                                  trainer_cfg.num_rollout_steps)
    best_reward = float('-inf')

    for update in tqdm(range(num_updates)):
        # Training update
        train_info = trainer.train_step()

        # Log training metrics
        logger.log_training_step(train_info, update)

        # Evaluation
        if update % trainer_cfg.eval_freq == 0:
            eval_rewards = torch.zeros(
                args.num_envs, device=env.unwrapped.device)
            obs = env.reset()

            with torch.no_grad():
                for _ in range(trainer_cfg.num_eval_steps):
                    dist, _ = trainer.policy(obs)
                    action = dist.mean  # Use mean action for evaluation
                    obs, reward, _, _, _ = env.step(action)
                    eval_rewards += reward

            eval_reward = eval_rewards.mean()

            # Save best model
            if eval_reward.mean() > best_reward:
                best_reward = eval_reward.mean()
                best_model_path = output_path / "best_model.pth"
                trainer.policy.save(best_model_path)

            # Log evaluation metrics
            logger.log_evaluation(eval_reward.item(), best_reward, update)

        # Regular checkpointing
        if update % trainer_cfg.save_freq == 0 and update != 0:
            model_path = progress_path / f"model_{update + 1}.pth"
            trainer.policy.save(model_path)

    # Save final model
    final_model_path = output_path / "final_model.pth"
    trainer.policy.save(final_model_path)

    # Close wandb run
    logger.finish()


if __name__ == "__main__":
    full_task_name = f"Isaac-TASER-{args.task}-v0"
    env_cfg = parse_env_cfg(
        full_task_name,
        num_envs=args.num_envs,
    )
    env = gym.make(full_task_name, cfg=env_cfg)

    train(env)

    env.close()
    simulation_app.close()
