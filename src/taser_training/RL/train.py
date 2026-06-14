import argparse

parser = argparse.ArgumentParser(description="Train one of the TASER tasks.")
parser.add_argument("--task", type=str, required=True, help="Task to train on.")
parser.add_argument(
    "--num_envs", type=int, default=16_384, help="Number of environments to spawn."
)
parser.add_argument("--resume", type=str, help="Path to checkpoint to resume from.")

############################################################

from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.task = f"TASER-{args.task}"

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

############################################################

from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import gymnasium as gym
import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg
from tqdm import tqdm

import taser_training.RL.isaaclab.tasks  # noqa: F401 # register tasks
from taser_training.RL.algorithm import PPO, TrainCfg
from taser_training.wandb_logger import WandbLogger


def train(env: ManagerBasedRLEnv):
    # Set up output path
    run_name = f"RL_{args.task}_{datetime.now().strftime('%m%d_%H%M%S')}"

    output_path = Path.cwd() / "outputs" / "RL" / run_name
    output_path.mkdir(parents=True, exist_ok=True)

    train_cfg = TrainCfg(
        num_iters=env.unwrapped.cfg.max_num_ppo_updates,
        device=env.unwrapped.device,
    )

    # Initialize wandb logger
    logger = WandbLogger(
        exp_name=run_name,
        base_path=output_path,
        config={"num_envs": args.num_envs, **asdict(train_cfg)},
    )

    # Initialize trainer
    alg = PPO(env=env, cfg=train_cfg)

    # Load checkpoint if resuming
    if args.resume:
        alg.policy.load(args.resume)

    env.unwrapped.num_ppo_updates = 0  # Set PPO update counter for curriculum
    env.reset()

    # Training loop
    best_reward = float("-inf")
    tqdm_bar = tqdm(
        range(train_cfg.num_iters), desc="Training", dynamic_ncols=True, leave=True
    )

    for iter in tqdm_bar:
        # Rollout
        with torch.no_grad():
            obs_dict = env.unwrapped.observation_manager.compute()
            obs_dict = {k: torch.nan_to_num(v, nan=0.0) for k, v in obs_dict.items()}

            for _ in range(train_cfg.num_rollout_steps):
                action_dist, value = alg.policy(obs_dict, update_norm=True)
                action = action_dist.sample()

                next_obs_dict, reward, terminated, truncated, extras = env.step(action)
                done = torch.logical_or(terminated, truncated)

                alg.update_buffers(obs_dict, action, reward, done, action_dist, value)

                obs_dict = {
                    k: torch.nan_to_num(v, nan=0.0) for k, v in next_obs_dict.items()
                }

            # Compute final value for bootstrapping
            _, final_val = alg.policy(obs_dict, update_norm=True)

        # Update policy
        info = alg.update(iter=iter, final_val=final_val)

        # Log training metrics
        logger.log(
            {
                **{f"train/{k}": v for k, v in info.items()},
                **extras["log"],
            },
            step=iter,
            max_steps=train_cfg.num_iters,
            tqdm=tqdm_bar,
        )

        # Save models
        if (iter + 1) % train_cfg.save_freq == 0:
            # Save latest model
            alg.policy.save(output_path / "latest_model.pth")

            # Save best model
            if info["mean_reward"] > best_reward:
                best_reward = info["mean_reward"]
                alg.policy.save(output_path / "best_model.pth")

    # Close wandb run
    logger.finish()


def main():
    env_cfg = parse_env_cfg(
        task_name=args.task,
        num_envs=args.num_envs,
    )
    env = gym.make(args.task, cfg=env_cfg)

    train(env)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
