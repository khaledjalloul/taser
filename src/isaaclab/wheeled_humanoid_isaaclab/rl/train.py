import argparse

parser = argparse.ArgumentParser(
    description="Train or run the wheeled humanoid environment."
)
parser.add_argument("--num_envs", type=int, default=1,
                    help="Number of environments to spawn.")
parser.add_argument("--save_freq", type=int, default=10,
                    help="How often to save model (in updates).")
parser.add_argument("--eval_freq", type=int, default=10,
                    help="How often to evaluate (in updates).")
parser.add_argument("--checkpoint_dir", type=str,
                    default="checkpoints", help="Directory to save checkpoints.")
parser.add_argument("--resume", type=str, default=None,
                    help="Path to checkpoint to resume from.")

############################################################

import isaaclab
from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

############################################################

import gymnasium as gym
import torch
from pathlib import Path
from tqdm import tqdm

from wheeled_humanoid_isaaclab.rl.ppo import PPOTrainer, PPOTrainerCfg
from wheeled_humanoid_isaaclab.tasks.moving import WheeledHumanoidEnvCfg

TOTAL_TIMESTEPS = 1000000
NUM_EPISODES = 5


def train(env):
    # Setup checkpoint directory
    checkpoint_dir = Path(args.checkpoint_dir)
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    trainer_cfg = PPOTrainerCfg(
        learning_rate=3e-4,
        num_steps=2048,
        num_envs=args.num_envs,
        batch_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_eps=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        device=env.unwrapped.device
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

        # Evaluation
        if (update + 1) % args.eval_freq == 0:

            total_reward = 0
            with torch.no_grad():
                for _ in range(NUM_EPISODES):
                    obs_dict = env.reset()
                    obs = torch.tensor(obs_dict["policy"],
                                       dtype=torch.float32, device=args.device)
                    done = False
                    episode_reward = 0

                    while not done:
                        dist, _ = trainer.policy(obs)
                        action = dist.mean  # Use mean action for evaluation
                        next_obs_dict, reward, terminated, truncated, _ = env.step(
                            action.cpu().numpy())
                        done = terminated or truncated
                        episode_reward += reward
                        obs = torch.tensor(
                            next_obs_dict["policy"], dtype=torch.float32, device=args.device)

                    total_reward += episode_reward
            eval_reward = total_reward / NUM_EPISODES

            print(f"\nUpdate {update + 1}/{num_updates}")
            print(f"Eval reward: {eval_reward:.2f}")
            print(f"Training loss: {train_info['loss']:.4f}")
            print(f"Policy loss: {train_info['policy_loss']:.4f}")
            print(f"Value loss: {train_info['value_loss']:.4f}")
            print(f"Entropy: {train_info['entropy']:.4f}\n")

            # Save best model
            if eval_reward > best_reward:
                best_reward = eval_reward
                trainer.save_model(checkpoint_dir / "best_model.pth")

        # Regular checkpointing
        if (update + 1) % args.save_freq == 0:
            trainer.save_model(checkpoint_dir / f"model_{update + 1}.pth")

    # Save final model
    trainer.save_model(checkpoint_dir / f"model_final.pth")

    return trainer, best_reward


def main():
    # Create environment
    env_cfg = WheeledHumanoidEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = args.device
    env = gym.make("Isaac-Wheeled-Humanoid-Moving-v0", cfg=env_cfg)

    # Train the model
    trainer, best_reward = train(env)
    print(f"\nTraining completed! Best reward achieved: {best_reward:.2f}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
