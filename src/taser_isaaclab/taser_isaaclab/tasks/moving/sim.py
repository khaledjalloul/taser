import argparse
from pathlib import Path

parser = argparse.ArgumentParser(
    description="Simulate the TASER robot moving task."
)
parser.add_argument("--num_envs", type=int, default=1,
                    help="Number of environments to spawn.")
parser.add_argument("--model_path", type=str,
                    default=Path.home() / "colcon_ws" / "checkpoints" / "best_model.pth",
                    help="Path to the trained model.")

############################################################

from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

############################################################

import torch
import gymnasium as gym

from taser_isaaclab.tasks.moving import TaserEnvCfg
from taser_isaaclab.rl.ppo.actor_critic import ActorCritic


def main():
    env_cfg = TaserEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = args.device
    env = gym.make("Isaac-TASER-Moving-v0", cfg=env_cfg)

    obs = env.reset()

    # Create the model instance with same dimensions
    model = ActorCritic(obs_dim=env.unwrapped.obs_dim,
                        act_dim=env.unwrapped.act_dim,
                        device=args.device).to(args.device)

    # Load weights
    model.load_state_dict(torch.load(
        args.model_path, map_location=args.device, weights_only=True))
    model.eval()

    # simulate physics
    count = 0

    while simulation_app.is_running():
        with torch.inference_mode():
            action_dist: torch.distributions.Normal
            action_dist, _ = model(obs, update_norm=True)
            action = action_dist.sample()

            # step the environment
            obs, rewards, terminated, truncated, info = env.step(action)
            print("obs", obs)
            print("rewards", rewards)
            # update counter
            count += 1

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
