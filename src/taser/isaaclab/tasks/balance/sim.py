import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description="Simulate the TASER robot balance task.")
parser.add_argument(
    "--num_envs", type=int, default=4, help="Number of environments to spawn."
)
parser.add_argument(
    "--model_path",
    type=str,
    default=Path(__file__).parent / "data" / "model.pth",
    help="Path to the trained model.",
)

############################################################

from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

############################################################

import gymnasium as gym
import torch

from taser.isaaclab.rl import ActorCritic
from taser.isaaclab.tasks.balance import TASK_NAME, TaserEnvCfg


def main():
    env_cfg = TaserEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env = gym.make(TASK_NAME, cfg=env_cfg)

    obs = env.reset()

    model = ActorCritic(
        obs_dim=env.unwrapped.obs_dim,
        act_dim=env.unwrapped.act_dim,
    ).to(env.unwrapped.device)

    model.load(args.model_path)
    model.eval()

    while simulation_app.is_running():
        with torch.inference_mode():
            action_dist, _ = model(obs)
            print(obs)
            action = action_dist.mean
            obs, rewards, terminated, truncated, info = env.step(action)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
