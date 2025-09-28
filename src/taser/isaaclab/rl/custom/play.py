import argparse

parser = argparse.ArgumentParser(description="Play one of the trained TASER tasks.")
parser.add_argument("--task", type=str, required=True, help="Task to play.")
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of environments to spawn."
)
parser.add_argument(
    "--model_path", type=str, required=True, help="Path to the trained model."
)

############################################################

from pathlib import Path

from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.task = f"TASER-{args.task}"

if not args.model_path:
    outputs_dir = Path("/workspaces/taser/outputs")
    subdirs = sorted(
        [outputs_dir / d for d in outputs_dir.glob(f"*{args.task}*") if d.is_dir()]
    )
    args.model_path = subdirs[-1] / "best_model.pth"
    print(f"No model path provided. Using the latest model at {args.model_path}")

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

############################################################

import gymnasium as gym
import torch
from isaaclab_tasks.utils import parse_env_cfg

import taser.isaaclab.tasks  # noqa: F401 # register tasks
from taser.isaaclab.rl.custom import ActorCritic


def play(env: gym.Env):
    obs_dim = sum([o.shape[1] for o in env.unwrapped.observation_space.values()])
    act_dim = env.unwrapped.action_space.shape[1]

    model = ActorCritic(obs_dim=obs_dim, act_dim=act_dim).to(env.unwrapped.device)

    model.load(args.model_path)
    model.eval()

    obs_dict, _ = env.reset()

    while simulation_app.is_running():
        with torch.inference_mode():
            action_dist, _ = model(obs_dict)
            action = action_dist.mean
            obs_dict, rewards, terminated, truncated, info = env.step(action)

    env.close()


if __name__ == "__main__":
    env_cfg = parse_env_cfg(
        task_name=args.task,
        num_envs=args.num_envs,
    )
    env = gym.make(args.task, cfg=env_cfg)

    play(env)

    env.close()
    simulation_app.close()
