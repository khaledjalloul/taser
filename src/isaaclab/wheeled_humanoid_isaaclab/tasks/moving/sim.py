import argparse

parser = argparse.ArgumentParser(
    description="Tutorial on creating a cartpole base environment."
)
parser.add_argument(
    "--num_envs",
    type=int,
    default=1,
    help="Number of environments to spawn."
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

from wheeled_humanoid_isaaclab.tasks.moving import WheeledHumanoidEnvCfg


def main():
    env_cfg = WheeledHumanoidEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = args.device
    env = gym.make("Isaac-Wheeled-Humanoid-Moving-v0", cfg=env_cfg)

    env.reset()

    # simulate physics
    count = 0

    while simulation_app.is_running():
        with torch.inference_mode():
            # sample random actions
            joint_efforts = torch.rand(
                env.action_space.shape, device=env.unwrapped.device)

            # step the environment
            obs, rewards, terminated, truncated, info = env.step(joint_efforts)

            # update counter
            count += 1

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
