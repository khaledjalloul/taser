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
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

############################################################

import gymnasium as gym
import torch
from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv

from wheeled_humanoid_isaaclab.cfgs.env import WheeledHumanoidEnvCfg


def main():
    env_cfg = WheeledHumanoidEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device
    env = gym.make("Isaac-Wheeled-Humanoid-v0", cfg=env_cfg)

    # simulate physics
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # reset
            if count % 300 == 0:
                count = 0
                env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")
            # sample random actions
            joint_efforts = torch.rand(env.action_space.shape, device=env.unwrapped.device)
            # step the environment
            obs, rew, terminated, truncated, info = env.step(joint_efforts)
            # print current orientation of pole
            print("[Env 0]: Joint states: ", obs["policy"][0])
            # update counter
            count += 1

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
