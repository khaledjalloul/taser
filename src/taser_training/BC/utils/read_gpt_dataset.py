import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--path", type=str, required=True, help="Input file path")
args = parser.parse_args()

##############################################################

import logging
import os
import time

import numpy as np

from taser_training.BC.utils.dataset import GPTDataset


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    logger = logging.getLogger(__name__)

    dataset = GPTDataset(file_path=args.path, action_chunk_size=1)
    total_episodes = len(dataset)
    ep_lengths = np.array([])

    for ep_id in range(total_episodes):
        os.system("clear")

        episode = dataset.get_full_episode(ep_id)
        obs = episode["observations"]
        actions = episode["actions"]

        ep_lengths = np.append(ep_lengths, obs.shape[0])

        logger.info(f"Episode: {ep_id + 1} / {total_episodes}")
        logger.info(f"Observations shape: {obs.shape}")
        logger.info(f"Actions shape: {actions.shape}")
        logger.info(f"Episode length (episode {ep_id + 1}): {len(obs)}")
        logger.info(f"Mean episode length: {np.mean(ep_lengths)}")

        logger.info(f"First observation (episode 1): {obs[0]}")
        logger.info(f"First action (episode 1): {actions[0]}")

        logger.info(f"Last observation (episode {ep_id + 1}): {obs[-1]}")
        logger.info(f"Last action (episode {ep_id + 1}): {actions[-1]}")

        time.sleep(0.01)


if __name__ == "__main__":
    main()
