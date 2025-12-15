import subprocess
from pathlib import Path

import pyperclip

from taser.common.logger import logger


def ensure_xclip_installed():
    try:
        subprocess.run(
            ["xclip", "-version"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError:
        logger.info("xclip not found. Installing...")
        subprocess.run(["sudo", "apt", "update"], check=True)
        subprocess.run(["sudo", "apt", "install", "-y", "xclip"], check=True)


if __name__ == "__main__":
    ensure_xclip_installed()

    isaac_sim_path = Path("/isaac-sim")

    paths = [
        isaac_sim_path / "exts",
        isaac_sim_path / "extsDeprecated",
        isaac_sim_path / "kit/extscore",
    ]

    paths_str = "\n"
    for path in paths:
        for ext in path.glob("*"):
            paths_str += f'"{ext}",\n'

    pyperclip.copy(paths_str)
    logger.info("Copied to clipboard.")
