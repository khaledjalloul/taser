import subprocess
from pathlib import Path

import pyperclip


def ensure_xclip_installed():
    try:
        subprocess.run(
            ["xclip", "-version"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError:
        print("xclip not found. Installing...")
        subprocess.run(["sudo", "apt", "update"], check=True)
        subprocess.run(["sudo", "apt", "install", "-y", "xclip"], check=True)


ensure_xclip_installed()

ISAAC_SIM_PATH = Path("/isaac-sim")

paths = [
    ISAAC_SIM_PATH / "exts",
    ISAAC_SIM_PATH / "extsDeprecated",
    ISAAC_SIM_PATH / "kit/extscore",
]

paths_str = "\n"
for path in paths:
    for ext in path.glob("*"):
        paths_str += f'"{ext}",\n'

pyperclip.copy(paths_str)
print("Copied to clipboard.")
