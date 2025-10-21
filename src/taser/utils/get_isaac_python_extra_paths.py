from pathlib import Path

import pyperclip

exts_cache_path = Path("/isaac-sim") / "extscache"
local_exts_path = Path.home() / ".local/share/ov/data/exts/v2"

paths_str = "\n"
for ext in exts_cache_path.glob("*"):
    paths_str += f"\"{ext}\",\n"

for ext in local_exts_path.glob("*"):
    paths_str += f"\"{ext}\",\n"

pyperclip.copy(paths_str)
print("Copied to clipboard:")