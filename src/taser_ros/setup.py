from pathlib import Path

from setuptools import find_packages, setup

package_name = "taser_ros"


def get_install_files(folder_name: str) -> list[tuple[str, list[str]]]:
    share_folder = f"share/{package_name}/"
    out = {}
    for file in Path(folder_name).rglob("*"):
        if file.is_file():
            out.setdefault(f"{share_folder}{file.parent}", [])
            out[f"{share_folder}{file.parent}"].append(str(file))
    return list(out.items())


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *get_install_files("config"),
        *get_install_files("launch"),
        *get_install_files("robot_description"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kjalloul",
    maintainer_email="khaled.jalloul@hotmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "sim = taser.ros.standalone.sim:main",
            "cmd_gui = taser.ros.standalone.cmd_gui:main",
        ],
    },
)
