from setuptools import find_packages, setup

setup(
    name="wheeled_humanoid_isaaclab",
    version="0.0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"}
)
