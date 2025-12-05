#!/isaac-sim/python.sh

import subprocess
import sys

import click

CONTEXT_SETTINGS = {"ignore_unknown_options": True, "allow_extra_args": True}


@click.group()
def cli():
    """Taser command-line interface."""
    pass


@cli.command(context_settings=CONTEXT_SETTINGS)
@click.option(
    "-t", "--type", type=click.Choice(["isaacsim", "ros", "rviz"]), default="isaacsim"
)
def sim(type: str):
    """Launch the Taser simulation environment."""
    if type == "isaacsim":
        sys.argv = sys.argv[1:]
        from taser_sim.sim import main as start_sim

        start_sim()
    elif type in ["ros", "rviz"]:
        launch_file = "sim" if type == "ros" else "rviz"
        subprocess.run(
            [
                "unset PYTHONPATH && source /opt/ros/jazzy/setup.bash && "
                f"ros2 launch taser_ros {launch_file}.launch.yaml"
            ],
            shell=True,
            executable="/bin/bash",
        )


@cli.group()
def training():
    """Commands for training Taser models."""
    pass


@training.group(name="rl", context_settings=CONTEXT_SETTINGS)
def reinforcement_learning():
    """Commands for Isaac Lab RL training/playback."""
    pass


@reinforcement_learning.command(context_settings=CONTEXT_SETTINGS)
def train():
    """Train an RL task in Isaac Lab."""
    sys.argv = sys.argv[3:]
    from taser_training.RL.train import main as train_rl

    train_rl()


@reinforcement_learning.command(context_settings=CONTEXT_SETTINGS)
def play():
    """Play back an RL task in Isaac Lab."""
    sys.argv = sys.argv[3:]
    from taser_training.RL.play import main as play_rl

    play_rl()


@training.group(name="bc", context_settings=CONTEXT_SETTINGS)
def behavior_cloning():
    """Commands for behavior cloning training/playback."""
    pass


@behavior_cloning.command(context_settings=CONTEXT_SETTINGS)
def collect_curobo_episodes():
    """Collect Curobo motion plans for behavior cloning."""
    sys.argv = sys.argv[3:]
    from taser_training.BC.curobo.collect_episodes import (
        main as collect_episodes,
    )

    collect_episodes()


@cli.group()
def urdf():
    """Commands for URDF file generation and conversion."""
    pass


@urdf.command()
def generate_from_xacro():
    """Generate the URDF file from the Xacro files."""
    subprocess.run(
        [
            "unset PYTHONPATH && source /opt/ros/jazzy/setup.bash && "
            "xacro /workspace/taser/src/taser/common/model/urdf/xacro/robot.urdf.xacro "
            "-o /workspace/taser/src/taser/common/model/urdf/taser.urdf"
        ],
        shell=True,
        executable="/bin/bash",
    )


@urdf.command(context_settings=CONTEXT_SETTINGS)
def convert_to_usd():
    """Convert the URDF file to USD format."""
    sys.argv = sys.argv[2:]
    from taser_sim.utils.urdf_to_usd import main as convert

    convert()


if __name__ == "__main__":
    cli()
