#!/isaac-sim/python.sh

import subprocess
import sys

import click

CONTEXT_SETTINGS = {"ignore_unknown_options": True, "allow_extra_args": True}


@click.group()
def cli():
    """Taser command-line interface."""
    pass


@cli.command()
def install():
    """Install Taser dependencies and project."""
    subprocess.run(["bash", "/workspace/taser/install.bash"])


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
def isaaclab():
    """Commands for Isaac Lab RL training/playback."""
    pass


@isaaclab.command(context_settings=CONTEXT_SETTINGS)
@click.option("--rsl", is_flag=True, help="Use RSL RL training/playback scripts")
def train(rsl):
    """Train an RL task in Isaac Lab."""
    sys.argv = sys.argv[2:]
    if rsl:
        from taser_training.isaaclab.rl.rsl_rl.train import main as train_rsl

        train_rsl()
    else:
        from taser_training.isaaclab.rl.custom.train import main as train_custom

        train_custom()


@isaaclab.command(context_settings=CONTEXT_SETTINGS)
@click.option("--rsl", is_flag=True, help="Use RSL RL training/playback scripts")
def play(rsl):
    """Play back an RL task in Isaac Lab."""
    sys.argv = sys.argv[2:]
    if rsl:
        from taser_training.isaaclab.rl.rsl_rl.play import main as play_rsl

        play_rsl()
    else:
        from taser_training.isaaclab.rl.custom.play import main as play_custom

        play_custom()


@cli.group()
def urdf():
    """Commands for URDF file generation and conversion."""
    pass


@urdf.command()
def generate():
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
