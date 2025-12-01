import subprocess

import click


@click.group()
def cli():
    """Taser command-line interface."""
    pass


@cli.command()
def install():
    """Install Taser dependencies and project."""
    subprocess.run(["bash", "/workspace/taser/install.bash"])


@cli.command()
@click.option(
    "-t", "--type", type=click.Choice(["isaacsim", "ros"]), default="isaacsim"
)
def sim(type: str):
    """Launch the Taser simulation environment."""
    if type == "isaacsim":
        subprocess.run(["/isaac-sim/python.sh", "-m", "taser_sim.sim"])
    elif type == "ros":
        subprocess.run(["ros2", "launch", "taser_ros", "sim.launch.yaml"])


@cli.group()
def isaaclab():
    """Commands for Isaac Lab RL training/playback."""
    pass


@isaaclab.command()
@click.option("--rsl", is_flag=True, help="Use RSL RL training/playback scripts")
def train(rsl):
    """Train an RL task in Isaac Lab."""
    folder = "rsl_rl" if rsl else "custom"
    subprocess.run(
        ["/isaac-sim/python.sh", "-m", f"taser_training.isaaclab.rl.{folder}.train"],
    )


@isaaclab.command()
@click.option("--rsl", is_flag=True, help="Use RSL RL training/playback scripts")
def play(rsl):
    """Play back an RL task in Isaac Lab."""
    folder = "rsl_rl" if rsl else "custom"
    subprocess.run(
        ["/isaac-sim/python.sh", "-m", f"taser_training.isaaclab.rl.{folder}.play"],
    )


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


@urdf.command()
def convert_to_usd():
    """Convert the URDF file to USD format."""
    subprocess.run(["/isaac-sim/python.sh -m taser_sim.utils.urdf_to_usd"], shell=True)


if __name__ == "__main__":
    cli()
