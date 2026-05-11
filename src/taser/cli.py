#!/isaac-sim/python.sh

import subprocess
import sys

import click

IGNORE_ARGS = {
    "ignore_unknown_options": True,
    "allow_extra_args": True,
    "help_option_names": [],
}


@click.group()
def cli():
    """Taser command-line interface."""
    pass


@cli.command()
@click.option(
    "--standalone",
    is_flag=True,
    help="Whether to launch RViz without the controller node.",
)
def rviz(standalone: bool):
    """Launch RViz for Taser visualization."""
    launch_file = "rviz_controller" if not standalone else "rviz"
    subprocess.run(
        [
            "unset PYTHONPATH && source /opt/ros/jazzy/setup.bash && source /workspace/taser/install/setup.bash && "
            f"ros2 launch taser_ros {launch_file}.launch.yaml"
        ],
        shell=True,
        executable="/bin/bash",
    )


@cli.group()
def urdf():
    """Commands for URDF file generation and conversion."""
    pass


@urdf.command()
def generate_from_xacro():
    """Generate the URDF file from the Xacro files."""
    model_path = "/workspace/taser/src/taser/common/model/urdf"
    ros_package_path = "/workspace/taser/src/taser_ros"
    xacro_path = f"{model_path}/xacro/robot.urdf.xacro"
    urdf_paths = [f"{model_path}/taser.urdf", f"{ros_package_path}/config/taser.urdf"]

    for urdf_path in urdf_paths:
        subprocess.run(
            [
                "unset PYTHONPATH && source /opt/ros/jazzy/setup.bash && "
                f"xacro {xacro_path} -o {urdf_path}"
            ],
            shell=True,
            executable="/bin/bash",
        )


@urdf.command(context_settings=IGNORE_ARGS)
def convert_to_usd():
    """Convert the URDF file to USD format."""
    sys.argv = sys.argv[2:]
    from taser_sim.utils.urdf_to_usd import main

    main()


@cli.command(context_settings=IGNORE_ARGS)
def sim():
    """Launch the Taser simulation environment."""
    sys.argv = sys.argv[1:]
    from taser_sim.sim import main

    main()


@cli.group()
def training():
    """Commands for training Taser models."""
    pass


@training.group(name="rl")
def reinforcement_learning():
    """Commands for Isaac Lab RL training/playback."""
    pass


@reinforcement_learning.command(name="train", context_settings=IGNORE_ARGS)
def train_rl():
    """Train an RL task in Isaac Lab."""
    sys.argv = sys.argv[3:]
    from taser_training.RL.train import main

    main()


@reinforcement_learning.command(name="play", context_settings=IGNORE_ARGS)
def play_rl():
    """Play back an RL task in Isaac Lab."""
    sys.argv = sys.argv[3:]
    from taser_training.RL.play import main

    main()


@training.group(name="bc")
def behavior_cloning():
    """Commands for behavior cloning training/playback."""
    pass


@behavior_cloning.command(context_settings=IGNORE_ARGS)
def collect_curobo_episodes():
    """Collect Curobo motion plans for behavior cloning."""
    sys.argv = sys.argv[3:]
    from taser_training.BC.curobo.collect_episodes import main

    main()


@behavior_cloning.command(name="train", context_settings=IGNORE_ARGS)
def train_bc():
    """Train the GPT model with behavior cloning."""
    sys.argv = sys.argv[3:]
    from taser_training.BC.train import main

    main()


@behavior_cloning.command(name="play", context_settings=IGNORE_ARGS)
def play_bc():
    """Evaluate the GPT model."""
    sys.argv = sys.argv[3:]
    from taser_training.BC.play import main

    main()


@behavior_cloning.command(context_settings=IGNORE_ARGS)
def read_dataset():
    """Read and print information about the GPT dataset."""
    sys.argv = sys.argv[3:]
    from taser_training.BC.utils.read_gpt_dataset import main

    main()


if __name__ == "__main__":
    cli()
