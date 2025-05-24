kj setup cli-completion

sudo chown -R $(whoami) .

# rosdep update
# rosdep install -iry --from-paths src

colcon build --packages-ignore wheeled_humanoid_isaaclab
/home/vscode/venv/bin/pip install -e src/isaaclab

echo "source /workspace/colcon_ws/.devcontainer/utils.bash" >> ~/.bashrc
