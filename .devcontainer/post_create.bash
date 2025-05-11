kj setup cli-completion

sudo chown -R $(whoami) .

# rosdep update
# rosdep install -iry --from-paths src 
colcon build

echo "source /workspace/colcon_ws/src/.devcontainer/utils.bash" >> ~/.bashrc
