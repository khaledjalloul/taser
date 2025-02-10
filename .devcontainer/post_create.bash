sudo chown -R $USER .

colcon build

echo "source /workspace/colcon_ws/install/setup.bash" >> ~/.bashrc

echo "yaml file:///workspace/colcon_ws/src/rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/1-custom-deps.list

echo 'export PYTHONPATH=$PYTHONPATH:/home/vscode/venv/lib/python3.12/site-packages' >> /home/vscode/venv/bin/activate

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

source /home/vscode/venv/bin/activate && pip install --no-input cvxpy ecos
