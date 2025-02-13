RUN kj cli completion

sudo chown -R $USER .

rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
