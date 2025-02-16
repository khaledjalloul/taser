kj cli completion

sudo chown -R $USER .

rosdep update
rosdep install -iry --from-paths src 
colcon build
