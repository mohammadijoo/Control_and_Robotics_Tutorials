# ROS2: create a pinned .repos file from current workspace
cd ~/ros2_ws/src
vcs export --exact > ros2_ws.repos

# Later (or on a different machine), reproduce the same sources:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
vcs import src < ros2_ws.repos
colcon build --symlink-install
      
