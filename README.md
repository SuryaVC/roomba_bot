[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# roomba_bot
### This ROS2 package implements a simple obstacle avoidance algorithm for TurtleBot3, similar to a Roomba vacuum cleaner.

Author: Surya Chappidi <br>
UID: 119398166

### Building the ROS package
```bash
# Source ros humble
source /opt/ros/humble/setup.bash

# Go to ros2_ws/src
cd ~/ros2_ws/src

# clone the rep
git clone https://github.com/SuryaVC/roomba_bot.git

# Go back to ros2_ws
cd ..

# Build the package
colcon build --packages-select roomba_obstacle_avoidance

# Source the package
. install/setup.bash

# launch and move the robot
ros2 launch roomba_obstacle_avoidance roomba_launch.py

# to record ros bag
ros2 launch roomba_obstacle_avoidance roomba_launch.py record_topics:=True

# check the bag info
ros2 bag info roomba_bag_topics_list

# move to results directory and roomba_bag_topics_list
cd roomba_bag_topics_list

# play the bag file
ros2 bag play roomba_bag_topics_list_0.db3
```

### Dependencies
rclcpp <br>
geometry_msgs <br>
sensor_msgs

# Demo

<p align="center">
<img width="100%" alt="Result Video" src="https://github.com/SuryaVC/roomba_bot/blob/main/roomba_obstacle_avoidance/results/demo.gif">
</p>



