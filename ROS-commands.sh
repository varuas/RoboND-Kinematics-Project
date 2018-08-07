# Password
robo-nd

# Initialize
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

### ROS commands
roscore
rosnode list
rostopic list
rostopic info <topic_name>

# Show the message spec
rosmsg show <package>/<msg_type>
rosed <package> <msg_type_file_name>

# Browse topic
rostopic echo <topic_name>

### Catkin commands
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# Build
cd ~/catkin_ws
catkin_make

# Create custom package
cd ~/catkin_ws/src
catkin_create_pkg <your_package_name> [dependency1 dependency2 …]

### Install dependencies
source devel/setup.bash
rosdep check simple_arm
rosdep install -i simple_arm

### ROS launch
cd ~/catkin_ws
source devel/setup.bash
roslaunch simple_arm robot_spawn.launch


### ROS Service call
rosservice call /arm_mover/safe_move "joint_1: 1.57
joint_2: 1.57"


### ROS param set
rosparam set /arm_mover/max_joint_2_angle 1.57


###################################
# Forward Kinematics
##################################
roslaunch kuka_arm forward_kinematics.launch


###################
# Launch project
###################

cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
./safe_spawner.sh
rosrun kuka_arm IK_server.py