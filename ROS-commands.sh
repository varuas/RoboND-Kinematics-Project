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


##################################
# Launch Kinematics project
##################################

cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
./safe_spawner.sh
rosrun kuka_arm IK_server.py



#################################
# Segmentation Mini Project
#################################
roslaunch sensor_stick robot_spawn.launch



###################################
# Object Recognition Mini Project
###################################
cd ~/catkin_ws
roslaunch sensor_stick training.launch
rosrun sensor_stick capture_features.py
# After the above completes, execute the following : 
rosrun sensor_stick train_svm.py

roslaunch sensor_stick robot_spawn.launch
./object_recognition.py



##############################################
# Perception Project
##############################################

# Run DEMO
cd ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts
./pr2_safe_spawner.sh

# Run actual project
roslaunch pr2_robot pick_place_project.launch
cd /home/robond/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts
./pick_place_project.py 