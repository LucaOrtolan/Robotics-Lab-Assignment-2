# Implementation Description:

 In this project, we focused on improving the ROS2-based control for the RX200 robotic arm to carry out pick and place operations. We changed the original code to add waypoint-based movement, gripper control, and synchronization between motions. These updates made the overall process smoother and more reliable.

# Build Configuration Description:

Build and source the setup file by:

...cd directory to ros assignment

colcon build
source install/setup.bash

# Run Commands:

Activate the robot with:
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=actual

To launch the default setup:
ros2 run rx200_moveit_control  rx200_moveit_client

To run a particular pick and place coordinate
ros2 run rx200_moveit_control rx200_moveit_client --pick_x 0.3 --pick_y 0.1 --place_x -0.5 --place_y -0.2

# Detailed Description

    1. We added parameters to take input points (x, y) for pick and place operations. 

    2. These points are divided into six way points, including a hover position (z_hover) to regulate the height of the end effector and making movement smoother. 

    3. The gripper now has open and close positions defined based on the way point where the robot is currently located. 

    4. To prevent fast command execution errors, we added a motion_done flag to confirm that each movement is complete before the next one starts. 

    5. Pitch and yaw values are calculated using Python’s math package. References are included below. 

    6. Control groups were introduced to separately activate the arm and gripper nodes when necessary. 
       
    7. Implemented checks for edge values for x and y coordinates. 

References used for pitch and yaw:

    • https://bukkit.org/threads/tutorial-how-to-calculate-vectors.138849

    • https://uk.mathworks.com/matlabcentral/answers/298940/how-to-calculate-roll-pitch-and-yaw-from-xyz-coordinates-of-3-planar-points

# Parsing Coordinates

The node accepts coordinates for the pick and place task. It automatically converts them into six safe waypoints and executes them one by one. It controls the gripper at the right moments.

# Team Members
    • Rishabh Jain
    • Luca Ortolan
    • Saandeep Guddanti
