This package contains code to run the RX200 interbotix robot arm to perform pick and place using coloured cubes. The project leverages the cv2 python library to extract cubes coordinates in the workspace. Vision is enabled by an Intel RealSense D415 camera mounted on top of the arm. The code also uses the Interbotix MoveIt package to initialize the camera and send movement commands to the arm.

To run the code, navigate to the folder's path and run the following commands:

`colcon build`

`source install/setup.bash`

Execute the `run.py` script using:

`python3 src/rx200_moveit_control/launch/run.py`

The terminal will ask for the cubes' pick-up order and the x-y coordinates of the place point. It validates the inserted parameters and if incorrect, it will ask for them again. 
Once verified, the script will call the ROS launch file: `main.launch.py`.

The launch file incorporates the following nodes:
- `robot_state_publisher` loads the robot's modified URDF file (that includes the camera mount mesh) and publishes the joint states
- `moveit_lTo run the code, naunch` calls the moveit package launch file from interbotix that enables movement for the robot
- `camera_mount_tf` publishes the fixed transformation of the camera frame, it's needed to perform inverse kinematics using the camera POV as center point
- `realsense_node` initializes the realsense D415 camera mounted on the wrist
- `vision_node` performs color detection and publishes the cubes coordinates for the manipulation node
- `manipulation_node` regulates robot movement: it first moves upright to get a better view of the workspace and, once obtained the coordinates, it performs the pick and place motion 

The `planning_scene_node` was originally being used for adding camera geometry for moveit but it was later replaced by the updated URDF file, called by the `robot_state_publisher`

Group members:
- Rishabh Jain 
- Luca Ortolan
- Saandeep Guddanti