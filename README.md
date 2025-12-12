To run:
Step 1. colcon build
Step 2. . install/setup.bash
Step 3  .python3 src/rx200_moveit_control/launch/run.py

The flow:
1. User Input & Launch (run.py): The user runs your command-line interface (run.py) which:

    Prompts for the cube order (e.g., red, yellow, blue)
    Prompts for the place coordinates (x, y position in workspace)
    Validates inputs and launches the full ROS system via main.launch.py with these parameters

2. System Initialization (main.launch.py): When the launch file starts, it brings up components in sequence:

    Robot State Publisher: Loads the robot URDF and publishes joint states
    MoveIt: Initializes motion planning, RViz visualization, and planning scene
    Camera Static TF: Publishes the fixed transform from wrist_link → camera_link (10.5cm forward, 4cm right, 11cm up, 45° pitch)
    RealSense Camera Node: Begins streaming RGB and depth images
    #Planning Scene Updater: Adds the camera geometry to MoveIt's collision checking
    Vision Node: Starts cube detection pipeline
    Manipulation Node: Prepares arm movements (but waits for cube detection)

3. Cube Detection (vision_node.py): This node runs continuously in a perception loop:

    Synchronized frame capture: Waits for aligned RGB + depth images from RealSense
    Color-based detection: Applies HSV thresholding to find red, yellow, and blue cubes in the image
    Contour validation: Filters for square-shaped objects with correct size
    Depth unprojection: Converts pixel coordinates + depth → 3D point in camera frame
    Coordinate transform: Uses TF to transform 3D points from camera_link → rx200/base_link
    Output: Publishes detected cube positions (x, y, z) in the robot's base frame

4. Manipulation & Execution (manipulation_node.py): Once cubes are detected, the manipulation node:

    Waits for cube positions from the vision node
    Plans motion using MoveIt2 to navigate above each cube in the specified order
    Executes pick-and-place: Move to cube → descend → grip → move to place position → release
    Updates scene: Removes picked cube from collision checking so subsequent cubes are reachable
