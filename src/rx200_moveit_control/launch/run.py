import subprocess

# Allowed short codes
VALID_COLORS = {"r", "y", "b"}
# flags to check if the inputs are valid

interface = True
valid_order=False
while interface:
    if not valid_order:
        order = input("Enter cube order (e.g. r,y,b): ")

        # Validate cube order
        colors = [c.strip() for c in order.split(",") if c.strip()]

        # check if the user has inserted a valid order
        if (
        len(colors) != 3
        or any(c not in VALID_COLORS for c in colors)
        or len(set(colors)) != 3):  # no duplicates
            print("Error: cube order must be a comma-separated permutation of r,y,b, e.g. r,y,b")
            continue
        else:
            # don't ask again for the order
            valid_order=True

        x = input("Enter place X (meters): ")

        y = input("Enter place Y (meters): ")

    # confirm inputs and exit interface
    interface=False

# run launch file
cmd = [
    "ros2", "launch", "rx200_moveit_control", "main.launch.py",
    f"cube_order:={order}",
    f"place_x:={x}",
    f"place_y:={y}",
]
subprocess.run(cmd)
