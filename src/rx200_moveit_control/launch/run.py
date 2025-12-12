import subprocess

VALID_COLORS = {"red", "yellow", "blue"}

interface = True
valid_order=False
while interface:
    if not valid_order:
        order = input("Enter cube order (e.g. red,yellow,blue): ")

        colors = [c.strip() for c in order.split(",") if c.strip()]

        if (
        len(colors) != 3
        or any(c not in VALID_COLORS for c in colors)
        or len(set(colors)) != 3):
            print("Error: cube order must be a comma-separated permutation of red,yellow,blue")
            continue
        else:
            valid_order=True

        x = input("Enter place X (meters): ")
        y = input("Enter place Y (meters): ")

    interface=False

cmd = [
    "ros2", "launch", "rx200_moveit_control", "main.launch.py",
    f"cube_order:={order}",
    f"place_x:={x}",
    f"place_y:={y}",
]
subprocess.run(cmd)