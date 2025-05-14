import bpy
import random
import math
import os

# Clear existing objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Parameters
num_cylinders = 7
radius = 0.05  # 5 cm
min_height = 1.5
max_height = 2.0
placement_range = (7.2, 5.4)
min_distance = 2 * radius + 0.05
plane_size = 0.1

positions = []
aruco_markers = []

def is_far_enough(x, y):
    for px, py in positions:
        if math.hypot(px - x, py - y) < min_distance:
            return False
    return True

# Generate cylinders and compute marker data
for i in range(num_cylinders):
    while True:
        x = random.uniform(0, placement_range[0])
        y = random.uniform(0, placement_range[1])
        if is_far_enough(x, y):
            break

    height = random.uniform(min_height, max_height)
    bpy.ops.mesh.primitive_cylinder_add(
        radius=radius,
        depth=height,
        location=(x, y, height / 2)
    )
    cylinder = bpy.context.object
    cylinder.name = f"Cylinder_{i}"
    positions.append((x, y))

    # Plane position and orientation
    angle = random.uniform(0, 2 * math.pi)
    z_offset = random.uniform(0.1, height - 0.1)
    px = x + (radius + 0.001) * math.cos(angle)
    py = y + (radius + 0.001) * math.sin(angle)
    pz = z_offset

    roll = math.radians(90)
    pitch = 0.0
    yaw = angle + math.radians(90)

    aruco_markers.append([round(px, 3), round(py, 3), round(pz, 3),
                          round(roll, 3), round(pitch, 3), round(yaw, 3)])

# Get current .blend file directory
base_path = bpy.path.abspath("//")

# Save aruco_markers.txt
txt_path = os.path.join(base_path, "aruco_markers.txt")
with open(txt_path, "w") as f:
    f.write("aruco_markers = [\n")
    for marker in aruco_markers:
        f.write(f"    {marker},\n")
    f.write("]\n")
