import re

# File path to your SDF file
sdf_file = "worlds/city_1_world.sdf"

# Hardcoded ArUco marker poses
aruco_markers = [
    (19.499, 54.745, 1.7988, 1.5708, 0, -3.1416),
    (25.233, 73.555, 10.522, 1.5708, 0, 0.807),
    (45.339, 102.54, 20.896, 1.5708, 0, 1.795),
    (83.253, 87.747, 0.72931, 1.5708, 0, 3.8275),
    (63.303, 12.454, 20.22, 1.5708, 0, 5.8486),
]

# ArUco marker template
marker_template = """    <include>
      <pose>{} {} {} {} {} {}</pose>
      <uri>model://aruco_markers/aruco_marker_{}</uri>
    </include>"""

# Read the original SDF file
with open(sdf_file, "r") as file:
    sdf_content = file.readlines()

# Find the index of the city model include
for i, line in enumerate(sdf_content):
    if "<uri>model://city_1</uri>" in line:
        insert_index = i + 2  # Insert after this line
        break

# Generate ArUco markers
aruco_insertions = "\n".join(
    marker_template.format(*pose, idx + 1) for idx, pose in enumerate(aruco_markers)
) + "\n"

# Insert ArUco markers into the SDF content
sdf_content.insert(insert_index, aruco_insertions)

# Write back the modified file
with open(sdf_file, "w") as file:
    file.writelines(sdf_content)

print("ArUco markers added successfully to city_1_world.sdf!")
