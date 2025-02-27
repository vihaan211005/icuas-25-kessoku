import re
import os
import shutil

def copy_aruco_images(source_dir, target_dir):
    """
    Copies only the image files from aruco_marker_x directories in source_dir
    to newly created aruco_marker_x directories in target_dir.
    
    :param source_dir: Path to the directory containing aruco_marker_x folders.
    :param target_dir: Path to the directory where new aruco_marker_x folders will be created.
    """
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    file = "marker_0001.png"
    shutil.copy(os.path.join(source_dir, file), os.path.join(target_dir, file))
    print(f"Copied {file} to {target_dir}")

def mkdir(n):
    
    sdf_template = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="aruco_marker_{}">
    <static>true</static>
    <link name="link">
      <!-- ArUco Marker 0 -->
      <visual name="aruco_marker_{}_vis">
        <pose>0 0 0 0 0 0</pose> <!-- Position: x, y, z; Rotation: roll, pitch, yaw -->
        <geometry>
          <plane>
            <normal>0 0 1</normal> <!-- Normal vector for the plane -->
            <size>0.25 0.25</size>   <!-- Size of the marker (e.g., 10cm x 10cm) -->
          </plane>
        </geometry>
        <material>
          <pbr>
            <metal>
              <albedo_map>marker_0001.png</albedo_map>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    config_template = """<?xml version="1.0" ?>
<model>
  <name>aruco_marker_5</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>
    A set of ArUco markers.
  </description>
</model>"""

    for i in range(n):
        copy_aruco_images("worlds/aruco_markers/aruco_marker_1", "worlds/aruco_markers/aruco_marker_" + str(i+6))
        with open(f"worlds/aruco_markers/aruco_marker_{i+6}/model.sdf", "w") as file:
            sdf_content = sdf_template.format(i+6, i+6)
            file.writelines(sdf_content)
        with open(f"worlds/aruco_markers/aruco_marker_{i+6}/model.config", "w") as file:
            config_content = config_template.format(i+6)
            file.writelines(config_content)
        print(f"Completed Directory {i+6}")

# File path to your SDF file
sdf_file = "worlds/city_1_world.sdf"

# Hardcoded ArUco marker poses
aruco_markers = [
    [54.262, 28.256, 1.573, 1.571, 0.0, -3.919], 
    [84.329, 29.798, 1.104, 1.571, -0.0, -2.805], 
    [112.566, 24.897, 19.027, 1.571, -0.0, -2.302], 
    [85.993, 69.449, 19.42, 1.571, -0.0, -3.938], 
    [92.844, 152.132, 2.057, 1.571, -0.0, -3.098], 
    [45.72, 30.217, 2.137, 1.571, -0.0, -2.349], 
    [71.415, 26.58, 26.753, 1.571, -0.0, -0.242], 
    [86.067, 86.665, 11.959, 1.571, -0.0, -0.684], 
    [62.32, 85.812, 4.32, 1.571, -0.0, -2.251], 
    [76.627, 125.865, 7.194, 1.571, -0.0, -1.542], 
    [111.746, 86.69, 3.017, 1.571, -0.0, -0.788], 
    [98.665, 47.411, 1.162, 1.571, -0.0, -3.121], 
    [64.77, 66.993, 19.409, 1.571, -0.0, -1.649], 
    [83.349, 146.149, 4.355, 1.565, 0.0, 0.059], 
    [56.775, 136.462, 4.971, 1.571, -0.0, -0.782],
]

def add_markers():
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

mkdir(10)
add_markers()
