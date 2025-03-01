#!/bin/python

import re
import os
import shutil

ROOT_DIR="/root/CrazySim/ros2_ws/src/icuas25_competition"

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
        copy_aruco_images(f"{ROOT_DIR}/worlds/aruco_markers/aruco_marker_1", f"{ROOT_DIR}/worlds/aruco_markers/aruco_marker_" + str(i+6))
        with open(f"{ROOT_DIR}/worlds/aruco_markers/aruco_marker_{i+6}/model.sdf", "w") as file:
            sdf_content = sdf_template.format(i+6, i+6)
            file.writelines(sdf_content)
        with open(f"{ROOT_DIR}/worlds/aruco_markers/aruco_marker_{i+6}/model.config", "w") as file:
            config_content = config_template.format(i+6)
            file.writelines(config_content)
        print(f"Completed Directory {i+6}")

# File path to your SDF file
sdf_file = f"{ROOT_DIR}/worlds/city_1_world.sdf"

# Hardcoded ArUco marker poses
aruco_markers = [
    [31.255, 10.016, 0.906, 1.571, 0.0, -3.919], 
    [48.573, 10.562, 0.636, 1.571, -0.0, -2.805], 
    [64.838, 8.825, 10.96, 1.571, -0.0, -2.302], 
    [49.532, 24.617, 11.186, 1.571, -0.0, -3.938], 
    [53.478, 53.925, 1.185, 1.571, -0.0, -3.098], 
    [26.335, 10.711, 1.231, 1.571, -0.0, -2.349], 
    [41.135, 9.422, 15.41, 1.571, -0.0, -0.242], 
    [49.575, 30.719, 6.888, 1.571, -0.0, -0.684], 
    [35.896, 30.417, 2.488, 1.571, -0.0, -2.251], 
    [44.137, 44.614, 4.143, 1.571, -0.0, -1.542], 
    [64.366, 30.728, 1.738, 1.571, -0.0, -0.788], 
    [56.831, 16.805, 0.669, 1.571, -0.0, -3.121], 
    [37.308, 23.746, 11.179, 1.571, -0.0, -1.649], 
    [48.009, 51.804, 2.508, 1.565, 0.0, 0.059], 
    [32.702, 48.371, 2.863, 1.571, -0.0, -0.782]
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
