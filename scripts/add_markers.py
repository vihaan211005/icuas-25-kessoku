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
sdf_file = f"{ROOT_DIR}/worlds/city_1_small_world.sdf"

# Hardcoded ArUco marker poses
aruco_markers = [
    [32.624, 78.667, 2.768, 1.571, -0.0, -0.814], 
    [47.495, 84.149, 2.413, -1.71, -0.007, 3.14], 
    [52.331, 63.819, 12.129, 1.571, -0.0, -1.491], 
    [72.0, 21.467, 9.239, 1.571, -0.0, -2.32], 
    [33.296, 25.196, 1.209, 1.571, -0.0, 4.372], 
    [19.611, 21.426, 1.244, 1.571, -0.0, -0.806], 
    [35.654, 21.547, 9.348, 1.571, -0.0, 1.7], 
    [48.5, 14.249, 2.335, 1.571, -0.0, 5.672], 
    [59.403, 84.909, 3.508, 1.571, -0.0, 2.315], 
    [51.525, 70.316, 3.803, 1.571, -0.0, 3.175], 
    [37.717, 49.693, 10.92, 1.571, -0.0, 3.005], 
    [56.138, 40.36, 5.785, 1.571, -0.0, 7.242], 
    [72.729, 44.92, 3.599, 1.571, -0.0, 7.304], 
    [72.729, 44.92, 3.599, 1.571, -0.0, 7.304], 
    [23.667, 63.025, 5.829, 1.571, -0.0, 4.689],
    [5.6583, 32.881, 4.1206, 1.531, 0.005, -0.8]
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
        if "<uri>model://city_1_small</uri>" in line:
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

    print("ArUco markers added successfully to city_1_small_world.sdf!")

mkdir(11)
add_markers()
