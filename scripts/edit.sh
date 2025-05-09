#!/bin/bash

# Define the files that require "1000" to "500" replacements
FILES_TO_REPLACE_1000_500=(
    "/root/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf.jinja"
    "/root/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie_thrust_upgrade/model.sdf.jinja"
)

# Define world files that need additional modifications
WORLD_FILES=(
    "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/empty.sdf"
    "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1_world.sdf"
    "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1_small_world.sdf"
    "/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/simple_world.sdf"
)

# Replace "1000" with "500" in the specified files
for FILE in "${FILES_TO_REPLACE_1000_500[@]}"; do
    if [[ -f "$FILE" ]]; then
        sed -i 's/1000/500/g' "$FILE"
        echo "Successfully replaced '1000' with '500' in $FILE"
    else
        echo "Error: File not found at $FILE"
    fi
done

# Modify world files with specific replacements
for WORLD_FILE in "${WORLD_FILES[@]}"; do
    if [[ -f "$WORLD_FILE" ]]; then
        sed -i 's|<max_step_size>0.001</max_step_size>|<max_step_size>0.002</max_step_size>|g' "$WORLD_FILE"
        sed -i 's|<cast_shadows>true</cast_shadows>|<cast_shadows>false</cast_shadows>|g' "$WORLD_FILE"
        sed -i 's|<shadows>true</shadows>|<shadows>false</shadows>|g' "$WORLD_FILE"

        echo "Successfully updated parameters in $WORLD_FILE"
    else
        echo "Error: World file not found at $WORLD_FILE"
    fi
done
