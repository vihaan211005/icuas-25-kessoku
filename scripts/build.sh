#!/bin/bash

build(){
    cd /root/CrazySim/ros2_ws && \
    colcon build --symlink-install --merge-install --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select icuas25_competition 2> out || { exit 1; }

    source /root/CrazySim/ros2_ws/install/setup.bash && \
    cd /root/CrazySim/ros2_ws/src/icuas25_competition/startup && \
    ./start.sh
}

