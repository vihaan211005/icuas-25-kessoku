#!/usr/bin/bash

mkdir -p $HOME/CrazySim/ros2_ws/src/icuas25_competition/ext && cd $HOME/CrazySim/ros2_ws/src/icuas25_competition/ext

git clone --recursive https://github.com/whoenig/uav_trajectories.git traj
mkdir -p traj/build && cd traj/build
cmake ..
make 

echo "export TRAJ_GEN=/root/CrazySim/ros2_ws/src/icuas25_competition/ext/traj/build/genTrajectory" >> $HOME/.bashrc