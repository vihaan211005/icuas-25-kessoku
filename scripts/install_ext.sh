#!/usr/bin/bash

git clone https://github.com/danfis/libccd.git
mkdir -p libccd/build
cd libccd/build
cmake ..
make install

cd $HOME
git clone https://github.com/flexible-collision-library/fcl.git
mkdir -p fcl/build
cd fcl/build
cmake ..
make install

cd $HOME
git clone --recursive https://github.com/whoenig/uav_trajectories.git
mkdir uav_trajectories/build
cd uav_trajectories/build
cmake ..
make 

wget https://github.com/google/or-tools/releases/download/v9.11/or-tools_amd64_ubuntu-24.04_cpp_v9.11.4210.tar.gz
tar -xvzf or-tools_amd64_ubuntu-24.04_cpp_v9.11.4210.tar.gz
mv or-tools_x86_64_Ubuntu-24.04_cpp_v9.11.4210  or_tools


