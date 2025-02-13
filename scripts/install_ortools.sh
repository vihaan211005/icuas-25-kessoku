#!/usr/bin/bash

mkdir -p $HOME/CrazySim/ros2_ws/src/icuas25_competition/ext && cd $HOME/CrazySim/ros2_ws/src/icuas25_competition/ext

wget https://github.com/google/or-tools/releases/download/v9.11/or-tools_amd64_ubuntu-22.04_cpp_v9.11.4210.tar.gz 
tar -xvzf or-tools_amd64_ubuntu-22.04_cpp_v9.11.4210.tar.gz
mv or-tools_x86_64_Ubuntu-22.04_cpp_v9.11.4210 ortools
rm or-tools_amd64_ubuntu-22.04_cpp_v9.11.4210.tar.gz
mv $HOME/CrazySim/ros2_ws/src/icuas25_competition/to_move/simple_routing_program.cc ortools/examples/simple_routing_program/.
mv $HOME/CrazySim/ros2_ws/src/icuas25_competition/to_move/waypoints.csv /.
cd ortools && make run SOURCE=examples/simple_routing_program/simple_routing_program.cc

echo "export TSP_SOLVER=/root/CrazySim/ros2_ws/src/icuas25_competition/ext/ortools/examples/simple_routing_program/build/bin/simple_routing_program" >> $HOME/.bashrc 



