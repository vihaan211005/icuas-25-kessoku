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