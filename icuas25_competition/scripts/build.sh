
build(){
    cd /root/CrazySim/ros2_ws
    colcon build --symlink-install --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select icuas25_competition 2>&1 | tee out

    flag="$(cat out | grep -i error | wc -l)"
    if [ "$flag" -eq 0 ]; then
        source /root/CrazySim/ros2_ws/install/setup.bash
        cd /root/CrazySim/ros2_ws/src/icuas25_competition/startup
        ./start.sh
    else
        echo "Build failed"
    fi
}

