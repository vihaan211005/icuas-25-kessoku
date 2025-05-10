#!/bin/bash

waitForMissionStart() {
    echo "Waiting for /mission_start to publish 'data: true'..."

    while true; do
        if ros2 topic echo /mission_start std_msgs/msg/Bool --once | grep -q "true"; then
            echo "/mission_start received 'True'. Proceeding..."
            break
        fi
        sleep 1
    done
}

