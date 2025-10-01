#!/bin/bash

# Function to kill all background processes
cleanup() {
    echo "Stopping all processes..."
    for pid in "${pids[@]}"; do
        kill -9 "$pid" 2>/dev/null
    done
    exit 0
}

# Trap SIGINT (Ctrl+C) and run cleanup
trap cleanup SIGINT

# Start processes and save PIDs
cd ~/webpage_ws/final_project_website
python3 -m http.server 7000 &
pids+=($!)

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
pids+=($!)

ros2 run web_video_server web_video_server --ros-args -p port:=11315 &
pids+=($!)

ros2 run tf2_web_republisher_py tf2_web_republisher &
pids+=($!)

# Wait for all background processes
wait


