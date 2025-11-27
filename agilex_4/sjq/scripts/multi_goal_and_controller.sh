#!/bin/bash

gnome-terminal --tab --title="roscore" -- bash -c "roscore; exec bash"
sleep 2
gnome-terminal --tab --title="YOLO Reader" -- bash -c "rosrun sjq read_yolo_res.py; exec bash"
sleep 1
gnome-terminal --tab --title="Scout Launch" -- bash -c "roslaunch scout_bringup scout_minimal.launch; exec bash"

echo "所有终端已启动！"
