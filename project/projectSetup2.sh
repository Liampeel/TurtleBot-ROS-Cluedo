#!/usr/bin/env bash


source ../devel/setup.bash
cd launch
roslaunch simulated_localisation.launch map_file:=$HOME/catkin_ws/src/group_project/project/example/map/project_map.yaml
cd ..