#!/bin/bash

# 'run.sh' supplies a unified script for:
#                                   1. Run csvMap2TA.py and Parse csv maps;
#                                   2. Run TACK (in docker) to get the planning path.
#

# Stage 1: Run 'csvMap2TA.py'
#
# Absolute Path of 'csvMap2TA.py' :
#       /home/devel_ws/src/mitl_tack/map_utils/map_utils/csvMap2TA.py
#
# Parms:
#       -f [csv_map_file] : csv map file path, default map file: None.
#                (x) '/home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/map1.csv'
#
#
#        -d [csv_maps_dir] : csv map files dir, default map file: None.
#                (x) '/home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/'
#
#        -h : usage
#
# Result Path:
#           .ta/.mitli/.npz : '/home/devel_ws/src/mitl_tack/map_utils/outputs/'
#           .png : '/home/devel_ws/src/mitl_tack/map_utils/outputs_map_figures'

/bin/python3 /home/devel_ws/src/mitl_tack/map_utils/map_utils/csvMap2TA.py \
    -f /home/devel_ws/src/ros2_explorer/explorer_gazebo/maps/map1.csv


# Stage 2: Run TACK (in docker)
# 
# 1. Mount Local Path: /home/devel_ws/src/mitl_tack/    
#     to  Docker Path: /workspace/mitl_tack
# 
# 2. Dir Tree in Docker (after mounted):
# /
#   /workspace
#       /onStart.sh
#       /
#       /...
#       /mitl_tack (mounted)
#           /docker
#               ...
#           /map_utils
#               ...
#
# 3. Sequence of Running:
#       run.sh (currnet) -> csvMap2TA.py -> onStart.sh(when docker started) -> run_in_docker.sh

sudo docker run \
            --rm \
            --privileged \
            -v /home/devel_ws/src/mitl_tack/:/workspace/mitl_tack \
            --name tack \
            -it i/tack bash /workspace/onStart.sh \
            -s -- \
            -p x86_64