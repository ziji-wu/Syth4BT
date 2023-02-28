#!/bin/bash

move_files_list=(
  TA2CLTLocstatistics.txt
  elementsIDmap.txt
  output.1.txt
  output.dict.txt
  output.smt.txt
  solver.txt
  tmp.zot
  counterexample.txt
  output.hist.txt
)

# change solver version

# /workspace/mitl_tack/map_utils/outputs/map1.ta
echo 'start java jar tack'
# time out after 600s or 10m or 1/6h
# timeout 600 java -jar tack.jar fischer/fischer_input_02.ta fischer/fischer_input_P0.mitli 10
timeout 600 java -jar tack.jar /workspace/mitl_tack/map_utils/outputs/map1.ta \
  /workspace/mitl_tack/map_utils/outputs/map1.mitli 20

#cp fischer/fischer_input_02.ta ./docker/env.ta
#cp fischer/fischer_input_P0.mitli ./docker/prop.mitli

# /workspace/mitl_tack/docker/result
rm -f /workspace/mitl_tack/docker/result/*
# shellcheck disable=SC2068
for file in ${move_files_list[@]}; do
  mv ./$file /workspace/mitl_tack/docker/result/$file
done
