#!/bin/bash
ENV="factory_low_resolution" #factory, villege, urban_city, town, old_town, office1, office2
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"


cd $SCRIPT_DIR
source ./devel/setup.bash
./src/vehicle_simulator/mesh/"$ENV"/environment/"$ENV".x86_64 &
roslaunch vehicle_simulator system_unity.launch
