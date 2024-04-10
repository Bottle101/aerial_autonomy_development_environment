#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./devel/setup.bash
./src/vehicle_simulator/mesh/urban_city/environment/urban_city.x86_64 &
roslaunch vehicle_simulator system_unity.launch