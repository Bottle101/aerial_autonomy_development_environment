#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./devel/setup.bash
./src/vehicle_simulator/mesh/factory/environment/factory.x86_64 &
roslaunch vehicle_simulator system_unity.launch