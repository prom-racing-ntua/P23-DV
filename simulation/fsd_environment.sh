#!/bin/bash

CATKIN_SHELL=bash

export FSD_ROOT=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)

printf "Make sure you have sourced this file and not run it\n"
printf "Sourcing ./devel/setup.bash\n"
printf "Variable FSD_ROOT set to:\n${FSD_ROOT}\n"

# check whether devel folder exists
if [ -f "${FSD_ROOT}/devel/setup.bash" ]; then
    # source setup.sh from same directory as this file
    source "/opt/ros/noetic/setup.bash"
    source "${FSD_ROOT}/devel/setup.bash"
else
    source "/opt/ros/noetic/setup.bash"
    printf "You need to build first before you can source\n"
    printf "Run 'catkin build' in the skeleton_repo directory\n"
fi

source ${FSD_ROOT}/fsd_aliases
