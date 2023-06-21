#!/bin/bash

echo "Which solver should I generate (select number)?"
echo "(1. Autocross, 2. Acceleration, 3. Skidpad)"

read missionNumber

case $missionNumber in

    1)
    mission="autox"
    ;;

    2)
    mission="accel"
    ;;

    3)
    mission="skidpad"
    ;;

    *)
    echo "Unknown input, run again"
    exit 1
esac
echo "Generating MPC Solver for ${mission} mission"
cd ~/P23-DV/MPC/MPC_embotech
./mpc_solver_generator_official.sh $mission

generationReturnCode=$?

if [ "$generationReturnCode" -eq 0 ]
    then
        echo "Succesfully generated solver, go to your ROS_Workspace and colcon build to use"
    else
        echo "Failed to generate solver, change your generator source file and try again"
fi