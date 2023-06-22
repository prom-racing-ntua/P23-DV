#!/bin/bash

echo "Which solver should I generate (select number)?\n"
echo "(1. Trackdrive, 2. Acceleration, 3. Skidpad)\n"

read missionNumber

case $missionNumber in

    1)
    mission="trackdrive"
    ;;

    2)
    mission="acceleration"
    ;;

    3)
    mission="skidpad"
    ;;

    *)
    echo "Unknown input, run again"
    exit
esac
echo "Generating MPC Solver for ${mission} mission\n"
cd ~/P23-DV/MPC/MPC_embotech
mpc_solver_generator_official.sh $mission

generationReturnCode=$?
if [generationReturnCode -eq 0]
    then
        echo "Succesfully generated solver, go to your ROS_Workspace and colcon build to use"
    else
        echo "Failed to generate solver, change your generator source file and try again"
fi