#!/bin/bash

echo "Which solver should I generate (select number)?"
echo "(1. Acceleration, 2. Skidpad, 3. Autocross, 4. Trackdrive, 5. Curv)"

read missionNumber

case $missionNumber in

    1)
    mission="accel"
    ;;

    2)
    mission="skidpad"
    ;;

    3)
    mission="autox"
    ;;

    4)
    mission="trackdrive"
    ;;

    5)
    mission="curv"
    ;;

    *)
    echo "Unknown track input, run again"
    exit 1
esac
echo
echo "Do you want simulation(select number)?"
echo "(0. No simulation 1. Simulation)"
read simulationNumber
case $simulationNumber in
    0)
    simulation="no_simulation"
    ;;

    1)
    simulation="one_simulation"
    ;;

    *)
    echo "Unknown siumlation input, run again"
esac
echo
echo "Generating MPC Solver for ${mission} mission. I also generate ${simulation}."
cd ~/P23-DV/MPC/MPC_embotech
./mpc_solver_generator_official.sh $mission $simulation
generationReturnCode=$?

cd ../.. 

if [ "$generationReturnCode" -eq 0 ] 
    then
        echo "Do you want immediate build of mpc node?"
        echo "(0. No build 1. Build)"
        read buildNumber
        echo "build number is: ${buildNumber}"
        case $buildNumber in
            0)
            echo "Succesfully generated solver, but you need to go to your ROS_Workspace and colcon build."
            ;;

            1)
            cd ROS_Workspace
            colcon build --packages-select mpc
            echo "Succesfully generated solver and build mpc node."
            ;;
            
            *)
            echo "Unknown build input, run again"
        esac
    else
        echo "Failed to generate solver, change your generator source file and try again"
fi