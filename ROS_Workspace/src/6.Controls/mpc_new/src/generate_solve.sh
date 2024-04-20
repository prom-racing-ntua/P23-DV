#!/bin/bash

pyenv local 3.9.18

export PYTHONPATH=/home/miltoskolovos/embotech_env:$PYTHONPATH

python3 solver_creator.py
returnCode=$?
rm -r .python-version
rm  *.forces
echo
if [ "$returnCode" -eq 0 ]
    then
        cp -r FORCESNLPsolver /home/miltoskolovos/P23-DV/ROS_Workspace/src/6.Controls/decoupled_controller/src 
    else
        echo "Solver generation failed"
fi
exit $returnCode