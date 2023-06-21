pyenv local 3.10.0
if [ $# -eq 0 ]
  then
    scriptToRun=mpc_solver_generator.py
  else
    scriptToRun=mpc_prom_$1.py
fi

export PYTHONPATH=../../../embotech_env:$PYTHONPATH
python3 $scriptToRun
returnCode=$?

if [ "$returnCode" -eq 0 ]
    then
        cp -r FORCESNLPsolver ../../ROS_Workspace/src/6.Controls/mpc/src
        rm -r .python-version
        rm -r FORCESNLPsolver
        rm  *.forces
    else
        echo "Solver generation failed"
fi

exit $returnCode