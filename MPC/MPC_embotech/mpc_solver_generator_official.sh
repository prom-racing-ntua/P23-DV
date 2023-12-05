pyenv local 3.9.18
if [ $# -eq 0 ]
  then
    scriptToRun=mpc_prom_skidpad.py
  else
    scriptToRun=mpc_prom_$1.py
fi

export PYTHONPATH=../../../embotech_env:$PYTHONPATH
python3 $scriptToRun $2
returnCode=$?
rm -r .python-version
rm  *.forces
echo
if [ "$returnCode" -eq 0 ]
    then
        cp -r FORCESNLPsolver ../../ROS_Workspace/src/6.Controls/mpc/src
    else
        echo "Solver generation failed"
fi
rm -r FORCESNLPsolver
exit $returnCode