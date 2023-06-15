pyenv local 3.10.0
export PYTHONPATH=../../../embotech_env:$PYTHONPATH
python3 mpc_solver_generator.py
cp -r FORCESNLPsolver ../../ROS_Workspace/src/6.Controls/mpc/src
rm -r .python-version
rm -r FORCESNLPsolver
rm  *.forces