export PYTHONPATH=../embotech_env:$PYTHONPATH
python3 MPC/MPC_embotech/mpc_solver_generator.py
cp -r FORCESNLPsolver ROS_Workspace/src/6.Controls/mpc/src
rm -r FORCESNLPsolver
rm  *.forces