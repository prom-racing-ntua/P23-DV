# MPCC
This is a Python software implementation of the MPC controller with a custom hybrid (kinematic-dynamic) model, using parameters of the P22 car.

### Solver
This version uses the FORCESPro solver 6.0.1 version. Users with license are connected to the forces.embotech.com server and results are being sent back to them.

### Input rate cost and constraints - Dynamics
To be written

### Tire constraints
To be written

### Cost Function
To be written

### License installation
To be analytically written. For license holders, it is also included on the website of FORCESPro, at the "What information do i need to submit?" section.

### MPC Execution

```
pip3 install -r P23-DV/MPC/MPC_embotech/requirements.txt
sudo apt-get install gcc libomp-dev
export PYTHONPATH="path/to/embotech":$PYTHONPATH
python3 P23-DV/MPC/MPC_embotech/mpc_prom_v2.py
```

or (for updated version)

```
pip3 install -r P23-DV/MPC/MPC_embotech/requirements.txt
sudo apt-get install gcc libomp-dev
export PYTHONPATH="path/to/embotech":$PYTHONPATH
python3 P23-DV/MPC/MPC_embotech/mpc_prom_v2.py
```


