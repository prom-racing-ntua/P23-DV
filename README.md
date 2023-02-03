# MPCC
This is a C++ implementation of the MPC controller with a custom dynamic model, using parameters of the P22 car.

### Solver
This version only supports hpipm as a solver. Implementation using FORCESPro to be added soon.

### Input rate cost and constraints - Dynamics
To be written

### Tire constraints
To be written

### Cost Function
To be written

## Installation 

To install all the dependencies run
```
./install.sh
```
this clones `blasfeo`, `hpipm`, `matplotlip-cpp`, `nlohmann/json`, and `eigen`, from their git repo, and safes them in a folder External. Additionally it installs `blasfeo` and `hpipm` in the same External folder, thus no admin rights are necessary.

Note that `matplotlib-cpp` does also require `Python-2.7` and `matplotlib`, for more details see (https://github.com/lava/matplotlib-cpp).

Once all dependencies are installed `cmake` can be used to build the project
```
cmake CMakeLists.txt
make
```
To run the code simply execute the `MPCC`
```
./MPCC
```
or 

```
sudo ./MPCC
```
in case an error regarding the matplotlibcpp library occurs

### TODO

There are still several things which should be added to the project. Most of them are marked with TODO in the code.
