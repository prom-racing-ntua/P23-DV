# Path Planning Spline Interpolation

This directory contains code for spline interpolation used in the path planning pipeline. The implemented algorithms are cubic and b-spline interpolation.

The *test_track* folder can hold custom tracks containing the checkpoints the car needs to pass through. These tracks can then be loaded in the code and be interpolated by the spline algorithms using the function in the *read_track.h* file.

## Requirements

<ins>Python:</ins>
- Matplotlib

<ins>C++:</ins>
- Eigen3

## How to run

1. Create a build directory to hold the compiled files
```sh
mkdir build
```
2. According to the desired algorithm run the corresponding script
```sh
# For the cubic spline algorithm
. cubic_spline.sh

# For the b-spline algorithm
. b_spline.sh
```

