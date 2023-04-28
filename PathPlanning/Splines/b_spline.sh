#!/bin/bash

CURRENT_DIR=$(pwd)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR/build
cmake ..
make
./b_spline.o

cd ..
python3 visualize.py ./build/b_spline_points.txt

cd $CURRENT_DIR