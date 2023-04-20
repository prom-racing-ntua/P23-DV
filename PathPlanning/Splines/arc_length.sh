#!/bin/bash

CURRENT_DIR=$(pwd)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR/build
cmake ..
make
./arc_length_spline.o

cd ..
python3 visualize.py ./build/arc_length_spline_points.txt

cd $CURRENT_DIR