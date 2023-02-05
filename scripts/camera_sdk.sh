#!/bin/bash

current_dir=$(pwd)

# Extract and install Camera SDK
tar xvfz GalaxyLinuxDriver.tar.gz
./GalaxyLinuxDriver/Galaxy_camera.run

cd $current_dir

# Install Python SDK
sudo apt-get install python3-setuptools libffi-dev python3-pip
echo $(pwd)
tar xvfz GalaxyPython.tar.gz && cd GalaxyPython/api
python3 setup.py build
sudo python3 setup.py install
sudo pip3 install numpy


