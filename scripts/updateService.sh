#!/bin/bash
scriptFolder="/home/prom/P23-DV/scripts"

# 3. Launch ROS2 Nodes
cd $scriptFolder
sudo cp p23_init.service /etc/systemd/system
sudo cp p23_ev_init.service /etc/systemd/system
sudo systemctl daemon-reload
