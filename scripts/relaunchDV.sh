#!/bin/bash

# Relaunch DV and skip an LV cycle
source /home/prom/P23-DV/scripts/setAliases.sh
resetROS
launchBase
sleep 5
launchNodes
