#!/bin/bash

# Relaunch DV and skip an LV cycle
source /home/prom/P23-DV/scripts/setAliases.sh
/home/prom/P23-DV/scripts/./resetROS.sh
/home/prom/P23-DV/scripts/./launchP23nodes.sh
sleep 5
/home/prom/P23-DV/scripts/./launchP23base.sh