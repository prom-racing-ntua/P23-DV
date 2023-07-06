#!/bin/bash

# This script needs to be sourced in the .bashrc configuration file as to always be available. 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/.ros/cyclonedds.xml

scriptFolder="/home/prom/P23-DV/scripts"

alias launchBase="${scriptFolder}/launchP23base.sh"
alias launchNodes="${scriptFolder}/launchP23nodes.sh"
alias resetROS="${scriptFolder}/resetROS.sh"
alias relaunchDV="${scriptFolder}/relaunchDV.sh"
alias limitCPU="${scriptFolder}/limitCPU.sh"
alias generateMPCSolver="${scriptFolder}/generateMPCSolver.sh"
