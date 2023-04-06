# P23 ROS2 Workspace

> Χέρια ψηλά για όσους βγάζουνε φράγκα από βέρια.<br>
**-Yung Light**

## Information
This is the P23's ROS Workspace. You should only see a src folder here and this README file.

This folder consists of all the ROS2 packages that P23 needs to run the DV circuit.

## Dependencies
Each package needs some dependencies installed. You can find these in the folders of the packages. Later, a script will be provided installing everything needed to run DV.

The P23 System is running using Ubuntu 20.04 as its OS and ROS2 Foxy as the ROS disto.

## Build
To build the P23 project, run the colcon build command while your current working directory is this folder. 

```
colcon build
. install/setup.bash
```

## Design Choices - Common Patterns
These are some of the rules that we followed such as naming schemes e.t.c.

- Node names are simple and do not include the words "node", "handler" or the language that they were written in.
- Topics/Services/Actions are named like this: {node_name}/{{topic,service,action}_name} e.g. /lifecycle_manager/change_dv_state
