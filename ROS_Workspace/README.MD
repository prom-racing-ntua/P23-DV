# P23 ROS2 Workspace

> Χέρια ψηλά για όσους βγάζουνε φράγκα από βέρια.<br>
**-Yung Light**


## Information
This is the P23's ROS Workspace. You should only see a src folder here and this README file.

This folder consists of all the ROS2 packages that P23 needs to run the DV circuit.

## Dependencies
Each package needs some dependencies installed. You can find these in the folders of the packages. Later, a script will be provided installing everything needed to run DV.

## Build
To build the P23 project, run the colcon build command while your current working directory is this folder. 

```
colcon build
. install/setup.bash
```

## Master Node
To be announced

## Velocity Estimation
Tha to grapsw argotera

## Perception
Perception consists of 3 important nodes.
- acquisition
- perception_msgs
- inference

There also exists an acquisition_logger node that saves photos locally instead of publishing them to the inference node.

It is not recommended to run the nodes by themselves. Instead, you are supposed to use the launch files.

Images explaining the architecture will be given at a later moment.

### Perception Launch
To launch normal Perception (meaning 3 Acquisition Nodes + 1 Inference Node):

```
ros2 launch Perception perception.launch.py
```

To launch Logging Perception (meaning 3 acquisition_logger nodes):

```
ros2 launch Perception perception_logger.launch.py
```

## SLAM (Simultaneous Localization and Mapping)
Tha to grapsw argotera

## Motion Planning
Tha to grapsw argotera

## Controls 
Tha to grapsw argotera

