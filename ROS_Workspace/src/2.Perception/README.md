# P23 ROS Perception

> Τα μάτια μου σχιστά, μοιάζουν με κλειστά.<br>
**-Billy Sio**

## Perception
Package Name: **perception**

Perception consists of 2 important nodes.
- Acquisition
- Inference

There also exists an acquisition_logger node that saves photos locally instead of publishing them to the inference node.

Currently, acquisition has 3 modes:
- Acquisition (Normal acquisition with refresh rate synced by a node timer found in the cofing file)
- Acquisition with Logging (Same as normal but does not publish messages but saves the images captured)
- Saltas Acquisition. This is the final node that is sycned by the master node (saltas). 

It is not recommended to run the nodes by themselves. Instead, you are supposed to use the launch files.

Images explaining the architecture will be given at a later moment.

### Perception Launch
To launch normal Perception (meaning 3 Acquisition Nodes + 1 Inference Node):

```
ros2 launch perception perception.launch.py
```

To launch Logging Perception (meaning 3 Acquisition Logger nodes):

```
ros2 launch perception perception_logger.launch.py
```

To launch Saltas Perception (meaning 3 Acquisition nodes synced by 1 master node):

```
ros2 launch perception saltas_perception.launch.py
```