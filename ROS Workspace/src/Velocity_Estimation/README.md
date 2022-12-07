# P23 Velocity Estimation and Sensor Drivers

This directory contains source code for:
- Velocity Estimation
- VectorNav Sensors
- CanBus/USB to ROS Interface
- Custom ROS Msgs for all the above

The use of **Ubuntu 20.04 and ROS 2 Foxy** is assumed everywhere on this directory.

## Compiling the source code

**1. Clone the repository**
```
git clone https://github.com/prom-racing-ntua/P23-DV.git
```

**2. Move into directory**
```
cd P23-DV/Velocity_Estimation_Sensor_Drivers/
```

**3. Build files**
```
colcon build
```

**4. Install ROS workspace**
```
source install/setup.bash
```