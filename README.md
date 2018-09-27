# README #

Interface for teleoperating a robotic manipulator using the geomagic device.

Requires the geomagic drivers, SDK and ROS node.

Example launch command:
```
roslaunch geomagic_manipulator_teleoperation geomagic_teleop.launch robot_name:=kuka geomagic_button:=phantom/button geomagic_pose:=phantom/pose geomagic_button:=/phantom/button
```


