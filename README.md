# streetbot4

StreetBot4 is a simple yet powerfull outdoor 4WD diff drive robot, based on a stainless steel chasis and 8" hoverboard motor wheels.
It is equipped with an RPLIDAR S2, Intel Realsense D455 camera and 4 ultrasonic distance sensors. Motor wheels are controlled using 2 ODrive boards through CAN bus.
Robot is also rquipped with UAVCAN PEM module for monitoring battery state and consumed power and to provide 5V supply to onboard electronics.
Computational power of Streetbot4 is supplied by Raspberry Pi 4B which is dedicated to sensor reading, motor control and network communications and a Jetson Nano which is dedicated to video and lidar processing as well as navigation and planning.

## What's in the project

streetbot4_description/ - Robot description
streetbot4_gazebo/ - Gazebo simulation

## Working with robot

To run simulation:

``roslaunch streetbot4_gazebo streetbot4_office.launch``

To run model visualization and robot monitoring:

``roslaunch streetbot4_gazebo streetbot4_office.launch``
