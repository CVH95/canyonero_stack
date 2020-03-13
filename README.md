# Canyonero Stack

Contents:

- C++ Library for using Canyonero.
- ROS driver (C++ written).
- Msg interface.
- Documentation.

Detailed guide available in the project's [wiki](https://github.com/CVH95/Canyonero/wiki).

## Canyonero library

GPIO management library.

## Canyonero ROS driver

This package provides ROS drivers of the robot. Motion driving is linked to GPIO library and can be easily set up for keyboard/joystick/autonomous control. Camera feed is also accessed from a remote ROS controller.

It requires ROS Kinetic installed on Raspberry Pi. For now, only Kinetic has been fully tested.

## Future projects

- Canyonero simulation (Gazebo and Rviz)
- Navigation

## Author

- @CVH95

![Canyonero wallpaper](doc/img/wallpaper.jpg)