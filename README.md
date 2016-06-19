# Quadcopter Control

This project controls a single quadcopter simulated in Gazebo.
Separate instances can be run in parallel to control multiple quads in a single Gazebo world.
I wrote this project as an educational exercise.
Currently there is no documentation, no unit tests, and barely an code commenting.

Executable is build/src/cntrl

To run, first start my [gazebo_quadcopter_model](https://github.com/mattprompt/gazebo_quadcopter_model).
Then cntrl <model name> <model number>
```
$ cd build/src
$ ./cntrl quad_2292_1 1
$ ./cntrl quad_2292_2 2
$ ./cntrl quad_2292_3 3
```
Developed using:
- Ubuntu 14.04 LTS
- KDevelop 4
- Gazebo 5
- qgroundcontrol

Depends on:
- MavLink Headers
- BuddyBox
- Ncurses

MavLink code based on http://qgroundcontrol.org/dev/mavlink_linux_integration_tutorial
BuddyBox is a bit verbose and spews a bunch of text to the terminal screen. 
Haven't figured out how to quiet it yet.




