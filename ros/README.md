#
Overview

These are the ROS packages that we'll be using for FYDP

## bee_test

Test package to run the simulator. Check [here](../README.md) for more information on how to use `bee_test`.

# Setup

Assuming you have a catkin workspace already, let's setup a symlink to add your packages.

Let's take the `bee_test` package for example.

```
ln -s Bee-Boys/ros/bee_test ~/catkin_ws/src
```

This will make a symbolic link to your package and make it easier to update packages via git and keeping the packages within your workspace.

To check that everything is working, go to your workspace directory and build:

```
cd ~/catkin_ws
catkin build
```
