# nuturtle control
By Marco Morales

# Overview
The nuturtle_control package allows for the control of the Turtlebot3 in both simulation and the real world.
## Nodes
There are three nodes in this package, circle, turtle_interface and odometry.

### circle
The circle node makes the robot create circles through the use of three services that can draw based on a angular velocity, reverse the direction of the robot or stop.

### turtle_interface
turtle_interface allows for the control of the robot based on the /cmd_vel topic measurements and then publishing to /wheel_cmd and /joint_states.
### odometry

## Launchfile
The `start_robot.launch` file is the primary launchfile for the package and consists of controlling the Turtlebot3 in multiple scenarios such as controlling a simualated robot, starting a real robot from the robot itself and starting the robot from the user's computer. 

To run the launchfile, run the following line.

`roslaunch nuturtle_control start_robot.launch`

There are a few arguments that can be set when calling the launchfile and they are listed below.

For loading RVIZ set the `use_rviz` argument as either `true` or `false`.

For controlling the robot, set the `cmd_src` argument in the launchfile.

1. `circle`

    The `circle` argument will include the circle node which enables the user to call a service that draws a circle.

2. `teleop`

    The `teleop` argument will allow the user to use keyboard commands to control the robot.

3. `none`

    Use `none` if the there is another source of control for the robot.

For using the real robot or simulated one, edit the `robot` argument.

1. `nusim`

    The `nusim` node will run the nusim node which will simulate a Turtlebot3 and will appear as a red Turtlebot in RVIZ.

2. `localhost`

    The `localhost` argument will run nodes directly from the Turtlebot3.

3. `<turtlebotname>`

    The `<turtlebotname>` argument is the name of the Turtlebot that will be used, for example `april`, and will run nodes on that turtlebot.

An examples of the launchfile being called with some of the arguments above is below.

`roslaunch nuturtle_control start_robot.launch cmd_src:=circle robot:=nusim use_rviz:=true`

## Services

There are three services provided by the `circle` node that can be called.

### control

The control service will draw a circle based on two arguments, the angular velocity and the radius of the circle. It can be called by using the example below.
```
rosservice call /control "angular_velocity: 0.2
radius: 0.2"
```

### reverse

The reverse service will make the Turtlebot3 go in reverse and continue drawing the circle.
```
rosservice call /reverse "{}"
```

### stop

The stop service will stop the Turtlebot3 and stop publishing to cmd_vel
```
rosservice call /stop "{}"
```


