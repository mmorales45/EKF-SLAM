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
The odometry node publishes odometry messages and transforms between the robot and odom frame.

## Launchfile
The `start_robot.launch` file is the primary launchfile for the package and consists of controlling the Turtlebot3 in multiple scenarios such as controlling a simulated robot, starting a real robot from the robot itself and starting the robot from the user's computer. 

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

###

The set_pose service will teleport the blue robot to a specified x,y and theta. It can be called by looking at the example below.
```
rosservice call /set_pose "x: 0.0
y: 0.0
theta: 0.0" 

```
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

## Videos and Analysis

### Only Translation

Real robot video.

https://youtu.be/jvZxpx4sIiQ

Simulated robot video.

https://youtu.be/I8frM0yJ3rw

For purely translational, the Turtlebot was controlled by the teleop node to go both straight forward and backward. For this, the robot's results were recorded for both the real robot and the simulated one. 

After going back and forth a few times, the real robot and the simulated robot's odometry aligned throughout the duration of the test eventually both ending at the same location where they started.

Given that the robot start at approximately x = 0 and y = -0.05, it landed at the origin point.
```
header: 
  seq: 72379
  stamp: 
    secs: 1644609061
    nsecs: 583225727
  frame_id: "odom"
child_frame_id: "blue-base_footprint"
pose: 
  pose: 
    position: 
      x: 0.02070790219949646
      y: -0.0510591881764680814
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.027047493389353926
      w: 0.9996341496274289
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```
### Only Rotation

Real robot video.

https://youtu.be/mMWQAFHkrzs

Simulated robot video.

https://youtu.be/mHmKkRkq2wg

For purely rotational, the Turtlebot was again controlled by the teleop node to rotate clockwise and counter clockwise and the results were recorded. 

After both robots completed multiple rotations in both directions, both robots's orientations were aligned throughout the duration of the test and the end position match the initial position. 

Given that the robot start at approximately z =0, it landed at the origin point.
```
---
header: 
  seq: 22203
  stamp: 
    secs: 1644609351
    nsecs: 848015376
  frame_id: "odom"
child_frame_id: "blue-base_footprint"
pose: 
  pose: 
    position: 
      x: -9.920217888198787e-05
      y: -0.0500378462197785
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.022145037114154133
      w: 0.9997547685963857
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```
### Circles

Real robot video.

https://youtu.be/pr3f19LJdqQ

Simulated robot video.

https://youtu.be/9V1hSmQoRU0

For making circles, the Turtlebot made a circle of radius 0.5 meters after going both and forth throughout the test. 

After the test and the real and simulated robot making a few circles, both robots were along the same position throughout the path and both ended where they began. 

Given that the robot start at approximately z =0, it landed at the origin point.
```
---
header: 
  seq: 67530
  stamp: 
    secs: 1644609581
    nsecs: 120419305
  frame_id: "odom"
child_frame_id: "blue-base_footprint"
pose: 
  pose: 
    position: 
      x: 0.007813922009871853
      y: -0.04984820890296897
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.009903585360916928
      w: 0.9999509582959552
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

### Worse Result for Circles

Real robot video.

https://youtu.be/5fPhm5tGRvs

Simulated robot video.

https://youtu.be/wfxXyTNY4jc

For getting different results, I ran the circle test again but this time making the robot complete the same circle but much faster then the previous test. 

The results are based on the real robot's start position. When the real robot reached it's starting position, the test was stopped and the simulated Turtlebot's odometry was not its starting position. This most likely occurred due to the wheels slipping as its moving faster. Slower speeds led to more accurate results between the simulated and real robot. 

Given that the robot start at approximately x = -0.02 and y = -0.05, it was not close to landing on its original location.
```
header: 
  seq: 118344
  stamp: 
    secs: 1644609682
    nsecs: 748437576
  frame_id: "odom"
child_frame_id: "blue-base_footprint"
pose: 
  pose: 
    position: 
      x: 0.39609572681925037
      y: 0.14381750297398824
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.43766391530775417
      w: 0.8991386418331085
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```
