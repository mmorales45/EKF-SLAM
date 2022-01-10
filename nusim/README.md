# nusim
By Marco Morales

## Overview
The nusim package allows the simualtion of a turtlebot3 that can be viewed in RVIZ. In this simualation, there are a number of obstacles in the enviornment that are presented as red cylinders.

### Launchfile
The launchfile, `nusim.launch`, is the main launchfile for the package and it runs the simulation node, `nusim.cpp`, RVIZ, and loads the yaml file for the parameters. There are arguments within the launchfile that the user can define when running the launchfile.

To run the launchfile with the arguments all set to the default, run the following run.
`roslaunch nusim nusim.launch`

Arguments can be set in the launchfile line as can be seen in the example below.
`roslaunch nusim nusim.launch rate:=100`

#### Arguments 
There are a number of arguments that can be defined by the user.

`rate`
The parameter rate sets the frequency of the main loop in the node.

### Services
There are two services that can be called in the package.

The first service is `reset`. This service sets the timestep to 0 and sets the turtlebot's position to (0,0,0). To call the service, see below.
```
rosservice call /nusim/reset
```

The next service is 'teleport' and it moves the position of the turtlebot where the user set when calling the service.
below.
```
rosservice call /nusim/teleport {}
```
