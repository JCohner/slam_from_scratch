# Turtle Rect Package
#### By Josh Cohen

## Description:
This package interfaces with the turtlesim package to command the turtle to command velocities such that the turtle follows a rectangular path in a manner defined by the parameters in *config/turtle_rect.yaml*

## To Launch:
`roslaunch tsim trect.launch` to launch with error plotter
`roslaunch tsim trect.launch plot_gui:=false` to launch without error plotter

## File Structure:
* config/turtle_way.yaml - configuration file containing waypoint parameters for following a pentagonal path, additionall parameters include frequency of publishing and max translational and rotational velocities

* config/turtle_rect.yaml - configuration file containing rectangular path parameters as well as robot velocity parameters and frequency of velocity publishing

* launch/trect.launch - launchfile that loads configuration parameters to parameter server, launches turtlesim_node, turtle_rect node, and , optionally, the error plotter node.

* launch/turtle_pent.launch - launchfile that loads configuration parameters to parameter server, launches turtlesim_node, turtle_pent node, and , optionally, the error plotter node.

* launch/turtle_odom.launch - launchfile that loads configuration parameters to parameter server, launches turtlesim_node, turtle_pent node, error plotter, robot description (loaded from nuturtle_description), optionally a joint state publisher, fake diff encoders node, and an odom node

* msg/PoseError.msg - the msg definition file for our error messages, published on /pose_error

* src/turtle_rect.cpp - defines turtle_rect node and TurtleRect object. Performs pubs, subs, service calls, and service implementations to provide functionallity to this trajectory commanding and error tracking node

* src/turtle_way.cpp - defines turtle_way node. Performs subs, pubs, and service calls to implement pentagonal trajectory following of a turtlestim turtle

## Screenshots:
![](media/turtle_pic1.png)
![](media/trippy_turt.png)
![](media/error_plot.png)

## Link to Video:
https://youtu.be/-sAzRHv7nr8
https://www.youtube.com/watch?v=ubyYtBsvCLE
