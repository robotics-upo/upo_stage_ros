# upo_stage_ros
A modified version of the regular ROS wrapper for the Stage robot simulator


This version has been modified in order to publish the people detected (blocks or other stage models) using the next messages:

upo_msgs/PersonPoseArrayUPO:

	Header header
	uint32 size
	PersonPoseUPO[] personPoses


upo_msgs/PersonPoseUPO:

	Header header
	uint32 id
	float64 vel
	geometry_msgs/Point position 
	geometry_msgs/Quaternion orientation 

The topic is: /people/navigation.
The reference frame in which the people coordinates are published is /odom


The simulator is also subscribed to the topic "initialpose".
This way, when someone publishes a new "initialpose", for instance,
the RViz "set initial pose" function, the position of the robot
in the simulator will change accordingly to this new position and
the odometry values will be reset to zero. 


NOTE: During the compilation you can get an error. This would mean that you have in your computer a higher version of the library libfltk1.1-dev installed. If so, you should install manually the library: sudo apt-get install libfltk1.1-dev. Then, try to compile the catkin workspace again.
