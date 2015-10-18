Table broadcaster
=================

Objective
----------

Set the origin and orientation of a table or any other world object given the data published by 
the optitrack node. 

The use of this node is to only call this service once at the begining of your simulation such to 
set the positions of the world objects, such as cuboards, book shelfs, ect...

If the real world physical object is subsequently moved calling the service again will update the 
position in simulation.


Input arguments (arg)
---------------------

	-fixed_frame: the link name of your root reference frame your world, typically
		      it is /world. Check in rviz the variable "Fixed Frame" is set to
		      and this will be the "-fixed_frame" input argument.

	-target_frame_vision: the link name outputed by OptiTrack which you defined 
			      when you setup up your optitrack world object. Typically
			      it is root_"name chosen" for example root_table.

	-target_frame_rviz: the target frame you want to update in rviz.


	-rate: the frequency of the node, default value is 100Hz.


Services
---------

set_table_tf: set the origin and orientation of the table provided by the user, the message type is of:
	      geometry_msgs/Transform transform


table_cmd: 
	
	- set_location : listens to OptiTrack and sets the position and orientation of the table once.


