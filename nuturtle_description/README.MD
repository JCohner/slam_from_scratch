# nuturtle_description package
#### by Josh Cohen

This package serves to provide a way to generate a urdf for and display in a rviz a differential drive robot.   

To launch: `roslaunch nuturtle_description view_diff_drive.launch`

File Structure:
* config:
	* diff_params.yaml - contains parameters for differential drive robot such as wheel radius, base, and width in addition to chassis parameters such as length and thickness
* launch:
	* view_diff_drive.launch - launchfile that loads yaml into parameter server and generates robot description urdf. also starts robot_state and joint_state publishers. finally launches rviz with corresponding config file 
* rviz:
	* view_robot.rviz - holds rviz configuration parameters
* urdf:
	* diff_drive.urdf.xacro - xacro file that is used to generate robot description urdf in launchfile
* package.xml - description of package