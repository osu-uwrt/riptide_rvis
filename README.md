# riptide_rviz

This is a ROS2 package designed around controlling the OSU-UWRT Ritpide software stack onboard our robot. 
The plugin set is able to start code on remote computers, control the state of the vehicle, manipulate onboard actuators, 
visualize and execute autonomous behavior, and display important telemetry in a centralized location for an operator.

This is the configuration that is saved with the repo. This configurations in only reccomended, but can be modified as needed.
<img width="1512" alt="image" src="https://user-images.githubusercontent.com/5054270/194714880-4972024b-ce7f-43ab-8435-2ca03f239d07.png">


## Control Panel
<img width="389" alt="image" src="https://user-images.githubusercontent.com/5054270/194715024-bd59b701-42c5-4737-a28b-004843d4dd3e.png">


The control panel focuses on commanding the vehicle in the water. It has the following functions:

1. Software kill management: The enable and disable buttons, in tandem with the require kill check-box govern the kill state of the system. When the disable button is illuminated, the system is active, and the robot may move. When the enable button is illminated, the robot is in a killed state and cannot move. The require kill checkbox allows the robot to come untehered from the system without the kill switch timeout mechanism triggering. this should only be used for runs where the riptide vehicle will not be tethered during its run.

2. Control mode management: The 4 controller state buttons in tandem govern the next control mode to be requested of the robot. For the velocity and position modes, the information in the text entry fields below is used to command the robot. For Teleoperation, and Feedforward, simply the state set command is used.

3. Control command management: The current command in both velocity and position control modes is grabbed from the text entry fields before sending the command. Should any of the fields fail to be parsed, the send command button will disable and become red for 1 seccond to indicate a parse failure. Addtionally, the current button can be used while connected to the robot, to load the current odometry readout (shown in the text boxes above the command) into the command field. This allows for easy updates to the robotcs command from its current location.

4. Special features: The plugin also has a dive in place button that is only activated when the vehicle is located within a half meter of the surface (z=0.0). When pressed, it will send a position command to the robot to tell it to dive 0.75m depth in the same x and y position while also preserving the yaw angle. Roll and pitch will become zeroed out so the vehicle sits level at the end of the move.


## Actuator Panel


## Bringup Panel


## Mission Panel
