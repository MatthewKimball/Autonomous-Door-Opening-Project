%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Name: Door_Open.m
%   Author: Matthew Kimball
%   Version: 1.0.0
%   Description: Peforms Door Opening Manipulation via ROS
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Door_Open (data)

%Define Door Characterisitics
rotation_axis_coordinate_X = data(1).door;
rotation_axis_coordinate_Y = data(2).door;
rotation_axis_coordinate_Z = data(3).door;
Door_Handle_Coordinates = [linear_coordinate_X, linear_coordinate_Y, linear_coordinate_Z]

door_hinge_coordinate_X = data(4).door;
door_hinge_coordinate_Y = data(5).door;
door_hinge_coordinate_Z = data(6).door;
Door_Hinge_Coordinates = [linear_coordinate_X, linear_coordinate_Y, linear_coordinate_Z]

handlelength = data(7).door

handletype = data(8).door % Handle Type knob = 0, handle = 1

hingedirection = data(9).door % Hinge Direction left = 0, right = 1

%Initialise ROS node
node = rosmatlab.node('MATLAB_CONTROLLER');

% Create a publisher for the custom ROS message
publisher = rosmatlab.publisher('/mico_arm_driver/in/door_open', 'jaco_msgs/DoorOpen', node);

% Set rotation axis coordinate components of message
msgHandle = rosmatlab.message('geometry_msgs/Vector3', node);
msgHandle.setX(rotation_axis_coordinate_X);
msgHandle.setY(rotation_axis_coordinate_Y);
msgHandle.setZ(rotation_axis_coordinate_Z);

% Set hinge coordinate components of message
msgHinge = rosmatlab.message('geometry_msgs/Vector3', node);
msgHinge.setX(door_hinge_coordinate_X);
msgHinge.setY(door_hinge_coordinate_Y);
msgHinge.setZ(door_hinge_coordinate_Z);

% Generate Custom Door Characteristic Message
msg = rosmatlab.message('jaco_msgs/DoorOpen', node);

% Set Message Co-ordinates
msg.setHandleAxisRotation (msgHandle);
msg.setDoorHinge (msgHinge);

% Set Message handle Length
msg.setHandleLength(handlelength);

% Set Message Handle type (Knob or Handle)
msg.setHandleType(handletype);

% Set Message Hinge Direction (Left or Right)
msg.setHandleType(hingedirection);

%Publish to the Arm Driver Node
publisher.publish(msg);
pause(1);
end