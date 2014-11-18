%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Name: Calibration.m
%   Author: Matthew Kimball
%   Version: 1.0.0
%   Description: Determines Kinova Arm and Kinect Offsets
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Calibration(data)

%Define Kinect Coordinates Variables
linear_coordinate_X = data(1).doorcorner;
linear_coordinate_Y = data(2).doorcorner;
linear_coordinate_Z = data(3).doorcorner;
Kinect_Calibration_Coordinates = [linear_coordinate_X, linear_coordinate_Y, linear_coordinate_Z]

%Initialise ROS node
node = rosmatlab.node('MATLAB_CONTROLLER');

% Create a publisher for the ROS Vector3 message
publisher = rosmatlab.publisher('mico_arm_driver/in/kinect_arm_calibration', 'geometry_msgs/Vector3', node);

% Set Linear coordinate components of message
msg = rosmatlab.message('geometry_msgs/Vector3', node);
msg.setX(linear_coordinate_X);
msg.setY(linear_coordinate_Y);
msg.setZ(linear_coordinate_Z);

%Publish to the Arm Driver Node
publisher.publish(msg);
pause(1);

end