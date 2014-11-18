%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Name: Elevator_Button_Press.m
%   Author: Matthew Kimball
%   Version: 1.0.0
%   Description: Peforms Elevator Button Manipulation via ROS
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Elevator_Button_Press(data)

%Defines Elevator Button Coordinates
linear_coordinate_X = data(1).elevator;
linear_coordinate_Y = data(2).elevator;
linear_coordinate_Z = data(3).elevator;

angular_coordinate_X = -120*(3.14/180);
angular_coordinate_Y = -90*(3.14/180);
angular_coordinate_Z = 60*(3.14/180);

Elevator_Button_Coordinates = [linear_coordinate_X, linear_coordinate_Y, linear_coordinate_Z]

%Initialise ROS node
node = rosmatlab.node('MATLAB_CONTROLLER');

% Create a publisher for the ROS TWISTSTAMPED message
publisher = rosmatlab.publisher('mico_arm_driver/in/elevator_button', 'geometry_msgs/TwistStamped', node);

% Set Linear coordinate components of message
msgLin = rosmatlab.message('geometry_msgs/Vector3', node);
msgLin.setX(linear_coordinate_X);
msgLin.setY(linear_coordinate_Y);
msgLin.setZ(linear_coordinate_Z);

% Set Angular coordinate components of message
msgAng = rosmatlab.message('geometry_msgs/Vector3', node);
msgAng.setX(angular_coordinate_X);
msgAng.setY(angular_coordinate_Y);
msgAng.setZ(angular_coordinate_Z);

% Create the TWIST message itself from the Linear and Angular Components
msg = rosmatlab.message('geometry_msgs/Twist', node);
msg.setLinear (msgLin);
msg.setAngular (msgAng);

% Create the TWISTSTAMPED message
elevator_msg = rosmatlab.message('geometry_msgs/TwistStamped', node);
elevator_msg.setTwist (msg);

%Publish to the Arm Driver Node
publisher.publish(elevator_msg);
pause(1);

end