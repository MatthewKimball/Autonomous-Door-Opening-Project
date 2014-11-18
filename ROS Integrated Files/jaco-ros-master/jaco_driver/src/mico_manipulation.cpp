//============================================================================
// Name        : mico_manipulation.cpp
// Author      : Matthew Kimball, Tai Trinh
// Version     : 1.0.0
// Copyright   : Your copyright notice
// Description : Performs Room Access Manipulation Routines
//============================================================================

#include "jaco_driver/mico_manipulation.h"
#include <string>
#include <vector>

#define PI 3.14159265359


namespace jaco
{
/*
*********************************************************************************************************
*                                              Kinect/Arm Calibration Function
*********************************************************************************************************
*/
void MicoManip::Coordinate_Calibration(const CartesianInfo &kinect_position_) {
int value = 0;
    CartesianInfo commandMin;
    CartesianInfo commandMax;
    commandMin.InitStruct();
    commandMax.InitStruct();
    CartesianPosition data;

    jaco_api_.moveHome();
    jaco_api_.initFingers();
    jaco_api_.startForceControl();

   commandMin.X = 1.0f;
    commandMax.X = 1.5f;
    commandMin.Y = 1.0f;
    commandMax.Y = 1.5f;
    commandMin.Z = 1.0;
    commandMax.Z = 1.5f;
    ROS_INFO("Cartesian force X,Y,Z min and max has been set.");

    jaco_api_.setCartesianForceMinMax(commandMin, commandMax);

	ROS_INFO("Move Kinova Arm to calibration position.");
	ROS_INFO("Then press ENTER to calibrate...");

fflush(stdout);
do value = getchar();while((value != '\n') && (value != EOF));

jaco_api_.getCartesianPosition(data);

//Calculating offset
calibration_position_.X = data.Coordinates.X -  kinect_position_.X;
calibration_position_.Y = data.Coordinates.Y -  kinect_position_.Y;
calibration_position_.Z = data.Coordinates.Z -  kinect_position_.Z;

ROS_INFO("Calibration Offset Values: %f, %f, %f", calibration_position_.X, calibration_position_.Y, calibration_position_.Z);

ROS_INFO("DONE");
jaco_api_.stopForceControl();
jaco_api_.moveHome();

}

/*
*********************************************************************************************************
*                                              Elevator Button Press Function
*********************************************************************************************************
*/
void MicoManip::elevatorButton(const CartesianInfo &position)
{
    //Global Variables----------SENT FROM ROS MATLAB NODE
    float x = position.X;
    float y = position.Y;
    float z = position.Z;
    float theta_x = position.ThetaX;
    float theta_y = position.ThetaY;
    float theta_z = position.ThetaZ;

ROS_INFO("%f, %f, %f, %f, %f, %f",position.X, position.Y, position.Z, position.ThetaX, position.ThetaY, position.ThetaZ);


    CartesianPosition data;

    jaco_api_.moveHome();
    jaco_api_.initFingers();

    TrajectoryPoint pointToSend;
    pointToSend.InitStruct();

    CartesianInfo commandMin;
    CartesianInfo commandMax;
    commandMin.InitStruct();
    commandMax.InitStruct();



    pointToSend.Position.HandMode = POSITION_MODE;
    pointToSend.Position.Type = CARTESIAN_POSITION;
    pointToSend.LimitationsActive = 1;

    //Note that the first position has a velocity limitation of 8 cm/sec
    pointToSend.Limitations.speedParameter1 = 0.08;
    pointToSend.Limitations.speedParameter2 = 0.7;

		pointToSend.Position.CartesianPosition.X = x;
		pointToSend.Position.CartesianPosition.Y = y;
		pointToSend.Position.CartesianPosition.Z = z;
		pointToSend.Position.CartesianPosition.ThetaX = theta_x;
		pointToSend.Position.CartesianPosition.ThetaY = theta_y;
		pointToSend.Position.CartesianPosition.ThetaZ = theta_z;
		pointToSend.Position.Fingers.Finger1 = 7350.0f;
		pointToSend.Position.Fingers.Finger2 = 7350.0f;
/*
    commandMin.X = 1.0f;
    commandMax.X = 1.5f;
    commandMin.Y = 1.0f;
    commandMax.Y = 1.5f;
    commandMin.Z = 1.0;
    commandMax.Z = 1.5f;
    ROS_INFO("Cartesian force X,Y,Z min and max has been set.");


    jaco_api_.setCartesianForceMinMax(commandMin, commandMax);*/

    jaco_api_.startForceControl();

    jaco_api_.sendAdvanceTrajectory(pointToSend);
 
   sleep(15);
    jaco_api_.getCartesianPosition(data);

    if ((data.Coordinates.X <= (x-0.02f)) || (data.Coordinates.Y >= (y+0.02f))
            || (data.Coordinates.Z <= (z-0.02)))
    {

        jaco_api_.eraseAllTrajectories();
        jaco_api_.moveHome();

    }
    else
    {
        sleep(2);
        jaco_api_.getCartesianPosition(data);
	jaco_api_.eraseAllTrajectories();
        pointToSend.Position.CartesianPosition.X = (x-0.02f);
        jaco_api_.sendAdvanceTrajectory(pointToSend);
	sleep(1);
	jaco_api_.eraseAllTrajectories();
pointToSend.Position.CartesianPosition.X = (x+0.15f);
jaco_api_.sendAdvanceTrajectory(pointToSend);
sleep(5);
jaco_api_.eraseAllTrajectories();
jaco_api_.stopForceControl();
jaco_api_.moveHome();
sleep(1);

    }


}

/*
*********************************************************************************************************
*                                              Door Opening Coordinate Generation Function
*********************************************************************************************************
*/

void MicoManip::opening_coordinates_generation(bool opening_direction)
{
    //Dummy values
    door_Raxis_Coordinate_x = -0.59;
    door_Raxis_Coordinate_y = 0.29;
    door_Raxis_Coordinate_z = 0.0;

    CartesianPosition data;
    jaco_api_.getCartesianPosition(data);

    int angle_resolution = 0;
    float origin_X_coordinate = 0;
    float origin_Y_coordinate = 0;
    navigation_point[0][0] =data.Coordinates.X;
    navigation_point[0][1] = data.Coordinates.Y;

    //Way point generation for Door Hinge Opening Radius
    if(opening_direction == true)
    {
	 ROS_INFO("Opening direction on the right");

    }
    else
    {
        ROS_INFO("Opening direction on the left");
        origin_Y_coordinate = door_Raxis_Coordinate_y + data.Coordinates.Y;
        origin_X_coordinate = data.Coordinates.X;

    for (int i = 1; i < 11; i++) {
        navigation_point_door[i][1] = origin_Y_coordinate* cos(angle_resolution * PI / 180);
        ROS_INFO("%f  ", navigation_point_door[i][1]);

        navigation_point_door[i][0] = origin_X_coordinate* sin(angle_resolution * PI / 180);
        ROS_INFO("%f  \n", navigation_point_door[i][0]);

        angle_resolution -= 2.5;
    }

    }
}

/*
*********************************************************************************************************
*                                              Hinge Door Opening Coordinates Function
*********************************************************************************************************
*/
void MicoManip::turning_coordinates_generation(const DoorInfo &door) {

    //Manual Calibration
    OFFSET_x = -0.530 + 0.488831;
    OFFSET_y = -0.163 + 0.174549;
    OFFSET_z = 0.156572 - 0.130;

    //dummy values

    handle_Raxis_Coordinate_x = door.cartesian_handle_position_.X/1000;
    handle_Raxis_Coordinate_y = door.cartesian_handle_position_.Y/1000;
    handle_Raxis_Coordinate_z = door.cartesian_handle_position_.Z/1000;
    handle_length = door.handle_length_/1000;
    hinge_position = door.opening_direction_;
    handle_type = door.handle_type_;
    safe_gripper_distance = 0.03;

	handle_Raxis_Coordinate_x = -0.53;
	handle_Raxis_Coordinate_y = -0.163;
	handle_Raxis_Coordinate_z = 0.14;
	handle_length = 0.100;
	hinge_position = true;
	handle_type = true;
	safe_gripper_distance = 0.03;

    gripping_Coordinate_x = (handle_Raxis_Coordinate_x
            - OFFSET_x) + safe_gripper_distance;
    gripping_Coordinate_z = (handle_Raxis_Coordinate_z
            + OFFSET_z);

//Determine Opening Type (Door handle or knob)
    if (handle_type == true) {
        gripping_Coordinate_y = (handle_Raxis_Coordinate_y
                - OFFSET_y) + (handle_length * 0.85);
       ROS_INFO("Door Handle");
    } else {
        gripping_Coordinate_y = (handle_Raxis_Coordinate_y
                - OFFSET_y);
 	ROS_INFO("Door Knob");
    }

    //Way point generation for Door Handle Turning Radius
    navigation_point[0][0] = gripping_Coordinate_y;
    navigation_point[0][1] = gripping_Coordinate_z;
    navigation_point[0][2] = (60 * (PI / 180));

    int angle_resolution = 0;

    for (int i = 1; i < 11; i++) {
        navigation_point[i][0] = (handle_Raxis_Coordinate_y
                - OFFSET_y)
                + (handle_length * 0.75)
                        * cos(angle_resolution * PI / 180);
        printf("%f  ", navigation_point[i][0]);
        navigation_point[i][1] = (handle_Raxis_Coordinate_z
                + OFFSET_z)
                + (handle_length * 0.75)
                        * sin(angle_resolution * PI / 180);
        printf("%f  ", navigation_point[i][1]);
        navigation_point[i][2] = navigation_point[i - 1][2]
                + (10 * (PI / 180));
        printf("%f\n", navigation_point[i][2] * (180 / PI));
        angle_resolution -= 10;
    }
    handle_Raxis_Coordinate_x = handle_Raxis_Coordinate_x
            - OFFSET_x;

    }

/*
*********************************************************************************************************
*                                              Door Opening Function
*********************************************************************************************************
*/
void MicoManip::DoorOpen(const DoorInfo &door) {

        jaco_api_.moveHome();
        jaco_api_.initFingers();
        jaco_api_.stopForceControl();
        turning_coordinates_generation(door);
	int value = 0;
        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();
        //-------------------Move to Handle-------------------
        pointToSend.Position.HandMode = POSITION_MODE;
        pointToSend.Position.Type = CARTESIAN_POSITION;
        pointToSend.LimitationsActive = 1;

        //Note that the first position has a velocity limitation of 8 cm/sec
        pointToSend.Limitations.speedParameter1 = 0.08;
        pointToSend.Limitations.speedParameter2 = 0.7;

        //Move to Door Handle Entry position

		pointToSend.Position.CartesianPosition.X =
				gripping_Coordinate_x;
		pointToSend.Position.CartesianPosition.Y =
				gripping_Coordinate_y;
		pointToSend.Position.CartesianPosition.Z =
				gripping_Coordinate_z;
        pointToSend.Position.CartesianPosition.ThetaX = (-120 * (PI / 180));
        pointToSend.Position.CartesianPosition.ThetaY = (-90 * (PI / 180));
        pointToSend.Position.CartesianPosition.ThetaZ = (60 * (PI / 180));

        pointToSend.Position.Fingers.Finger1 = 6400.0f;
        pointToSend.Position.Fingers.Finger2 = 6400.0f;

        ROS_INFO("Sending Initial Door Opening trajectory");

        jaco_api_.sendAdvanceTrajectory(pointToSend);

	

        //Delay
        sleep(5);
        //Open Gripper
        pointToSend.Position.Fingers.Finger1 = 4267.0f;
        pointToSend.Position.Fingers.Finger2 = 4267.0f;

        ROS_INFO("Open Gripper");
        jaco_api_.sendAdvanceTrajectory(pointToSend);


       //Move to Handle Grabbing Position
        pointToSend.Position.CartesianPosition.X =
                (handle_Raxis_Coordinate_x) - 0.030;

        ROS_INFO("Sending Handle Grabbing trajectory");
        jaco_api_.sendAdvanceTrajectory(pointToSend);

        //Delay
                sleep(2);

        //Close Gripper
        pointToSend.Position.Fingers.Finger1 = 7350.0f;
        pointToSend.Position.Fingers.Finger2 = 7350.0f;

        ROS_INFO("Close Grippers" );
        jaco_api_.sendAdvanceTrajectory(pointToSend);

        //Delay
        sleep(1);

        //-------------------------Turn Handle-----------------------
        pointToSend.Limitations.speedParameter1 = 0.05;
        pointToSend.Limitations.speedParameter2 = 0.1;
/*
        //api.stopForceControl();
        jaco_api_.startForceControl();
         CartesianInfo commandMin;
         CartesianInfo commandMax;
         commandMin.InitStruct();
         commandMax.InitStruct();

         commandMin.X = 150.0f;
         commandMin.Y = 150.0f;
         commandMin.Z = 150.0f;
         commandMin.ThetaX = 150.0f;
         commandMin.ThetaY = 150.0f;
         commandMin.ThetaZ = 150.0f;

         commandMax.X = 200.0f;
         commandMax.Y = 200.0f;
         commandMax.Z = 200.0f;
         commandMax.ThetaX = 200.0f;
         commandMax.ThetaY = 200.0f;
         commandMax.ThetaZ = 200.0f;
*/
         jaco_api_.setCartesianForceMinMax(commandMin, commandMax);
         sleep(5);

        CartesianPosition force_data;
        jaco_api_.getCartesianForce(force_data);

        //Send Handle opening coordinates
                for (int i = 1; i < 11; i++) {
                    pointToSend.Position.CartesianPosition.Y =
                            navigation_point[i][0];
                    ROS_INFO("Send: %f  ", navigation_point[i][0]);
                    pointToSend.Position.CartesianPosition.Z =
                            navigation_point[i][1];
                   ROS_INFO("%f  ", navigation_point[i][1]);
                    pointToSend.Position.CartesianPosition.ThetaZ =
                            navigation_point[i][2]- (10 * (PI / 180));
                    ROS_INFO("%f\n", navigation_point[i][2]);
                    jaco_api_.sendAdvanceTrajectory(pointToSend);
                }

        //Check the Torque applied to the handle
                float init = force_data.Coordinates.Y;
                while (force_data.Coordinates.ThetaY < (1.20)) { //Torque value found from experimentation...Needs to be smarter

                    jaco_api_.getCartesianForce(force_data);
                    ROS_INFO("  Force Y Theta: ");

                }

                jaco_api_.eraseAllTrajectories();

//Open the door outwards
		api.getCartesianPosition(position_data);

						//float init_X_axis_force = force_data.Coordinates.X;


						pointToSend.Position.CartesianPosition.X = position_data.Coordinates.X + 0.05;
						pointToSend.Position.CartesianPosition.Y = position_data.Coordinates.Y;
						pointToSend.Position.CartesianPosition.Z = position_data.Coordinates.Z;
						pointToSend.Position.CartesianPosition.ThetaZ = position_data.Coordinates.ThetaZ;
						pointToSend.Position.CartesianPosition.ThetaY = position_data.Coordinates.ThetaY;
						pointToSend.Position.CartesianPosition.ThetaX = position_data.Coordinates.ThetaX;
						pointToSend.Position.Fingers.Finger1 = 7350.0f;
						pointToSend.Position.Fingers.Finger2 = 7350.0f;

						api.sendAdvanceTrajectory(pointToSend);

                sleep(15);

                //---------Test Door Opening Direction------//

                                //Close Grippers
                                CartesianPosition position_data;
                                jaco_api_.getCartesianPosition(position_data);

                ROS_INFO("End of Door Opening Application");

            }



}
