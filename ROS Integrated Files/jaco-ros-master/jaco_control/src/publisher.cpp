

#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <sstream>

int main(int argc, char **argv)
{
    //Global Variables----------SENT FROM ROS MATLAB NODE
    float position_x = -0.54f;
    float position_y = 0.1451f;
    float position_z = 0.3418f;
    float position_theta_x = -2.85942f;
    float position_theta_y = -1.4483f;
    float position_theta_z = 1.82466f;

  ros::init(argc, argv, "Elevator_Button_Publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/elevator_button", 1000);
  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::TwistStamped msg;
    msg.twist.linear.x = position_x;
    msg.twist.linear.y = position_y;
    msg.twist.linear.z = position_z;
    msg.twist.angular.x = position_theta_x;
    msg.twist.angular.y = position_theta_y;
    msg.twist.angular.z = position_theta_z;

    ROS_INFO("SENDING ELEVATOR BUTTON COORDINATES");
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
