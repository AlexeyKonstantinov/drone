#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "ardrone_autonomy/Navdata.h"


#include <sstream>
#include <ros/console.h>
#include "iostream"

geometry_msgs::Vector3 pos;
ros::Publisher position_pub;
ros::Publisher takeoff_pub;
ros::Publisher land_pub;
ros::Publisher cmd_vel_pub;



void Fly50Meters()
{
	ros::Duration(3).sleep();
	ROS_DEBUG("Taking off..");
	std_msgs::Empty empty;
	takeoff_pub.publish(empty);
	ros::Duration(5).sleep();
	ROS_DEBUG("Vzleteli");
	geometry_msgs::Twist twist;
	twist.linear.x = 1.0f;
	cmd_vel_pub.publish(twist);
	
}
void posCallback(const geometry_msgs::Vector3& position)
{
	pos = position;
	geometry_msgs::Twist twist;
	
	if (pos.x < 10)
	{
		twist.linear.x = 1.0f;
		cmd_vel_pub.publish(twist);
		position_pub.publish(pos);
	}else
	{
		twist.linear.x = 0.0f;
		cmd_vel_pub.publish(twist);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fly");

  	ros::NodeHandle n;
	
  	ros::Subscriber pos_sub = n.subscribe("drone_pos", 1000, posCallback);
  	position_pub = n.advertise<geometry_msgs::Vector3>("positionInfo", 1000);
	takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
	land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1000);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  	ros::spin();
	

  return 0;
}

