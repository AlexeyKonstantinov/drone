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

float rotZ;
float desired_rotZ = 0.0f;
float Kp = 0.7f;

int Sign(float Val) {
  if (Val == 0)  return 0;
  if (Val > 0)  return 1;
  else return -1;
}

void navdataCallback(const ardrone_autonomy::Navdata& navdata)
{
	rotZ = navdata.rotZ;
}

void posCallback(const geometry_msgs::Vector3& position)
{
	pos = position;
	geometry_msgs::Twist twist;
	float error = desired_rotZ - rotZ;
	
	if (pos.x < 1)
	{
		twist.linear.x = 1.0f;

		float velZ = Kp * error * 0.01f;
		if (abs(velZ) > 1)
			velZ = 1 * Sign(velZ);
		twist.angular.z = velZ;

		cmd_vel_pub.publish(twist);
		position_pub.publish(pos);
	}else if(rotZ < 40)
	{
		twist.linear.x = 0.0f;
		desired_rotZ = 45.0f;
		twist.linear.x = 0.0f;
		float velZ = Kp * error * 0.01f;
		if (abs(velZ) > 1)
			velZ = 1 * Sign(velZ);
		twist.angular.z = velZ;

		cmd_vel_pub.publish(twist);
		position_pub.publish(pos);
	}else if (pos.x < 2)
	{
		desired_rotZ = 45.0f;
		twist.linear.x = 1.0f;
		float velZ = Kp * error * 0.01f;
		if (abs(velZ) > 1)
			velZ = 1 * Sign(velZ);
		twist.angular.z = velZ;

		cmd_vel_pub.publish(twist);
		position_pub.publish(pos);
	}else
	{
		desired_rotZ = 90.0f;
		twist.linear.x = 0.0f;
		float velZ = Kp * error * 0.01f;
		if (abs(velZ) > 1)
			velZ = 1 * Sign(velZ);
		twist.angular.z = velZ;

		cmd_vel_pub.publish(twist);
		position_pub.publish(pos);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fly");

  	ros::NodeHandle n;
	
  	ros::Subscriber pos_sub = n.subscribe("drone_pos", 1000, posCallback);
	ros::Subscriber navdata_sub = n.subscribe("ardrone/navdata", 1000, navdataCallback);

  	position_pub = n.advertise<geometry_msgs::Vector3>("positionInfo", 1000);
	takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
	land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1000);
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::Duration(2).sleep();
	std_msgs::Empty empty;
	takeoff_pub.publish(empty);
	ROS_INFO("Taking off..");
	ros::Duration(3).sleep();
	ROS_INFO("Vzleteli");
  	ros::spin();

  return 0;
}

