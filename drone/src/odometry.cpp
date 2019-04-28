#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "ardrone_autonomy/Navdata.h"

#include <sstream>
#include "iostream"

#define PI 3.14159265

ros::Publisher position_pub;
geometry_msgs::Vector3 pos;
geometry_msgs::Vector3 vel;
geometry_msgs::Vector3 vel_prev;

float lastFrame = 0;

void toEulerAngle(const sensor_msgs::Imu& imu, float& x, float& y, float& z)
{
	double q_x = imu.orientation.x;
	double q_y = imu.orientation.y;
	double q_z = imu.orientation.z;
	double q_w = imu.orientation.w;

	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q_w * q_x + q_y * q_z);
	double cosr_cosp = +1.0 - 2.0 * (q_x * q_x + q_y * q_y);
	x = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q_w * q_y - q_z * q_x);
	if (fabs(sinp) >= 1)
		y = copysign(PI / 2, sinp); // use 90 degrees if out of range
	else
		y = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
	double cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);
	z = atan2(siny_cosp, cosy_cosp);
}

float alphaFilter = 0.1f;
bool flag = true;
void navdataCallback(const ardrone_autonomy::Navdata& navdata)
{
	if(flag == true){
		flag = false;
		lastFrame = 0;
	}
	
	
	vel.x = (alphaFilter * navdata.vx + (1-alphaFilter)*vel.x) * 0.001f;
	vel.y = (alphaFilter * navdata.vy + (1-alphaFilter)*vel.y) * 0.001f;
	vel.z = (alphaFilter * navdata.vz + (1-alphaFilter)*vel.z) * 0.001f;
	vel_prev.x = vel.x;
	vel_prev.y = vel.y;
	vel_prev.z = vel.z;


	ros::Time currFrame = ros::Time::now();
	float  deltaT = currFrame.nsec - lastFrame;
	deltaT = deltaT * 0.000000001f;
	if(deltaT < 0)
		deltaT = 0.0f;

	pos.x += vel.x * deltaT;
	pos.y += vel.y * deltaT;
	pos.z += vel.z * deltaT;

	lastFrame = currFrame.nsec;

	position_pub.publish(pos);
	
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry");

  	ros::NodeHandle n;
	
  	ros::Subscriber navdata_sub = n.subscribe("ardrone/navdata", 1000, navdataCallback);
  	position_pub = n.advertise<geometry_msgs::Vector3>("drone_pos", 1000);
	
  
  	ros::spin();

  return 0;
}




