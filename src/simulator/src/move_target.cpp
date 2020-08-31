#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

double vel_x;
double vel_z;
double vel_y;
double twist_x;
double twist_y;
double twist_z;
int freq;
double wait_time;

int main(int argc,char **argv){

	ros::init(argc,argv,"move_target");
	ros::NodeHandle n("~");

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

	n.getParam("/object/init_wait_time",wait_time);
	n.getParam("/object/frequency",freq);
	n.getParam("/object/linear_x",vel_x);
	n.getParam("/object/linear_y",vel_y);
	n.getParam("/object/linear_z",vel_z);
	n.getParam("/object/twist_x",twist_x);
	n.getParam("/object/twist_y",twist_y);
	n.getParam("/object/twist_z",twist_z);

	ros::Rate loop_rate(freq);
	ros::Duration(wait_time).sleep(); 
	
	while(ros::ok()){

		geometry_msgs::Twist twist_cmd;

		twist_cmd.linear.x = vel_x;
		twist_cmd.linear.y = vel_y;
		twist_cmd.linear.z = vel_z;

		twist_cmd.angular.x = twist_x;
		twist_cmd.angular.y = twist_y;
		twist_cmd.angular.z = twist_z;

		pub.publish(twist_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
