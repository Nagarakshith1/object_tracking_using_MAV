#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"move_target");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	ros::Rate loop_rate(20);
	ros::Duration(10.0).sleep(); 
	
	while(ros::ok()){

		geometry_msgs::Twist twist_cmd;

		twist_cmd.linear.x = 0.4;
		twist_cmd.linear.y = 0;
		twist_cmd.linear.z = 0;

		twist_cmd.angular.x = 0;
		twist_cmd.angular.y = 0;
		twist_cmd.angular.z = 0.1;

		pub.publish(twist_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
