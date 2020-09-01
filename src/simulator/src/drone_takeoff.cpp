#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

int main(int argc,char **argv) {
	ros::init(argc, argv, "drone_takeoff");
	ros::NodeHandle n;
	ros::ServiceClient motor_client = n.serviceClient<std_srvs::SetBool>("/ddk/mav_services/motors");
	ros::ServiceClient takeoff_client = n.serviceClient<std_srvs::Trigger>("/ddk/mav_services/takeoff");

	motor_client.waitForExistence();
	takeoff_client.waitForExistence();
	std_srvs::SetBool motor_srv;
	motor_srv.request.data = true;

	std_srvs::Trigger takeoff_srv;
	if (motor_client.call(motor_srv)) {
		ros::Duration(4).sleep(); 
		takeoff_client.call(takeoff_srv);
	}

	return 0;
}