#include <iostream>
#include <ros/ros.h>
#include "obj_traj_est/traj.h"
#include <geometry_msgs/Twist.h>

int n_order;
int n_dim;
int n_coeff;

double planning_horizon;
double executing_horizon;                  // Time to execute the planned trajectory
double prev_time;
double node_wait_time;                     // Controller timeout
Traj drone_tr;

geometry_msgs::Twist twist_cmd;		       // Msg for the velocity
geometry_msgs::Twist twist_cmd_idle;	   // Msg for the velocity when drone doesn't receive msg for some time
ros::Publisher drone_vel_pub;		       // Drone velocity publisher


void drone_traj_callback(const obj_traj_est::traj_msg &msg) {
	// Controller for the drone
    drone_tr = Traj(msg, planning_horizon);

	prev_time = ros::Time::now().toSec();
	double curr = ros::Time::now().toSec();

	while(ros::ok() && curr - prev_time < executing_horizon) {
		curr = ros::Time::now().toSec();
		Eigen::VectorXd t(1);
		t(0,0) = curr-prev_time;
		auto vel = drone_tr.evaluate(t,1);
		twist_cmd.linear.x = vel(0);
		twist_cmd.linear.y = vel(1);
		twist_cmd.linear.z = vel(2);

		twist_cmd.angular.x = 0;
		twist_cmd.angular.y = 0;
		twist_cmd.angular.z = 0;
		drone_vel_pub.publish(twist_cmd);
	}
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "drone_planner");
	ros::NodeHandle n("~");

	n.getParam("n_order", n_order);
	n.getParam("n_dim", n_dim);
    n.getParam("planning_horizon", planning_horizon);
    n.getParam("executing_horizon", executing_horizon);
    n.getParam("node_wait_time", node_wait_time);

    n_coeff = n_order + 1;

    ros::Subscriber drone_traj_sub = n.subscribe("drone_traj_coeff", 1, &drone_traj_callback);
    drone_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    drone_tr = Traj(Eigen::MatrixXd::Zero(n_coeff,n_dim), planning_horizon);

    // Setting velocities to zero to let the drone hover
    twist_cmd_idle.linear.x = 0;                 
    twist_cmd_idle.linear.y = 0;
    twist_cmd_idle.linear.z = 0;
    twist_cmd_idle.angular.x = 0;
    twist_cmd_idle.angular.y = 0;
    twist_cmd_idle.angular.z = 0;

    ros::Duration(5).sleep(); 
    prev_time = ros::Time::now().toSec();

    while (ros::ok()) {
        ros::spinOnce();

        if ((ros::Time::now().toSec() - prev_time) > node_wait_time) {
            drone_vel_pub.publish(twist_cmd_idle);
        }
    }
}