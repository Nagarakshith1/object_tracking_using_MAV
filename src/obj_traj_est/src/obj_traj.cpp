#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <queue>
#include <vector>
#include <cmath>
#include "obj_traj_est/k_filter.h"
#include "obj_traj_est/traj.h"
#include "apriltag_msgs/ApriltagPoseStamped.h"

// Pose of drone in world frame.
Eigen::Matrix<double,3,1> t_drone;
Eigen::Matrix<double,3,3> R_drone;

// Rotation matrix - drone to camera
Eigen::Matrix3d R_drone_cam;

// Pose of the target in drone frame
Eigen::Matrix<double,3,1> t_obj_drone;
Eigen::Matrix<double,3,3> R_obj_drone;

// Pose of the target in world frame
Eigen::Matrix<double,3,1> t_obj_world;
Eigen::Matrix<double,8,3> obj_coeff;

int odom_buffer_size; 				// No of drone odom messages in buffer
int n_obs;							// No of target observation for regression
int p_order;						// Polynomial order of the target trajectory
int drone_p_order;					// Polynommial order of the drone trajectory
double planning_horizon;			// Planning horizon in seconds

std::deque<nav_msgs::Odometry> odoms; // Queue to hold drone odom msgs
Eigen::MatrixXd observations;		  // Matrix to hold the observations
Traj tr;							  // Trajectory class object
obj_traj_est::traj_msg tr_msg; 		  // Target trajectory coefficients msg variable
visualization_msgs::Marker vis_msg;	  // Target trajectory visualization msg variable
ros::Publisher obj_traj_pub;		  // Target trajectory coefficients publisher
ros::Publisher obj_vis_pub; 		  // Target trajectory visualization publisher

Eigen::MatrixXd regression();
Eigen::MatrixXd gen_basis(Eigen::MatrixXd times);


/**
	Stores the latest drone odom msgs in the global variable odoms.
	@param msg The message receieved on the dron odometry topic.
*/
void drone_odom_callback(const nav_msgs::Odometry &msg) {
	if (odoms.size() < odom_buffer_size) {
		odoms.push_back(msg);
	}
	else {

		odoms.pop_front();
		odoms.push_back(msg);
	}
}

/**
	Manipulates the observaions and performs regression to fit the polynomial
	@param msg The april_tag pose message 
*/
void apriltag_pose_callback(const apriltag_msgs::ApriltagPoseStamped &msg) {
	// Get the april_tag pose
	t_obj_drone << msg.posearray.poses[0].position.x,
			   	   msg.posearray.poses[0].position.y,
			       msg.posearray.poses[0].position.z;

	// Keep track of the last time 
	static double last_time = 0;

	// Current time as on the msg header
	double curr_time = msg.header.stamp.toSec();
	
	// Time difference between the current and last message
	double dt = curr_time - last_time;
	last_time = curr_time;
	
	// Find the nearest(in time) drone odom msg
	double min_dtime = std::numeric_limits<double>::infinity();
	double time;
	double dtime;
	nav_msgs::Odometry *odom;
	auto i = odoms.begin();
	
	for (; i != odoms.end(); ++i) {
		time = i->header.stamp.toSec();
		dtime = fabs(curr_time - time);
		if(dtime < min_dtime){
			min_dtime = dtime;
			odom = &*i;
		}
		else{
			break;
		}
	}
	
	t_drone << odom->pose.pose.position.x,
			   odom->pose.pose.position.y,
			   odom->pose.pose.position.z;
	Eigen::Quaterniond q (odom->pose.pose.orientation.w,
						  odom->pose.pose.orientation.x,
						  odom->pose.pose.orientation.y,
						  odom->pose.pose.orientation.z
						  );
	R_drone = q;
	
	// Convert the target position to the world frame
	t_obj_world = R_drone * R_drone_cam * t_obj_drone + t_drone;

	// Counter to see if the n_obs has reached
	static int obs_count = 0;
	if (obs_count < n_obs) {	
		// Latest observation on the top row
		observations.row(n_obs - obs_count - 1) << t_obj_world.transpose(),dt; 
		obs_count++;

		if (obs_count == n_obs) {   
			// If n_obs is reached zero the time of first observation
			observations(n_obs - 1,3) = 0;
			// Update the time of other observations with respect to the first one
			for(int i = n_obs - 3; i >= 0; i--){
				observations(i,3)+=observations(i+1,3);
			}
		}		
		return;
	}
	else {
		// Manipulation to keep the latest observation on the top
		static Eigen::MatrixXd obs_update = Eigen::MatrixXd::Zero(n_obs,4);
		obs_update << t_obj_world.transpose(), (dt+observations(0,3)), observations.block(0, 0, n_obs - 1, 4);
		observations = obs_update;
	}
	
	// Make the recent observation time as zero and shift the past observations negative in time 
	double time_shift = observations(0,3);
	for (int i = 0; i < n_obs; i++) {
		observations(i,3) -= time_shift;
	}
	
	// Perform regression
	auto coeff = regression();
	
	// Adjust to the drone trajectory polynomial order by adding zeros
	Eigen::MatrixXd adj_coeff_mat = Eigen::MatrixXd::Zero(drone_p_order - p_order,3);
	Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(drone_p_order + 1,3);
	coefficients <<adj_coeff_mat,coeff;
	
	// set the new coefficients
	tr = Traj (coefficients, planning_horizon);
	
	vis_msg = tr.to_vismsg();
	obj_vis_pub.publish(vis_msg);

	tr_msg = tr.to_rosmsg();		
	obj_traj_pub.publish(tr_msg);
}

/**
	Linear regression using the Pseudo Inverse
	@return coeff Coefficients of the target trajectory
*/
Eigen::MatrixXd regression() {
	auto basis = gen_basis(observations.col(3));	
	Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(p_order + 1,3);
	// x_pos in first coloumn and y_pos in the second coloumn
	coeff.col(0) = (basis.transpose()*basis).inverse()*basis.transpose()*observations.col(0);
	coeff.col(1) = (basis.transpose()*basis).inverse()*basis.transpose()*observations.col(1);
	return coeff;
}

/**
	Generate the 'A' matrix in A * x = b;
	@param times The time instances to impose the constraints
	@return basis The 'A' matrix
*/
Eigen::MatrixXd gen_basis(Eigen::MatrixXd times) {
	Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(n_obs,p_order+1);	
	for(int i=p_order;i>0;--i){
		basis.col(p_order-i) << times.array().pow(i).matrix();
	}
	basis.col(p_order).setOnes();
	return basis;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "obj_traj");
	ros::NodeHandle n("~");

	ros::Subscriber drone_odom_sub = n.subscribe("odom", 1, &drone_odom_callback);
	ros::Subscriber april_pose_odom = n.subscribe("apriltag_poses", 1, &apriltag_pose_callback);

	obj_traj_pub = n.advertise<obj_traj_est::traj_msg>("obj_traj_coeff", 1);
	obj_vis_pub = n.advertise<visualization_msgs::Marker>("obj_traj_vis", 1);
	
	int planning_rate;

	n.param("odom_buffer_size", odom_buffer_size, 5);
	n.param("n_obs", n_obs, 20);
	n.param("p_order", p_order, 2);
	n.param("drone_p_order", drone_p_order, 7);
	n.param("planning_rate", planning_rate, 20);
	n.param("planning_horizon", planning_horizon, 3.0);

	observations = Eigen::MatrixXd::Zero(n_obs,4);
	tr = Traj(Eigen::MatrixXd::Zero(drone_p_order + 1,3),planning_horizon);

	R_drone_cam<< 1, 0, 0,
			      0,-1, 0,
	    		  0, 0,-1;

	ros::Rate replan_rate(planning_rate);

	while(ros::ok()) {	
		ros::spinOnce();
		replan_rate.sleep();
	}
	return 0;	
}