#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <queue>
#include <vector>
#include <cmath>
#include "obj_traj_est/k_filter.h"
#include "obj_traj_est/traj.h"
#include "apriltag_msgs/ApriltagPoseStamped.h"

Eigen::Matrix<double,3,1> t_drone;
Eigen::Matrix<double,3,3> R_drone;

Eigen::Matrix<double,3,1> t_obj_drone;
Eigen::Matrix<double,3,3> R_obj_drone;

Eigen::Matrix<double,9,1> x_init_target;
Eigen::Matrix<double,9,1> x_target;

Eigen::Matrix<double,3,1> t_obj_world;
Eigen::Matrix<double,8,3> obj_coeff;

const int odom_buffer_size = 5;
const int n_obs =10;

const int p_order = 2;
const int drone_p_order = 7;

double planning_horizon;

std::deque<nav_msgs::Odometry> odoms;
Eigen::Matrix<double,n_obs,4> observations;

Traj tr(Eigen::Matrix<double,drone_p_order+1,3>::Zero(),1);
Eigen::Matrix<double,p_order+1,3> regression();
Eigen::Matrix<double,n_obs,p_order+1> gen_basis(Eigen::Matrix<double,n_obs,1> times);

void drone_odom_callback(const nav_msgs::Odometry &msg){
	
	if(odoms.size()<odom_buffer_size){
		odoms.push_back(msg);
	}
	else{

		odoms.pop_front();
		odoms.push_back(msg);
	}
	
}

void apriltag_pose_callback(const apriltag_msgs::ApriltagPoseStamped &msg){

	
	t_obj_drone << msg.posearray.poses[0].position.x,
			   	   msg.posearray.poses[0].position.y,
			       msg.posearray.poses[0].position.z;

	static double last_time = 0;
	double curr_time = msg.header.stamp.toSec();
	double dt = curr_time - last_time;
	last_time += dt;

	// Convert to the world frame.
	// Find the nearest drone odom.
	double min_dtime = std::numeric_limits<double>::infinity();
	double time;
	double dtime;
	nav_msgs::Odometry *odom;
	auto i=odoms.begin();
	
	for(;i != odoms.end();++i){
		time = i->header.stamp.toSec();
		dtime = fabs(curr_time - time);
		if(dtime<min_dtime){
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
	// ROS_ERROR_STREAM("a" << t_drone);
	
	// odoms.erase(odoms.begin(),i);


	t_obj_world = R_drone*t_obj_drone + t_drone;

	// ROS_ERROR_STREAM("a" << t_obj_world);
	static int obs_count=0;
	if(obs_count<n_obs){
		observations.row(n_obs-obs_count-1) << t_obj_world.transpose(),dt;
		// ROS_ERROR_STREAM("o" << observations);
		obs_count++;
		if(obs_count == n_obs){
			observations(n_obs-1,3) = 0;
			for(int i=n_obs-3;i>=0;i--){
				observations(i,3)+=observations(i+1,3);
			}
		}
		ROS_ERROR_STREAM("." << observations);
		return;
	}
	else{
		static Eigen::Matrix<double,n_obs,4> obs_update;
		obs_update << t_obj_world.transpose(),(dt+observations(0,3)),observations.block<n_obs-1,4>(0,0);
		observations = obs_update;
	}
	
	// double offset = observations(n_obs-1,3); 
	// observations(n_obs-1,3) = 0;
	// for(int i=0;i<n_obs-1;i++){
	// 	observations(i,3)-= offset;
	// }
	
	double time_shift = observations(0,3);
	for(int i=0;i<n_obs;i++){
		observations(i,3)-= time_shift;
	}
	

	// observations.col(3)-= observations(n_obs-1,3)*Eigen::Matrix<double,n_obs,1>::Ones();
	// observations.col(3)-= observations(0,3)*Eigen::Matrix<double,n_obs,1>::Ones();
	ROS_ERROR_STREAM("." << observations);
	if((observations.col(0).sum()+observations.col(1).sum()) == 0){ 
		ROS_INFO("Object not moving");
		return;
	}

	auto coeff = regression();
	// Adjust to the drone trajectory polynomial order
	
	auto adj_coeff_mat = Eigen::Matrix<double,drone_p_order - p_order,3>::Zero();

	Eigen::Matrix<double,drone_p_order+1,3> coefficients;
	coefficients <<adj_coeff_mat,coeff;
	tr = Traj(coefficients,planning_horizon);

}

Eigen::Matrix<double,p_order+1,3> regression(){
	auto basis = gen_basis(observations.col(3));
	
	Eigen::Matrix<double,p_order+1,3> coeff = Eigen::Matrix<double,p_order+1,3>::Zero();
	coeff.col(0) = (basis.transpose()*basis).inverse()*basis.transpose()*observations.col(0);
	coeff.col(1) = (basis.transpose()*basis).inverse()*basis.transpose()*observations.col(1);
	return coeff;
}

Eigen::Matrix<double,n_obs,p_order+1> gen_basis(Eigen::Matrix<double,n_obs,1> times){
	Eigen::Matrix<double,n_obs,p_order+1> basis;
	
	for(int i=p_order;i>0;--i){
		basis.col(p_order-i) << times.array().pow(i).matrix();
	}
	basis.col(p_order).setOnes();
	return basis;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"obj_traj");
	ros::NodeHandle n;

	ros::Subscriber drone_odom_sub = n.subscribe("/drone/odom",1,&drone_odom_callback);
	ros::Subscriber april_pose_odom = n.subscribe("/apriltag_pose_estimator/apriltag_poses",1,&apriltag_pose_callback);

	ros::Publisher obj_traj_pub = n.advertise<obj_traj_est::traj_msg>("/obj_traj_coeff",1);

	obj_traj_est::traj_msg msg;

	int planning_rate = 1;
	planning_horizon = 1;

	ros::Rate replan_rate(planning_rate);

	while(ros::ok()){

		msg = tr.to_rosmsg();
		obj_traj_pub.publish(msg);

		ros::spinOnce();
		replan_rate.sleep();

	}
	
	}