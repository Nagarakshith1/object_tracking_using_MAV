#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
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

void drone_odom_callback(const nav_msgs::Odometry &msg){
	t_drone << msg.pose.pose.position.x,
			   msg.pose.pose.position.y,
			   msg.pose.pose.position.z;
	Eigen::Quaterniond q (msg.pose.pose.orientation.w,
						  msg.pose.pose.orientation.x,
						  msg.pose.pose.orientation.y,
						  msg.pose.pose.orientation.z
						  );
	R_drone = q;

}

void apriltag_pose_callback(const apriltag_msgs::ApriltagPoseStamped &msg){
	t_obj_drone << msg.posearray.poses[0].position.x,
			   	   msg.posearray.poses[0].position.y,
			       msg.posearray.poses[0].position.z;
	Eigen::Quaterniond q (msg.posearray.poses[0].orientation.w,
						  msg.posearray.poses[0].orientation.x,
						  msg.posearray.poses[0].orientation.y,
						  msg.posearray.poses[0].orientation.z
						  );
	R_obj_drone = q;		  

}

int main(int argc,char **argv){
	ros::init(argc,argv,"obj_traj");
	ros::NodeHandle n;

	ros::Subscriber drone_odom_sub = n.subscribe("/drone/odom",1,&drone_odom_callback);
	ros::Subscriber april_pose_odom = n.subscribe("/apriltag_pose_estimator/apriltag_poses",1,&apriltag_pose_callback);

	ros::Publisher obj_traj_pub = n.advertise<obj_traj_est::traj_msg>("/obj_traj_coeff",1);

	obj_traj_est::traj_msg msg;

	int planning_rate = 1;
	int planning_horizon = 1;

	ros::Rate replan_rate(planning_rate);

	double p_cov[]={1e8,1e8,1e8,1e8,1e8,1e8,1e8,1e8,1e8};
	auto P = Eigen::Matrix<double,1,n_states>(p_cov).asDiagonal();

	double r_cov[]={0.2,0.2,0.2};
	auto R = Eigen::Matrix<double,1,n_meas>(r_cov).asDiagonal();

	double q_cov[]={5e-4,5e-4,5e-4,5e-4,5e-4,5e-4,5e-4,5e-4,5e-4};
	auto Q = Eigen::Matrix<double,1,n_states>(q_cov).asDiagonal();

	KalmanFilter kf(x_init_target,Q,R,P);
	Traj tr(obj_coeff,planning_horizon);

	while(ros::ok()){

		t_obj_world = R_drone*t_obj_drone + t_drone;
		x_target = kf.MeasurementUpdate(t_obj_world);
		obj_coeff << x_target[0]+x_target[3]*planning_rate ,x_target[1]+x_target[4]*planning_rate,x_target[2]+x_target[5]*planning_rate,
					 x_target[3],x_target[4],x_target[5],
					 0,0,0,
					 0,0,0,
					 0,0,0,
					 0,0,0,
					 0,0,0,
					 0,0,0;

		tr.set_coefficients(obj_coeff);
		msg = tr.to_rosmsg();
		obj_traj_pub.publish(msg);

		ros::spinOnce();
		replan_rate.sleep();
	}


}