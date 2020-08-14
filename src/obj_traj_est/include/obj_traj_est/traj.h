#ifndef TRAJ_H
#define TRAJ_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include  "obj_traj_est/traj_msg.h"

class Traj{

int _n_order;
int _n_dimensions;
float _planning_horizon;
Eigen::MatrixXd _coefficients;


public:
	Traj(const Eigen::MatrixXd &c, float planning_horizon);
	void set_coefficients(const Eigen::MatrixXd &c);
	Eigen::MatrixXd get_coefficients();
	obj_traj_est::traj_msg to_rosmsg();
	Eigen::MatrixXd evaluate(Eigen::VectorXd times);
	visualization_msgs::Marker to_vismsg();

};



#endif