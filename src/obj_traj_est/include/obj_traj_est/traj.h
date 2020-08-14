#ifndef TRAJ_H
#define TRAJ_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <iostream>
#include <ros/ros.h>

#include  "obj_traj_est/traj_msg.h"

class Traj{

int _n_order;
int _n_dimensions;
float _planning_horizon;
Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _coefficients;

public:
	Traj(const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &c, float planning_horizon);
	void set_coefficients(const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &c);
	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> get_coefficients();
	obj_traj_est::traj_msg to_rosmsg();


};



#endif